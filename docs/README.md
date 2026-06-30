# 26-v2-r2-ind-mcu-chassis

Robocon 2026 `26-v2-r2` 独立升降麦克纳姆底盘下位机工程，运行在 `STM32F407` 平台上，
使用 `STM32CubeMX + HAL + FreeRTOS` 组织硬件初始化、周期调度与协议处理。

这个根项目的定位不是“通用驱动库”，而是一个面向当前整车形态的**整机集成工程**：

- 向下对接电机、陀螺仪、GPIO、CAN、UART 等硬件；
- 向上承接上位机的位姿与控制命令；
- 在本地下位机内完成连接握手、校准、定位、运动控制与动作编排；
- 通过统一反馈帧把底盘姿态、升降高度、动作状态、连接状态回传给上位机。

## 项目职能总览

| 模块 | 主要文件 | 职能摘要 |
| --- | --- | --- |
| 编译期形态裁剪 | `UserCode/project_parts.hpp` | 统一控制底盘、升降、夹具、陀螺仪、上位机定位、上位机控制等能力开关，并派生出 `EnableStepAction`、`EnableUpperHostProtocol` 等系统能力。 |
| 启动与实时调度 | `UserCode/app.cpp`、`UserCode/system.hpp` | 组织系统初始化顺序、连接等待、校准等待、定位初始化等待，并以 `1kHz / 500Hz / 100Hz` 节拍驱动控制链路。 |
| 设备接入与总线管理 | `UserCode/device.*`、`UserCode/connection.*` | 完成 UART 陀螺仪、UART 上位机、CAN 电机映射、周期发包与连接状态表维护。 |
| 底盘 / 升降 / 定位 | `UserCode/chassis/` | 用统一的 `IndLiftMecanum4` 运动对象承载四轮底盘与前后双侧升降；根据开关选择 `JustEncoder` 或 `LocEKF`，并驱动 `Master` 控制器。离线轨迹跟随器 `OfflineTrajectoryFollower` 支持编译期静态轨迹的 4-DOF 回放。 |
| 夹取机构 | `UserCode/grip/` | 管理 arm / turn 双电机轨迹、夹爪 GPIO、校准流程，若干常用姿态切换，以及 `SpearGrab` 和 `KfsStore` 两个高层动作组。 |
| 上位机协议 | `UserCode/protocol/` | 解析固定帧格式的上位机命令，维护对时、位姿接入、控制指令、反馈帧发送与动作状态表。 |
| 静态内存模型 | `UserCode/arena.cpp` | 通过静态 arena 覆盖全局 `new/delete`，适配下位机长期运行场景；设计上不依赖运行时堆释放。 |

## 根项目实际负责什么

从代码职责上看，这个工程承担的是“**整车下部执行层**”而不是“纯驱动层”或“纯算法层”：

1. **把硬件资源装配成可控对象**
   - `UART2`：航向陀螺仪 `HWT101CT`
   - `UART3`：上位机串口
   - `CAN1 / CAN2`：底盘轮电机、升降电机、夹具电机

2. **把项目形态收敛为可裁剪能力**
   - 是否启用轮式底盘
   - 是否启用升降机构
   - 是否启用 grip
   - 是否启用本地 EKF / 纯编码器定位
   - 是否启用上位机定位输入
   - 是否启用上位机控制指令

3. **在启动阶段保证“可工作”**
   - 按顺序初始化设备、底盘、协议、夹具；
   - 等待已启用对象全部在线；
   - 执行底盘 / 升降 / grip 校准；
   - 对于上位机定位模式，等待首个满足接入条件的位姿后再完成定位与控制器构造。

4. **在运行阶段保证“能闭环”**
   - 周期更新定位、控制器、轨迹、总线发包、连接状态；
   - 根据上位机命令改变底盘目标位姿、车体高度或动作状态；
   - 将底盘姿态、高度、连接状态与动作状态回传给上位机。

## 启动顺序与运行节拍

### 启动顺序

`Init()` 的启动契约如下：

1. `Device::init()`
2. `Chassis::init()`
3. `Protocol::init()`
4. 若启用 grip，再执行 `Grip::init()`
5. 等待所有已启用连接对象上线：`Connection::waitAll()`
6. 使能 `Chassis::motion`，并在启用升降时启动升降校准
7. 若启用 grip，启动 grip 校准
8. 等待底盘 / 升降 / grip 达到 ready
9. 若不依赖上位机初始位姿，则直接 `Chassis::initStandaloneLocCtrl()`
10. 若依赖上位机定位首帧，则等待 `System::Init::postureReceived`
11. 最后使能底盘控制器与 grip

### 运行节拍

- `1kHz`：
  - 刷新连接状态表
  - 刷新动作状态表
  - 更新底盘控制链路
  - 发送 CAN 电机控制报文
  - 更新 grip 控制
  - 执行 `service::Watchdog::EatAll()`（吃狗）
- `500Hz`：
  - 通过 `1kHz` 回调内部分频，更新 grip 误差链路
  - 底盘控制器也在 `1kHz` 内部按 `500Hz` 做误差更新
- `100Hz`：
  - 更新底盘与 grip 的轨迹 profile

## 关键业务模块

### 1. 形态裁剪与能力派生

`UserCode/project_parts.hpp` 是本工程唯一的一级能力开关入口。它不只是开关集合，
而是把“模块是否存在”翻译成“系统是否具备某种能力”：

- `EnableChassisMotion`：轮组或升降任一启用，就创建统一的 `IndLiftMecanum4`
- `EnableUpperHostProtocol`：只要上位机定位或上位机控制任一启用，就启动上位机串口协议
- `EnableJustEncoderLocalization`：有底盘、无陀螺仪
- `EnableEkfLocalization`：有底盘、有陀螺仪
- `NeedUpperHostInitPosture`：启用上位机定位时，必须等待首个满足接入条件的位姿
- `EnableStepAction`：同时启用上位机控制、轮式底盘、升降后，台阶动作组才可用

这使得同一套工程可以裁剪成：

- 仅底盘调试
- 底盘 + 升降
- 仅升降
- 仅 grip
- 底盘 + 陀螺仪本地定位
- 底盘 + 陀螺仪 + 上位机定位
- 全量整车模式

### 2. 设备接入与连接管理

`UserCode/device.*` 把板级外设映射为业务对象：

- 四个底盘轮电机：
  - 前轮：`CAN1 id1=1,2`
  - 后轮：`CAN2 id1=3,4`
- 四个升降电机：
  - 前侧：`CAN1 id1=3,4`
  - 后侧：`CAN2 id1=5,6`
- 两个 grip 电机：
  - arm：`CAN2 id1=1`
  - turn：`CAN2 id1=2`
- 陀螺仪：
  - `UART2`
- 上位机串口：
  - `UART3`

`UserCode/connection.*` 维护一个 16-bit 连接位图，只对**当前启用的对象**做等待门控。
也就是说，系统不是盲等全部硬件，而是只等待当前构型真正需要的对象上线。

### 3. 底盘、升降与定位

`UserCode/chassis/` 的核心设计是：**底盘轮组与升降机构共用一个运动对象**
`Chassis::motion`，也就是 `IndLiftMecanum4`。

这意味着：

- 只要轮组或升降任一启用，就会创建 `motion`
- 升降并不是独立的“另一个系统”，而是整合在底盘运动对象内部
- 每个 `LiftSide` 由两台电机组成，按“前侧一组、后侧一组”管理

定位链路则按编译期开关自动切换：

- **无陀螺仪**：`JustEncoder`
- **有陀螺仪，无上位机定位**：本地下位机 `LocEKF`
- **有陀螺仪，且启用上位机定位**：等上位机首个满足接入条件的位姿到来后，再延迟初始化 `LocEKF + Master`

### 4. 台阶动作

`UserCode/chassis/actions/Step.*` 封装了完整的上台阶 / 下台阶动作编排。
它不是单次位置命令，而是把以下要素串成状态机：

- 底盘相对台阶的阶段性目标位姿
- 前后 lift 侧的抬升、收腿、放腿、复位过程
- 支持 200mm / 400mm 台阶高度和 R1 台阶
- 下台阶后是否恢复到底盘正常高度

上位机通过 `0x50..0x5F` 平面台阶命令组或 `0x35/0x36` R1 台阶命令触发，命令字按位编码台阶类型、方向、高度和最终高度。
这部分能力只有在 `EnableStepAction = PC 控制 + 轮式底盘 + 升降` 同时满足时才启用。

### 5. 离线轨迹

`UserCode/chassis/trajectory/OfflineTrajectoryFollower.hpp` 实现了 4-DOF 离线轨迹跟随器，
将编译期静态轨迹点通过 Slave 控制器回放。

- 轨迹点为编译期静态数组，原始 500Hz 由 `planning/` 与 `planning4/` 子项目生成，下位机运行时降采样至 100Hz
- 支持 X 轴镜像翻转（`mirror` 参数）
- 仅在当前位置与轨迹起点误差 < 5cm(x), 5cm(y), 5deg(yaw) 时接受启动
- 启动后底盘进入 Slave 模式；收到其他底盘命令或动作组命令时自动停止并恢复 Master 控制

下表时长为当前 `Config::TrajectoryOffline::kSpeedRatio = 0.7` 下的 100Hz 播放时长：

| `traj_id` | 起点 (x, y, yaw, h) | 终点 (x, y, yaw, h) | 时长 |
|-----------|---------------------|---------------------|------|
| `1` | (8.55, 1.80, 0°, 0.412m) | (10.75, 2.00, -90°, 0.440m) | ~6.78s |
| `2` | (8.55, 3.00, 0°, 0.412m) | (10.75, 2.00, -90°, 0.440m) | ~6.24s |
| `3` | (8.55, 4.20, 0°, 0.412m) | (10.75, 2.00, -90°, 0.440m) | ~5.44s |
| `4` | (6.40, 5.40, 0°, 0.215m) | (11.05, 3.227, 0°, 0.215m) | ~5.37s |

### 6. Grip 夹取机构

`UserCode/grip/` 负责：

- arm / turn 两条电机轨迹
- 夹爪 GPIO 开闭
- 上电校准
- 常用姿态切换：`toStandbyPose()`, `toPrepareGrabPose()`, `toGrabPose()`, `toDockingPose()`, `toKfsPickupPose()`, `toKfsStorePose()`, `toKfsReleasePose()`
- 两个高层动作组：
  - `SpearGrab`：取矛头工作流（chassis + lift + grip 组合动作）
  - `KfsStore`：卷轴暂存/释放工作流（仅 grip + suction）

grip 与底盘 / 升降是解耦的，可以单独启用、单独校准、单独使能。

## 上位机协议总览

上位机协议由 `UserCode/protocol/PCProtocol.*` 实现，核心特征如下：

- 帧头固定为 `0xAA 0xBB`
- 控制帧与反馈帧均为固定长度，带 `CRC16 Modbus`
- 协议当前接入两路串口：`UART3` 为主上位机链路，`UART1` 为辅控制链路
- 串口波特率要求为 `230400`
- 协议内置时钟对齐：主上位机链路上的每帧都会用 `tx_timestamp` 做上下位机时间偏移更新
- **不再保留固定的“前 49 帧仅对时”窗口；CRC 正确的命令会立即进入命令处理**

### 控制帧格式

上位机下发给下位机的控制帧总长度为 **21 Byte**：

- 帧头 `2 Byte`
- 载荷 `17 Byte`
- CRC `2 Byte`

多字节字段均按**高字节在前**传输；`CRC16 Modbus` 的计算范围为
**从 `cmd` 开始到 `tx_timestamp` 结束**，不包含帧头和 CRC 自身。

| 字节偏移 | 字段 | 长度 | 类型 | 含义 |
| --- | --- | --- | --- | --- |
| `0` | `header[0]` | `1` | `uint8` | 固定为 `0xAA` |
| `1` | `header[1]` | `1` | `uint8` | 固定为 `0xBB` |
| `2` | `cmd` | `1` | `uint8` | 指令编号 |
| `3 ~ 14` | `data` | `12` | `byte[12]` | 指令参数区；不同命令复用这 12 个字节 |
| `15 ~ 18` | `tx_timestamp` | `4` | `uint32` | 上位机发送时间戳，用于上下位机时钟对齐 |
| `19 ~ 20` | `crc16` | `2` | `uint16` | `CRC16 Modbus` 校验值 |

#### 控制帧载荷布局

- 控制帧的公共布局固定为：
  - `AA BB | cmd | data[12] | tx_timestamp | crc16`
- 其中 `data[12]` 会按 `cmd` 不同解释为不同参数结构
- 目前所有命令都共享这一套外层封包格式，只是内部参数定义不同

### 上位机控制指令表

| 指令 | 名称 | 生效条件 | 作用 | 数据含义 |
| --- | --- | --- | --- | --- |
| `0x01` | `Ping` | 始终可接收 | 用于对时；当前未实现 Pong | 无实际控制载荷 |
| `0x10` | `StopChassis` | `EnablePcControl` 且 `ctrl != nullptr` | 停止当前底盘控制器输出，同时停止离线轨迹 | 无 |
| `0x11` | `SetChassisHeight` | `EnablePcControl && EnableLift` 且 `motion` 已 ready | 同时设置前后 lift 的目标高度 | `chassisHeight*2000`, `v_max*1000`, `a_max*100`, `j_max`, `linkMode` |
| `0x12` | `SlavePushChassisTrajectory` | 预留 | 已定义帧格式，但下位机暂未实现处理逻辑 | `x*2000`, `y*2000`, `yaw*100`, `vx*2000`, `vy*2000`, `wz*100` |
| `0x13` | `SetMasterChassisTargetCurrentState` | `EnablePcControl && EnableWheelChassis` 且 `ctrl` 已存在 | 以"当前状态"为衔接方式设置底盘目标位姿 | `x*2000`, `y*2000`, `yaw*100`, `xy_vmax*200`, `xy_amax*200`, `yaw_vmax`, `yaw_amax` |
| `0x14` | `SetMasterChassisTargetPreviousCurve` | 同上 | 以上一条曲线为衔接方式设置底盘目标位姿 | 与 `0x13` 完全一致 |
| `0x15` | `SetMasterChassisVelocity` | `EnablePcControl && EnableWheelChassis` 且 `ctrl` 已存在 | 设置底盘车体系速度指令 | `vx*2000`, `vy*2000`, `wz*100`, `reserve x3` |
| `0x16` | `SetGripPose` | `EnablePcControl && EnableGrip` 且 grip 已 enable | 设置 Grip 双轴关节目标，可选控制夹爪 | `arm_pos*100`, `turn_pos*100`, `clawMode(0=保持/1=开/2=闭)`, `reserve x3` |
| `0x17` | `SetGripPresetPose` | `EnablePcControl && EnableGrip` 且 grip 已 enable | 设置 Grip 到预设姿态 | `presetId(0=Standby/1=PrepareGrab/2=Grab/3=Docking/4=KfsPickup/5=KfsStore/6=KfsRelease)`, `reserve x5` |
| `0x18` | `StartOfflineTrajectory` | `EnablePcControl && EnableWheelChassis` 且定位已就绪 | 启动离线轨迹跟随 | `traj_id(1..4)`, `mirror(0=normal/1=mirror)`, `reserve x4` |
| `0x21` | `LidarPosture` | `EnablePcLocalization` | 输入外部位姿观测；首个满足接入条件的包触发延迟定位初始化 | `x*2000`, `y*2000`, `yaw*100`, `lidarTimestamp(ms)` |
| `0x35` | `StepUpR1` | `EnableStepAction` | 触发上 R1 台阶动作 | `stepTarget_x*2000`, `stepTarget_y*2000`, `stepTarget_yaw*100`, `direction(0=Fwd/1=Bwd)`, `reserve x2` |
| `0x36` | `StepUpR1Direct` | `EnableStepAction` | 触发上 R1 台阶动作（直接版，使用当前位置） | `direction(0=Fwd/1=Bwd)`, `reserve x5` |
| `0x50..0x5F` | `StepPose` | `EnableStepAction` | 平面台阶动作组，cmd 低 4 位编码 type/dir/height/finalHeight | `stepTarget_x*2000`, `stepTarget_y*2000`, `stepTarget_yaw*100`, `end_x*2000`, `end_y*2000`, `end_yaw*100` |
| `0x40` | `TakeSpear` | `EnableSpearGrabAction` | 取矛头动作组 | `target_x*2000`, `target_y*2000`, `target_yaw*100`, `end_x*2000`, `end_y*2000`, `end_yaw*100` |
| `0x41` | `TakeSpearById` | `EnableSpearGrabAction` | 通过固定矛位索引取矛头 | `spearId(0..5)`, `end_x*2000`, `end_y*2000`, `end_yaw*100`, `reserve x2` |
| `0x42` | `StoreKFS` | `EnableKfsAction` | 卷轴临时存放动作组 | 无 |
| `0x43` | `ReleaseKFS` | `EnableKfsAction` | 卷轴释放动作组 | 无 |
| `0x44` | `SetGripSuction` | `EnableGripSuction` | 控制 Grip 吸盘开关 | `on(0=关/非0=开)`, `reserve x5` |
| `0x45` | `SetAbdomenSuction` | `EnableAbdomenSuction` | 控制腹部吸盘开关 | `on(0=关/非0=开)`, `reserve x5` |
| `0x46` | `SetGripClaw` | `EnablePcControl && EnableGrip` 且 grip 已 enable | 独立控制夹爪开合 | `clawMode(0=开/1=闭)`, `reserve x5` |

#### 指令补充说明

- `SetChassisHeight`
  - 高度使用"底盘离地高度"语义，而不是 lift 电机原始位置
  - `v_max / a_max / j_max = 0` 时会退回到带载默认参数
  - `linkMode`：`0/2 = PreviousCurve`，`1 = CurrentState`

- `SetMasterChassisTarget*`
  - 目标位姿中的 `x / y` 单位为米，`yaw` 单位为度
  - 4 个轨迹约束参数采用连续打包的 `uint12`
  - `0x13` 与 `0x14` 的差别仅在轨迹衔接方式

- `SetMasterChassisVelocity`
  - 速度按车体系解释，等价于 `setVelocityInBody(body_velocity, false)`

- `StartOfflineTrajectory`
  - 仅在当前位置与轨迹起点误差 < 5cm(x), 5cm(y), 5deg(yaw) 时接受
  - `mirror=1` 时轨迹按 X 轴对称翻转
  - 启动后自动打断正在执行的离线轨迹

- `LidarPosture`
  - 只接受来自主上位机链路、且当前对时已经稳定的位姿包
  - 首个满足接入条件的外部位姿负责写入 `System::Init::posture`
  - 当系统使用上位机定位模式时，只有收到这个满足接入条件的初始位姿后，底盘定位 / 控制器才算真正初始化完成

- Step 平面台阶命令 (`0x50..0x5F`)
  - cmd 低 4 位编码：`bit3=type(0=up/1=down)`, `bit2=dir(0=Fwd/1=Bwd)`, `bit1=height(0=200/1=400)`, `bit0=finalHeight(0=Low/1=High)`
  - 数据区同时指定台阶边缘作业点 (`stepTarget`) 和动作终点 (`end`)

- `StepUpR1` / `StepUpR1Direct`
  - 终点由下位机内部配置常量 `UpR1EndRelativePos` 生成
  - `StepUpR1Direct` 使用当前位姿作为台阶收腿点，无需上位机指定 `stepTarget`

- Grip 相关命令
  - 所有 Grip 命令均要求 `Grip::grip` 已创建、calibrated 且 enabled()
  - `StoreKFS` / `ReleaseKFS` 需 `EnableKfsAction` 派生能力成立
  - `TakeSpear` / `TakeSpearById` 需 `EnableSpearGrabAction` 派生能力成立

### 反馈帧格式

下位机发给上位机的反馈帧总长度为 **22 Byte**：

- 帧头 `2 Byte`
- 反馈载荷 `18 Byte`
- CRC `2 Byte`

多字节字段同样按**高字节在前**传输；`CRC16 Modbus` 的计算范围为
**从 `timestamp` 开始到 `connection state` 结束**，不包含帧头和 CRC 自身。

| 字节偏移 | 字段 | 长度 | 类型 | 含义 |
| --- | --- | --- | --- | --- |
| `0` | `header[0]` | `1` | `uint8` | 固定为 `0xAA` |
| `1` | `header[1]` | `1` | `uint8` | 固定为 `0xBB` |
| `2 ~ 5` | `timestamp` | `4` | `uint32` | 下位机 `HAL_GetTick()` 时间戳 |
| `6 ~ 7` | `x` | `2` | `int16` | 当前世界坐标系 `x * 2000` |
| `8 ~ 9` | `y` | `2` | `int16` | 当前世界坐标系 `y * 2000` |
| `10 ~ 11` | `yaw` | `2` | `int16` | 当前航向角 `yaw(deg) * 100` |
| `12 ~ 13` | `frontHeight` | `2` | `int16` | 前 lift 对应底盘离地高度 `* 2000` |
| `14 ~ 15` | `rearHeight` | `2` | `int16` | 后 lift 对应底盘离地高度 `* 2000` |
| `16 ~ 17` | `action_state` | `2` | `uint16` | 动作状态位图，编码 Step/Lift/Grip 状态、底盘模式、红外接收和吸盘检测 |
| `18 ~ 19` | `connection_state` | `2` | `uint16` | 连接状态位图 |
| `20 ~ 21` | `crc16` | `2` | `uint16` | `CRC16 Modbus` 校验值 |

#### 反馈帧载荷布局

- 反馈帧的公共布局固定为：
  - `AA BB | timestamp | x | y | yaw | frontHeight | rearHeight | action_state | connection_state | crc16`
- 反馈帧由下位机周期发送，用于向上位机同步：
  - 当前位姿
  - 前后升降高度
  - 动作状态
  - 连接在线状态

## 连接状态位表

`Connection::table` 是上位机判断系统上线状态的标准位图：

| Bit | 含义 |
| --- | --- |
| `0` | `wheel[0]` |
| `1` | `wheel[1]` |
| `2` | `wheel[2]` |
| `3` | `wheel[3]` |
| `4` | `lift[0]`（前侧抬升 0） |
| `5` | `lift[1]`（前侧抬升 1） |
| `6` | `lift[2]`（后侧抬升 0） |
| `7` | `lift[3]`（后侧抬升 1） |
| `8` | `grip_arm` |
| `9` | `grip_turn` |
| `10` | `gyro_yaw` |
| `11` | `GripSuctionPressure`（仅 `EnableGripSuctionPressureSensor` 时有效） |
| `12` | `AbdomenSuctionPressure`（预留，当前未实现） |
| `13` | 预留 |
| `14` | `upper_host_localization`（定位流在线） |
| `15` | `upper_host`（串口链路在线） |

## 目录速览

| 路径 | 说明 |
| --- | --- |
| `UserCode/` | 当前工程唯一的项目业务代码目录 |
| `UserCode/chassis/` | 底盘、升降、定位、控制器接入与台阶动作 |
| `UserCode/grip/` | 夹取机构 |
| `UserCode/protocol/` | 上位机串口协议 |
| `Modules/` | 复用驱动与算法模块仓库 |
| `26-v2-r2-ind-mcu-chassis.ioc` | CubeMX 工程配置 |
| `cmake/wtr_modules.cmake` | 由 `cpkg` 生成的模块接入清单 |

当前根项目通过 `cpkg` / `wtr` 管理并接入的核心模块仓库包括：

- `BasicComponents`
- `ChassisController`
- `MotorDrivers`
- `Sensors`
- `TrajectoryControl`
- `VelocityProfile`

## 构建与开发

### 常用命令

- `cmake --preset Debug`
- `cmake --build --preset Debug`
- `cmake --preset Release && cmake --build --preset Release`
- `stm32tool generate`

### 开发约束

- 项目业务代码只放在 `UserCode/`
- 优先改 `.ioc` 并重新生成，而不是手改 `Core/`、`Drivers/`、`Middlewares/`
- 功能开关统一只改 `UserCode/project_parts.hpp`
- 不要假设 `delete` 会真正释放内存；本工程使用的是单向静态内存 arena
