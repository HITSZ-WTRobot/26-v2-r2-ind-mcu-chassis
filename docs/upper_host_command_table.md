# 上位机指令表

本文档面向上位机开发，描述当前串口下行协议的帧格式、字节序、缩放规则和命令字定义。

## 1. 通用帧格式

下位机当前使用固定长度命令帧：

| 偏移 | 长度 | 类型 | 含义 |
| --- | --- | --- | --- |
| 0 | 1 | `uint8` | 帧头 `0xAA` |
| 1 | 1 | `uint8` | 帧头 `0xBB` |
| 2 | 1 | `uint8` | `cmd` |
| 3 | 12 | `uint8[12]` | 命令数据区 `data[12]` |
| 15 | 4 | `uint32` | 上位机发送时间戳 `tx_timestamp` |
| 19 | 2 | `uint16` | `CRC16` |

- 总帧长固定为 `21` 字节。
- 所有多字节字段都按大端发送，即高字节在前。
- `CRC16` 使用 `CRC16-Modbus` 参数：
  `poly=0x8005, init=0xFFFF, refin=true, refout=true, xorout=0x0000`。
- `CRC16` 的计算范围是从 `cmd` 开始，到 `tx_timestamp` 结束，不包含帧头和 `CRC16` 自身。
- 即使是无参命令，也需要发送完整的 `data[12] + tx_timestamp + CRC16`。

## 2. 缩放规则

| 物理量 | 编码方式 |
| --- | --- |
| 位置 `x/y` | `int16 = value_m * 2000` |
| 偏航 `yaw` | `int16 = value_deg * 100` |
| 线速度 `vx/vy` | `int16 = value_mps * 2000` |
| 角速度 `wz` | `int16 = value_degps * 100` |
| 底盘高度 | `int16 = chassis_height_m * 2000` |
| `xy_vmax / xy_amax` | `uint12`，缩放见各命令说明 |

## 3. 对时与生效时机

- 下位机收到命令帧后，会先用 `tx_timestamp` 做时钟对齐。
- 当前实现中，前 `49` 帧只参与对时，不执行具体控制命令。
- 从第 `50` 帧开始，命令才会实际进入分发逻辑。

## 4. 指令表

| Cmd | 名称 | 数据区格式 | 说明 |
| --- | --- | --- | --- |
| `0x01` | `Ping` | 无 | 当前仅参与对时，暂无 `Pong` 回复。 |
| `0x10` | `StopChassis` | 无 | 停止底盘控制。 |
| `0x11` | `SetChassisHeight` | `chassisHeight(int16), v_max(uint16), a_max(uint16), j_max(uint16), linkMode(uint16)` | 设置底盘离地高度。 |
| `0x12` | `SlavePushChassisTrajectory` | `x(int16), y(int16), yaw(int16), vx(int16), vy(int16), wz(int16)` | 当前固件已保留格式，但处理逻辑仍为空。 |
| `0x13` | `SetMasterChassisTargetCurrentState` | `x(int16), y(int16), yaw(int16), xy_vmax(uint12), xy_amax(uint12), yaw_vmax(uint12), yaw_amax(uint12)` | Master 模式目标位姿，轨迹从当前状态衔接。 |
| `0x14` | `SetMasterChassisTargetPreviousCurve` | 与 `0x13` 相同 | Master 模式目标位姿，轨迹沿上一条曲线衔接。 |
| `0x21` | `LidarPosture` | `x(int16), y(int16), yaw(int16), lidarTimestamp(uint32)` | 上位机定位位姿输入。 |
| `0x30` | `StepUp` | `startDistance(int16), endDistance(int16), direction(uint16), willTake(uint16)` | 上台阶动作组。 |
| `0x31` | `StepUpResume` | 无 | 恢复此前因 `willTake=1` 暂停的上台阶流程。 |
| `0x32` | `StepDown` | `startDistance(int16), endDistance(int16), direction(uint16), shouldReset(uint16)` | 下台阶动作组。 |
| `0x40` | `TakeSpear` | `target_x(int16), target_y(int16), target_yaw(int16), end_x(int16), end_y(int16), end_yaw(int16)` | 直接指定待取矛头位姿和结束位姿。 |
| `0x41` | `TakeSpearById` | `spearId(uint16), end_x(int16), end_y(int16), end_yaw(int16), reserve(uint16), reserve(uint16)` | 通过固定矛位索引启动取矛头。 |
| `0x42` | `StoreKFS` | 无 | 卷轴临时存放动作组。 |
| `0x43` | `ReleaseKFS` | 无 | 卷轴释放动作组。 |

## 5. 各命令细节

### 5.1 `0x11 SetChassisHeight`

数据区：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `chassisHeight` | `int16` | 底盘离地高度，单位 m，编码为 `height * 2000` |
| `v_max` | `uint16` | 抬升最大速度，编码为 `value * 1000` |
| `a_max` | `uint16` | 抬升最大加速度，编码为 `value * 100` |
| `j_max` | `uint16` | 抬升最大加加速度，按原值发送 |
| `linkMode` | `uint16` | `0=PreviousCurve, 1=CurrentState, 2=PreviousCurve` |

补充：

- 上位机发送的是底盘离地高度；下位机内部会再换算成 lift 位置。
- `v_max / a_max / j_max` 为 `0` 时，下位机会回退到带载默认参数。

### 5.2 `0x13 / 0x14` 目标位姿命令

前三个字段直接按 `int16` 发送：

| 字段 | 编码 |
| --- | --- |
| `x` | `x_m * 2000` |
| `y` | `y_m * 2000` |
| `yaw` | `yaw_deg * 100` |

后四个限幅字段为连续打包的 `uint12`，顺序如下：

- `xy_vmax * 200`
- `xy_amax * 200`
- `yaw_vmax`
- `yaw_amax`

打包方式：

```text
[a11:a4] [a3:a0|b11:b8] [b7:b0] [c11:c4] [c3:c0|d11:d8] [d7:d0]
```

### 5.3 `0x21 LidarPosture`

- `lidarTimestamp` 为上位机定位数据原始时间戳，单位 ms。
- 当工程启用了上位机定位模式时，首帧 `LidarPosture` 还承担系统初始位姿的延迟初始化职责。

### 5.4 `0x30 StepUp`

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `startDistance` | `int16` | 开始时车体中心距离台阶边缘的距离，编码为 `m * 2000` |
| `endDistance` | `int16` | 结束时车体中心距离台阶边缘的距离，编码为 `m * 2000` |
| `direction` | `uint16` | `0=Forward`, `1=Backward` |
| `willTake` | `uint16` | `0=连贯上台阶`, `1=中途停下等待取卷轴` |

### 5.5 `0x32 StepDown`

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `startDistance` | `int16` | 开始时车体中心距离台阶边缘的距离，编码为 `m * 2000` |
| `endDistance` | `int16` | 结束时车体中心距离台阶边缘的距离，编码为 `m * 2000` |
| `direction` | `uint16` | `0=Forward`, `1=Backward` |
| `shouldReset` | `uint16` | `1=下台阶后恢复正常高度`, `0=最后一步不回收底盘` |

### 5.6 `0x40 TakeSpear`

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `target_x` | `int16` | 待取矛头世界系 `x` |
| `target_y` | `int16` | 待取矛头世界系 `y` |
| `target_yaw` | `int16` | 待取矛头世界系 `yaw` |
| `end_x` | `int16` | 动作结束世界系 `x` |
| `end_y` | `int16` | 动作结束世界系 `y` |
| `end_yaw` | `int16` | 动作结束世界系 `yaw` |

补充：

- 安全撤离距离固定使用 `Grip::Config::SpearGrab::SafeDistance`，当前值为 `0.20 m`。
- 若 `end_pos` 相对 `target_pos` 的 `x` 方向距离不大于安全撤离距离，下位机会直接忽略该命令。

### 5.7 `0x41 TakeSpearById`

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `spearId` | `uint16` | 固定矛位索引，当前合法范围 `0..5` |
| `end_x` | `int16` | 动作结束世界系 `x` |
| `end_y` | `int16` | 动作结束世界系 `y` |
| `end_yaw` | `int16` | 动作结束世界系 `yaw` |
| `reserve0` | `uint16` | 预留，当前忽略 |
| `reserve1` | `uint16` | 预留，当前忽略 |

下位机会将 `spearId` 映射到固定位表 `Grip::Config::SpearGrab::TargetPoses[spearId]`，再执行与 `TakeSpear` 相同的动作流。

## 6. 固定矛位表

当前固件中共有 6 个固定矛位：

| `spearId` | `target_x` | `target_y` | `target_yaw` | 备注 |
| --- | --- | --- | --- | --- |
| `0` | `0.0` | `0.0` | `0.0` | 当前为占位值 |
| `1` | `0.0` | `0.0` | `0.0` | 当前为占位值 |
| `2` | `0.0` | `0.0` | `0.0` | 当前为占位值 |
| `3` | `0.0` | `0.0` | `0.0` | 当前为占位值 |
| `4` | `0.0` | `0.0` | `0.0` | 当前为占位值 |
| `5` | `0.0` | `0.0` | `0.0` | 当前为占位值 |

在填入真实标定位姿前，不建议上位机直接依赖 `TakeSpearById`。
