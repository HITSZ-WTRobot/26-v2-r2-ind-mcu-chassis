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
| Grip 关节角 `arm_pos/turn_pos` | `int16 = value_deg * 100` |
| 线速度 `vx/vy` | `int16 = value_mps * 2000` |
| 角速度 `wz` | `int16 = value_degps * 100` |
| 底盘高度 | `int16 = chassis_height_m * 2000` |
| `xy_vmax / xy_amax` | `uint12`，缩放见各命令说明 |

## 3. 对时与生效时机

- 下位机收到命令帧后，会先用 `tx_timestamp` 做时钟对齐。
- 当前实现中，只要命令帧 `CRC16` 校验通过，就会立即进入命令分发逻辑，不再保留固定的“前 49 帧仅对时”预热窗口。
- 对于来自主上位机链路的帧，下位机会在命令分发前先更新全局上下位机时钟偏移。
- `LidarPosture` 额外要求该帧来自主上位机链路，且当前对时残差已经进入稳定阈值；在此之前该命令会被忽略。

## 4. 指令表

| Cmd | 名称 | 数据区格式 | 说明 |
| --- | --- | --- | --- |
| `0x01` | `Ping` | 无 | 当前仅参与对时，暂无 `Pong` 回复。 |
| `0x10` | `StopChassis` | 无 | 停止底盘控制。 |
| `0x11` | `SetChassisHeight` | `chassisHeight(int16), v_max(uint16), a_max(uint16), j_max(uint16), linkMode(uint16)` | 设置底盘离地高度。 |
| `0x12` | `SlavePushChassisTrajectory` | `x(int16), y(int16), yaw(int16), vx(int16), vy(int16), wz(int16)` | 当前固件已保留格式，但处理逻辑仍为空。 |
| `0x13` | `SetMasterChassisTargetCurrentState` | `x(int16), y(int16), yaw(int16), xy_vmax(uint12), xy_amax(uint12), yaw_vmax(uint12), yaw_amax(uint12)` | Master 模式目标位姿，轨迹从当前状态衔接。 |
| `0x14` | `SetMasterChassisTargetPreviousCurve` | 与 `0x13` 相同 | Master 模式目标位姿，轨迹沿上一条曲线衔接。 |
| `0x15` | `SetMasterChassisVelocity` | `vx(int16), vy(int16), wz(int16), reserve(uint16), reserve(uint16), reserve(uint16)` | Master 模式车体系速度指令。 |
| `0x16` | `SetGripPose` | `arm_pos(int16), turn_pos(int16), clawMode(uint16), reserve(uint16), reserve(uint16), reserve(uint16)` | 设置 Grip 双轴关节目标和可选夹爪状态。 |
| `0x17` | `SetGripPresetPose` | `presetId(uint16), reserve(uint16), reserve(uint16), reserve(uint16), reserve(uint16), reserve(uint16)` | 设置 Grip 到预设姿态。 |
| `0x21` | `LidarPosture` | `x(int16), y(int16), yaw(int16), lidarTimestamp(uint32)` | 上位机定位位姿输入。 |
| `0x30` | `StepUp200` | `startDistance(int16), endDistance(int16), direction(uint16), willTake(uint16)` | 上 200mm 台阶动作组。 |
| `0x31` | `StepUpResume` | 无 | 恢复此前因 `willTake=1` 暂停的上台阶流程，200mm / 400mm 共用。 |
| `0x32` | `StepDown200` | `startDistance(int16), endDistance(int16), direction(uint16), shouldReset(uint16)` | 下 200mm 台阶动作组。 |
| `0x33` | `StepUp400` | 与 `StepUp200` 相同 | 上 400mm 台阶动作组。 |
| `0x34` | `StepDown400` | 与 `StepDown200` 相同 | 下 400mm 台阶动作组。 |
| `0x50..0x5F` | `StepPose` | `stepTarget_x(int16), stepTarget_y(int16), stepTarget_yaw(int16), end_x(int16), end_y(int16), end_yaw(int16)` | 平面台阶动作组，cmd 低 4 位编码动作类型、方向、高度和参数。 |
| `0x40` | `TakeSpear` | `target_x(int16), target_y(int16), target_yaw(int16), end_x(int16), end_y(int16), end_yaw(int16)` | 直接指定待取矛头位姿和结束位姿。 |
| `0x41` | `TakeSpearById` | `spearId(uint16), end_x(int16), end_y(int16), end_yaw(int16), reserve(uint16), reserve(uint16)` | 通过固定矛位索引启动取矛头。 |
| `0x42` | `StoreKFS` | 无 | 卷轴临时存放动作组。 |
| `0x43` | `ReleaseKFS` | 无 | 卷轴释放动作组。 |
| `0x44` | `SetGripSuction` | `on(uint16), reserve(uint16) x5` | 控制 Grip 吸盘开关。`on=1` 启动气泵，`on=0` 关闭气泵。 |
| `0x45` | `SetAbdomenSuction` | `on(uint16), reserve(uint16) x5` | 控制腹部吸盘开关。`on=1` 启动气泵，`on=0` 关闭气泵。 |

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

### 5.3 `0x15 SetMasterChassisVelocity`

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `vx` | `int16` | 车体系前向速度，编码为 `m/s * 2000` |
| `vy` | `int16` | 车体系左向速度，编码为 `m/s * 2000` |
| `wz` | `int16` | 绕 z 轴角速度，编码为 `deg/s * 100` |
| `reserve0` | `uint16` | 预留，当前忽略 |
| `reserve1` | `uint16` | 预留，当前忽略 |
| `reserve2` | `uint16` | 预留，当前忽略 |

补充：

- 该命令按车体系解释，等价于下位机内部调用 `setVelocityInBody(body_velocity, false)`。
- 后 3 个 `reserve` 字段当前保留并忽略。

### 5.4 `0x16 SetGripPose`

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `arm_pos` | `int16` | Grip 大臂目标角，单位 deg，编码为 `deg * 100` |
| `turn_pos` | `int16` | Grip 转向目标角，单位 deg，编码为 `deg * 100` |
| `clawMode` | `uint16` | `0=保持当前夹爪状态`, `1=张开夹爪`, `2=闭合夹爪` |
| `reserve0` | `uint16` | 预留，当前忽略 |
| `reserve1` | `uint16` | 预留，当前忽略 |
| `reserve2` | `uint16` | 预留，当前忽略 |

补充：

- 该命令仅在 `PROJECT_PART_ENABLE_PC_CONTROL=1` 且 `PROJECT_PART_ENABLE_GRIP=1` 时生效。
- 下位机要求 `Grip::grip` 已创建且已经 `enable()`；未完成回零校准或未使能时会忽略该命令。
- 内部等价于调用 `Grip::grip->toJointPose({arm_pos, turn_pos})`，使用 grip 本地轨迹限幅参数。
- `clawMode` 为 `0` 或非 `1/2` 值时不会改变当前夹爪状态。
- 这是低层直接姿态命令；若高层 Grip 动作组仍在运行，后续动作阶段可能再次下发自己的姿态目标，上位机应避免混用。

### 5.5 `0x17 SetGripPresetPose`

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `presetId` | `uint16` | 预设姿态编号，见下表 |
| `reserve0` | `uint16` | 预留，当前忽略 |
| `reserve1` | `uint16` | 预留，当前忽略 |
| `reserve2` | `uint16` | 预留，当前忽略 |
| `reserve3` | `uint16` | 预留，当前忽略 |
| `reserve4` | `uint16` | 预留，当前忽略 |

预设姿态：

| `presetId` | 预设名 | 下位机调用 |
| --- | --- | --- |
| `0` | `Standby` | `Grip::grip->toStandbyPose()` |
| `1` | `PrepareGrab` | `Grip::grip->toPrepareGrabPose()` |
| `2` | `Grab` | `Grip::grip->toGrabPose()` |
| `3` | `Docking` | `Grip::grip->toDockingPose()` |
| `4` | `KfsPickup` | `Grip::grip->toKfsPickupPose()` |
| `5` | `KfsStore` | `Grip::grip->toKfsStorePose()` |
| `6` | `KfsRelease` | `Grip::grip->toKfsReleasePose()` |

补充：

- 该命令仅在 `PROJECT_PART_ENABLE_PC_CONTROL=1` 且 `PROJECT_PART_ENABLE_GRIP=1` 时生效。
- 下位机要求 `Grip::grip` 已创建且已经 `enable()`；未完成回零校准或未使能时会忽略该命令。
- 非法 `presetId` 会被忽略。
- 预设命令走现有语义姿态接口；例如 `Standby/PrepareGrab` 会张开夹爪，`Grab` 会闭合夹爪。
- 这是低层直接姿态命令；若高层 Grip 动作组仍在运行，后续动作阶段可能再次下发自己的姿态目标，上位机应避免混用。

### 5.6 `0x21 LidarPosture`

- `lidarTimestamp` 为上位机定位数据原始时间戳，单位 ms。
- 该命令仅在启用了上位机定位模式时参与处理。
- 当前实现只接受来自主上位机链路、且对时已稳定的 `LidarPosture`；未满足这两个条件时，该帧不会喂定位 watchdog，也不会进入定位更新。
- 当工程启用了上位机定位模式时，首个满足上述接入条件的 `LidarPosture` 会承担系统初始位姿的延迟初始化职责。

### 5.7 `0x30 StepUp200 / 0x33 StepUp400`

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `startDistance` | `int16` | 开始时车体中心距离台阶边缘的距离，编码为 `m * 2000` |
| `endDistance` | `int16` | 结束时车体中心距离台阶边缘的距离，编码为 `m * 2000` |
| `direction` | `uint16` | `0=Forward`, `1=Backward` |
| `willTake` | `uint16` | `0=连贯上台阶`, `1=中途停下等待取卷轴` |

补充：

- `0x30 StepUp200` 使用 200mm 台阶高度配置。
- `0x33 StepUp400` 使用 400mm 台阶高度配置。
- `0x31 StepUpResume` 对两种上台阶动作共用，只恢复当前正在等待取件的上台阶流程。

### 5.8 `0x32 StepDown200 / 0x34 StepDown400`

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `startDistance` | `int16` | 开始时车体中心距离台阶边缘的距离，编码为 `m * 2000` |
| `endDistance` | `int16` | 结束时车体中心距离台阶边缘的距离，编码为 `m * 2000` |
| `direction` | `uint16` | `0=Forward`, `1=Backward` |
| `shouldReset` | `uint16` | `1=下台阶后恢复正常高度`, `0=最后一步不回收底盘` |

补充：

- `0x32 StepDown200` 使用 200mm 台阶高度配置。
- `0x34 StepDown400` 使用 400mm 台阶高度配置。

### 5.9 `0x50..0x5F StepPose`

该命令组用于平面作业面的台阶动作。命令字按位编码：

```text
cmd = 0x50 | type<<3 | dir<<2 | height<<1 | param
```

| 字段 | 位 | 说明 |
| --- | --- | --- |
| `type` | bit3 | `0=up`, `1=down` |
| `dir` | bit2 | `0=Forward`, `1=Backward` |
| `height` | bit1 | `0=Step200`, `1=Step400` |
| `param` | bit0 | `type=up` 时为 `willTake`；`type=down` 时为 `shouldReset` |

数据区：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `stepTarget_x` | `int16` | 台阶边缘选定作业点世界系 `x`，编码为 `m * 2000` |
| `stepTarget_y` | `int16` | 台阶边缘选定作业点世界系 `y`，编码为 `m * 2000` |
| `stepTarget_yaw` | `int16` | 台阶作业方向世界系 `yaw`，编码为 `deg * 100` |
| `end_x` | `int16` | 动作最终结束世界系 `x`，编码为 `m * 2000` |
| `end_y` | `int16` | 动作最终结束世界系 `y`，编码为 `m * 2000` |
| `end_yaw` | `int16` | 动作最终结束世界系 `yaw`，编码为 `deg * 100` |

命令字展开：

| Cmd | 动作 | 方向 | 高度 | 参数 |
| --- | --- | --- | --- | --- |
| `0x50` | up | Forward | Step200 | `willTake=0` |
| `0x51` | up | Forward | Step200 | `willTake=1` |
| `0x52` | up | Forward | Step400 | `willTake=0` |
| `0x53` | up | Forward | Step400 | `willTake=1` |
| `0x54` | up | Backward | Step200 | `willTake=0` |
| `0x55` | up | Backward | Step200 | `willTake=1` |
| `0x56` | up | Backward | Step400 | `willTake=0` |
| `0x57` | up | Backward | Step400 | `willTake=1` |
| `0x58` | down | Forward | Step200 | `shouldReset=0` |
| `0x59` | down | Forward | Step200 | `shouldReset=1` |
| `0x5A` | down | Forward | Step400 | `shouldReset=0` |
| `0x5B` | down | Forward | Step400 | `shouldReset=1` |
| `0x5C` | down | Backward | Step200 | `shouldReset=0` |
| `0x5D` | down | Backward | Step200 | `shouldReset=1` |
| `0x5E` | down | Backward | Step400 | `shouldReset=0` |
| `0x5F` | down | Backward | Step400 | `shouldReset=1` |

补充：

- `StepTargetPos` 定义在世界系中，表示台阶边缘上的选定作业点；其 `yaw` 描述台阶作业方向直线。
- 下位机内部使用相对 `StepTargetPos` 的位姿 `R{x, y, yaw}` 规划预备点和跨台阶阶段。
- `Forward` 的相对 yaw 为 `0deg`，`Backward` 的相对 yaw 为 `180deg`，不做 yaw 归一化。
- `0x31 StepUpResume` 同样恢复该命令组中 `willTake=1` 暂停的上台阶流程。

### 5.10 `0x40 TakeSpear`

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

### 5.11 `0x41 TakeSpearById`

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

### 5.12 `0x44 SetGripSuction`

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `on` | `uint16` | `0=关闭气泵`, `1=启动气泵` |
| `reserve0` | `uint16` | 预留，当前忽略 |
| `reserve1` | `uint16` | 预留，当前忽略 |
| `reserve2` | `uint16` | 预留，当前忽略 |
| `reserve3` | `uint16` | 预留，当前忽略 |
| `reserve4` | `uint16` | 预留，当前忽略 |

补充：

- 该命令仅在 `PROJECT_PART_ENABLE_GRIP_SUCTION=1` 时生效。
- 控制 Grip 侧吸盘气泵（RELAY1），直接操作 GPIO，不经过 KFS 动作组。
- 与 `StoreKFS` / `ReleaseKFS` 独立：KFS 动作组内部也会控制该吸盘，上位机应避免与动作组同时操作同一吸盘。

### 5.13 `0x45 SetAbdomenSuction`

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `on` | `uint16` | `0=关闭气泵`, `1=启动气泵` |
| `reserve0` | `uint16` | 预留，当前忽略 |
| `reserve1` | `uint16` | 预留，当前忽略 |
| `reserve2` | `uint16` | 预留，当前忽略 |
| `reserve3` | `uint16` | 预留，当前忽略 |
| `reserve4` | `uint16` | 预留，当前忽略 |

补充：

- 该命令仅在 `PROJECT_PART_ENABLE_ABDOMEN_SUCTION=1` 时生效。
- 控制车体腹部吸盘气泵（RELAY2），独立于 Grip 吸盘。
- 当前不连接气压计，无吸附检测能力。
