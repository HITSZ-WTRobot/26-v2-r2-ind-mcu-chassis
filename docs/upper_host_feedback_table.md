# 上位机反馈数据表

本文档面向上位机开发，描述当前串口上行反馈帧的结构、缩放规则以及 `ActionState` / `Connection::table` 的解码方法。

## 1. 通用帧格式

下位机当前使用固定长度反馈帧：

| 偏移 | 长度 | 类型 | 含义 |
| --- | --- | --- | --- |
| 0 | 1 | `uint8` | 帧头 `0xAA` |
| 1 | 1 | `uint8` | 帧头 `0xBB` |
| 2 | 4 | `uint32` | 下位机当前时间戳 `timestamp` |
| 6 | 2 | `int16` | `x * 2000` |
| 8 | 2 | `int16` | `y * 2000` |
| 10 | 2 | `int16` | `yaw * 100` |
| 12 | 2 | `int16` | `frontHeight * 2000` |
| 14 | 2 | `int16` | `rearHeight * 2000` |
| 16 | 2 | `uint16` | `ActionState::table` |
| 18 | 2 | `uint16` | `Connection::table` |
| 20 | 2 | `uint16` | `CRC16` |

- 总帧长固定为 `22` 字节。
- 所有多字节字段都按大端发送，即高字节在前。
- `CRC16` 使用 `CRC16-Modbus` 参数：
  `poly=0x8005, init=0xFFFF, refin=true, refout=true, xorout=0x0000`。
- `CRC16` 的计算范围是从 `timestamp` 开始，到 `connection_state` 结束，不包含帧头和 `CRC16` 自身。

## 2. 数值缩放规则

| 字段 | 解码方式 |
| --- | --- |
| `x / y` | `value = int16 / 2000.0`，单位 m |
| `yaw` | `value = int16 / 100.0`，单位 deg |
| `frontHeight / rearHeight` | `value = int16 / 2000.0`，单位 m |

补充：

- `frontHeight / rearHeight` 反馈的是底盘离地高度，不是 lift 内部原始行程。
- 当升降机构未启用或未就绪时，这两个高度会回退到接地默认高度。

## 3. `ActionState::table`

`ActionState::table` 是一个 16 位打包字段，当前布局如下：

| 位段 | 掩码 | 名称 | 说明 |
| --- | --- | --- | --- |
| `bit0..1` | `0x0003` | `StepStatus` | 台阶动作状态 |
| `bit2..3` | `0x000C` | `ChassisMode` | 底盘控制模式 |
| `bit4` | `0x0010` | `ChassisCurveFinished` | 底盘位置轨迹是否完成 |
| `bit5..6` | `0x0060` | `LiftStatus` | 升降机构状态 |
| `bit7..9` | `0x0380` | `GripStatus` | Grip / 动作组状态 |
| `bit10` | `0x0400` | `GripSuctionHasObject` | Grip 吸盘是否检测到物体 |
| `bit11..15` | `0xF800` | Reserved | 预留 |

建议上位机按下面方式解码：

```text
step_status              = (table >> 0) & 0x3
chassis_mode             = (table >> 2) & 0x3
chassis_curve_finished   = (table >> 4) & 0x1
lift_status              = (table >> 5) & 0x3
grip_status              = (table >> 7) & 0x7
grip_suction_has_object  = (table >> 10) & 0x1
```

补充：

- `ActionState::table` 由下位机一个独立的低优先级 `50 Hz` 任务刷新；串口反馈帧可能在相邻多个发送周期里重复同一份 `ActionState`。

### 3.1 `StepStatus`

| 值 | 枚举 | 含义 |
| --- | --- | --- |
| `0` | `Idle` | 无台阶动作 |
| `1` | `Done` | 台阶动作完成 |
| `2` | `Running` | 台阶动作执行中 |
| `3` | `WaitingTake` | 正在等待取卷轴后恢复 |

### 3.2 `ChassisMode`

| 值 | 枚举 | 含义 |
| --- | --- | --- |
| `0` | `Stop` | 静止 / 停止状态 |
| `1` | `Velocity` | Master 速度控制模式 |
| `2` | `Position` | Master 位置控制模式 |
| `3` | `Slave` | 预留给从机控制模式 |

### 3.3 `ChassisCurveFinished`

| 值 | 含义 |
| --- | --- |
| `0` | 当前底盘位置轨迹尚未完成 |
| `1` | 当前底盘位置轨迹已完成，或当前并不在位置轨迹模式 |

### 3.4 `LiftStatus`

| 值 | 枚举 | 含义 |
| --- | --- | --- |
| `0` | `Calibrating` | 升降仍在初始化 / 未 ready |
| `1` | `Running` | 正在执行升降动作 |
| `2` | `Ready` | 升降机构已就位 |
| `3` | `NotEnabled` | 当前工程未启用升降机构 |

### 3.5 `GripStatus`

| 值 | 枚举 | 含义 |
| --- | --- | --- |
| `0` | `Calibrating` | Grip 仍在初始化 / 未校准完成 |
| `1` | `TakingSpear` | 取矛头动作执行中 |
| `2` | `KfsStore` | 卷轴临时存放动作执行中 |
| `3` | `KfsRelease` | 卷轴释放动作执行中 |
| `4` | `Idle` | 当前无 Grip 动作 |
| `5` | `Done` | 最近一次 Grip 动作已完成 |

补充：

- `GripStatus::Done` 会在动作结束后保持，直到发起下一次 Grip 动作或回到其他状态。
- `KfsStore / KfsRelease` 的区分来自 KFS 动作内部记录的 `workflowPhase`。

### 3.6 `GripSuctionHasObject`

| 值 | 含义 |
| --- | --- |
| `0` | 当前未检测到物体，或当前工程未启用 Grip suction 气压计 |
| `1` | 当前 Grip suction 检测到物体 |

补充：

- 该位只在 `PROJECT_PART_ENABLE_GRIP_SUCTION_PRESSURE_SENSOR=1` 时有实际语义。
- 当前它复用下位机吸盘组件内部的气压施密特判定结果，因此会跟随当前新鲜压力样本变化。

## 4. `Connection::table`

`Connection::table` 是一个 16 位连接位图，`bit=1` 表示对应对象在线。

| Bit | 掩码 | 名称 | 说明 |
| --- | --- | --- | --- |
| `0` | `0x0001` | `wheel[0]` | 前轮驱动，CAN1 `id1=1` |
| `1` | `0x0002` | `wheel[1]` | 前轮驱动，CAN1 `id1=2` |
| `2` | `0x0004` | `wheel[2]` | 后轮驱动，CAN2 `id1=3` |
| `3` | `0x0008` | `wheel[3]` | 后轮驱动，CAN2 `id1=4` |
| `4` | `0x0010` | `lift[0]` | 前侧抬升 0 |
| `5` | `0x0020` | `lift[1]` | 前侧抬升 1 |
| `6` | `0x0040` | `lift[2]` | 后侧抬升 0 |
| `7` | `0x0080` | `lift[3]` | 后侧抬升 1 |
| `8` | `0x0100` | `grip_arm` | Grip 大臂电机 |
| `9` | `0x0200` | `grip_turn` | Grip 转向电机 |
| `10` | `0x0400` | `gyro_yaw` | 航向陀螺仪 |
| `11` | `0x0800` | Reserved | 预留 |
| `12` | `0x1000` | Reserved | 预留 |
| `13` | `0x2000` | Reserved | 预留 |
| `14` | `0x4000` | `upper_host_localization` | 上位机定位流在线 |
| `15` | `0x8000` | `upper_host` | 上位机串口链路在线 |

补充：

- `bit14` 表示“定位流在线”，不是单纯“UART 收到过数据”。
- 当前实现中，只有收到合法 `LidarPosture` 帧后才会喂定位流 watchdog；watchdog 超时后 `bit14` 会自动清零。
- 当前定位流 watchdog 超时时间为收到最后一帧合法 `LidarPosture` 后 `200` 个 watchdog tick。
- `bit15` 表示上位机串口链路本身在线。

## 5. 当前反馈字段与上位机建议

- 上位机通常应同时解析 `ActionState::table` 与 `Connection::table`，不要只依赖其中一项判断系统是否 ready。
- 若 `GripStatus=Done` 但相关连接位已掉线，应优先按故障处理，而不是按动作成功处理。
- 若使用上位机定位，应同时监视 `bit14` 与 `bit15`：
  `bit15=1` 但 `bit14=0` 表示串口链路还在，但定位流已经超时。
