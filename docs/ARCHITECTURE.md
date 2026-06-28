# 项目架构文档

本文档描述 `26-v2-r2-ind-mcu-chassis` 的软件架构，从代码实际状态自动生成。

## 1. 系统分层

```
┌──────────────────────────────────────┐
│         上位机 (PC / NUC)              │
│   UART3(主链路) / UART1(辅链路)         │
└──────────────┬───────────────────────┘
               │ 串口协议 (AA BB + CRC16)
┌──────────────▼───────────────────────┐
│       UserCode/ (应用层)               │
│  ┌─────────────────────────────────┐ │
│  │ protocol/  协议解析与命令分发      │ │
│  │ chassis/   底盘/升降/定位/控制     │ │
│  │ grip/      夹取机构与动作组        │ │
│  │ infrared/  红外接收               │ │
│  │ suction/   吸盘组件               │ │
│  │ diagnostics/ 动作诊断             │ │
│  │ sync/      时钟对齐               │ │
│  │ i2c.*      I2C总线管理            │ │
│  │ device.*   设备实例化             │ │
│  │ connection.* 连接状态管理          │ │
│  │ project_parts.hpp 编译期形态裁剪    │ │
│  │ app.cpp    启动与实时调度          │ │
│  │ arena.cpp  静态内存分配            │ │
│  └─────────────────────────────────┘ │
├──────────────────────────────────────┤
│       Modules/ (复用驱动库)             │
│  BasicComponents / ChassisController  │
│  MotorDrivers / Sensors               │
│  TrajectoryControl / VelocityProfile  │
├──────────────────────────────────────┤
│  Core/ + Drivers/ + Middlewares/      │
│  (STM32CubeMX 生成代码)                │
└──────────────────────────────────────┘
```

## 2. 启动流程

```
Device::init()          构建设备对象（电机/传感器/吸盘）
  │
Chassis::init()         创建 IndLiftMecanum4 运动对象
  │
Protocol::init()        创建 PCProtocol, 启动接收DMA
  │
Grip::init()            [条件] 创建 Grip 对象
  │
Infrared::init()        [条件] 初始化红外接收
  │
Action::Xxx::inst()     [条件] 预创建动作组单例
  │
Connection::init()      初始化连接位图与 watchdog
  │
AppI2C::start_bus2_manager()  启动 I2C2 总线管理任务
  │
创建 Guard 任务          安全监护（断连检测 + 急停）
  │
启动定时器中断            1kHz / 100Hz 周期回调
  │
Connection::waitAll()   等待请求的硬件全部在线
  │
Chassis::motion->enable() / startCalibration()
Grip::grip->startCalibration()  [条件]
  │
等待 motion / grip ready
  │
Chassis::initStandaloneLocCtrl()  [无上位机定位]
或等待 System::Init::postureReceived  [上位机定位]
  │
Chassis::enable()       使能底盘控制器
Grip::grip->enable()    [条件] 使能 grip
motion->liftAllTo(Normal) 抬升至正常高度
```

## 3. 实时调度

| 频率 | 触发源 | 操作 |
|------|--------|------|
| 1kHz_1 | TIM 回调 (前半周期) | `Chassis::update_1kHz()`, `Grip::update_500Hz_1/2()` (交替), DM motor pings |
| 1kHz_2 | TIM 回调 (后半周期, 偏移半周期) | `Device::update_1kHz()` (CAN发包), `Connection::updateTable()`, `Watchdog::EatAll()` |
| 100Hz | TIM 回调 | `Chassis::update_100Hz()`, `Grip::update_100Hz()`, `Infrared::update_100Hz()` |
| 50Hz | FreeRTOS 任务 | `Protocol::ActionState::updateTable()` |
| 实时 | FreeRTOS 任务 | `PCCommandHandlerTask` - 命令消费与分发 |

## 4. 编译期形态裁剪

`UserCode/project_parts.hpp` 定义 12 个主开关，并派生系统能力常量：

```
PROJECT_PART_ENABLE_WHEEL_CHASSIS          → EnableChassisMotion
PROJECT_PART_ENABLE_LIFT                  → EnableChassisLocalization
PROJECT_PART_ENABLE_GRIP                  → EnableJustEncoderLocalization
PROJECT_PART_ENABLE_GRIP_SUCTION          → EnableEkfLocalization
PROJECT_PART_ENABLE_GRIP_SUCTION_PRESSURE_SENSOR
PROJECT_PART_ENABLE_GYRO                  → EnableUpperHostProtocol
PROJECT_PART_ENABLE_PC_LOCALIZATION       → EnableStepAction
PROJECT_PART_ENABLE_PC_CONTROL            → EnableStepWorkflow
PROJECT_PART_ENABLE_UPPER_HOST_IDENTIFY_INIT
PROJECT_PART_ENABLE_CONNECTION_TABLE_I2C_TX → EnableSpearGrabAction
PROJECT_PART_ENABLE_INFRARED_RECEIVER     → EnableSpearGrabWorkflow
PROJECT_PART_ENABLE_ABDOMEN_SUCTION       → EnableKfsAction
```

## 5. 核心模块

### 5.1 底盘模块 (`UserCode/chassis/`)

```
Chassis::motion   → IndLiftMecanum4  (统一的轮组+升降运动对象)
                       ├── 4× MotorVelController (轮电机)
                       └── 2× LiftSide (前后双侧升降)
                              └── 各含 2× MotorVelController + HomingMotorTrajectory<2>

Chassis::loc      → JustEncoder | LocEKF<256>  (定位)
Chassis::ctrl     → Master  (底盘控制器)
Chassis::offline_trajectory → OfflineTrajectoryFollower  (离线轨迹)
```

**定位模式选择：**
- 无陀螺仪 → `JustEncoder`
- 有陀螺仪 + 无上位机定位 → `LocEKF`
- 有陀螺仪 + 有上位机定位 → 延迟初始化 `LocEKF`，等待首个 `LidarPosture`

### 5.2 Grip 模块 (`UserCode/grip/`)

```
Grip::grip  → Grip 对象
                ├── arm: MotorVelController + MotorTrajectory<1> (DM4310, velocity模式)
                ├── turn: MotorVelController + HomingMotorTrajectory<1> (DM2325, MIT torque模式)
                └── claw: GPIO (RELAY3)

Grip::Action::SpearGrab  → 取矛头工作流 (10状态状态机, chassis+lift+grip 组合动作)
Grip::Action::KfsStore    → 卷轴暂存/释放工作流 (7状态状态机, grip+suction)
```

### 5.3 协议模块 (`UserCode/protocol/`)

```
PCProtocol          → UART 帧同步 (AA BB 帧头, CRC16 Modbus, DMA发送)
PCCommandHandler    → 命令消费与分发 (RingBuffer + RTOS任务)
PCFeedback          → 反馈帧构造与周期发送
ActionState         → 16位动作状态打包 (50Hz低优先级任务刷新)
```

**帧格式：**
- 控制帧: `AA BB | cmd(1B) | data(12B) | tx_timestamp(4B) | CRC16(2B)` = 21字节
- 反馈帧: `AA BB | timestamp(4B) | x(2B) | y(2B) | yaw(2B) | frontHeight(2B) | rearHeight(2B) | action_state(2B) | connection_state(2B) | CRC16(2B)` = 22字节

### 5.4 红外模块 (`UserCode/infrared/`)

- USART6 DMA 单字节循环接收
- 支持 `0xA0..0xA3` 四字节协议
- 需连续 ≥3 个相同合法字节才切换状态
- 稳定 `0xA0→0xA1` 时触发 `Grip::openClaw()` + 延时回收

### 5.5 吸盘模块 (`UserCode/suction/`)

`SuctionCup` - 可复用吸盘组件：
- 气泵 GPIO 控制 (Grip: RELAY2, Abdomen: RELAY0)
- 可选 I2C 气压计 (XGZP6847D, addr=0x6D, I2C2)
- Schmitt 触发器判定 (on: -5000Pa, off: -2000Pa)

## 6. 连接状态

16位连接位图 `Connection::table`，由 1kHz_2 回调刷新：

| Bit | 设备 | 总线/ID |
|-----|------|---------|
| 0-3 | wheel[0-3] | CAN1 id1=1,2 / CAN2 id1=3,4 |
| 4-7 | lift[0-3] | CAN1 id1=3,4 / CAN2 id1=1,2 |
| 8 | grip_arm | CAN2 DM id0=0x09 |
| 9 | grip_turn | CAN2 DM id0=0x0A |
| 10 | gyro_yaw | UART2 HWT101CT |
| 11 | GripSuctionPressure | I2C2 (条件) |
| 12 | AbdomenSuctionPressure | 预留 |
| 14 | upper_host_localization | 定位流 watchdog (200 tick超时) |
| 15 | upper_host | UART3 串口链路 |

## 7. 动作状态表

16位打包 `ActionState::table`，50Hz 刷新：

| 位段 | 名称 | 编码 |
|------|------|------|
| 0-1 | StepStatus | Idle(0)/Done(1)/Running(2)/WaitingTake(3) |
| 2-3 | ChassisMode | Stop(0)/Velocity(1)/Position(2)/Slave(3) |
| 4 | ChassisCurveFinished | 轨迹完成标志 |
| 5-6 | LiftStatus | Calibrating(0)/Running(1)/Ready(2)/NotEnabled(3) |
| 7-9 | GripStatus | Calibrating(0)/TakingSpear(1)/KfsStore(2)/KfsRelease(3)/Idle(4)/Done(5) |
| 10 | GripSuctionHasObject | 吸盘物体检测 |
| 11-12 | InfraredReceiverState | A0(0)/A1(1)/A2(2)/A3(3) |
| 13-15 | Reserved | 预留 |

## 8. 硬件映射

| 硬件 | 连接 | 协议 |
|------|------|------|
| 陀螺仪 HWT101CT | UART2 | 串口 |
| 上位机(主) | UART3 | 230400bps, AA BB 帧协议 |
| 辅控制器 | UART1 | AA BB 帧协议 |
| 红外接收 | USART6 | 57600bps, DMA 单字节循环 |
| 轮电机 x4 | CAN1(前), CAN2(后) | DJI |
| 升降电机 x4 | CAN1(前), CAN2(后) | DJI |
| Grip Arm | CAN2 | DM4310 (id0=0x09, velocity) |
| Grip Turn | CAN2 | DM2325 (id0=0x0A, MIT torque) |
| Grip Suction | RELAY2 | GPIO |
| Abdomen Suction | RELAY0 | GPIO |
| Grip Claw | RELAY3 | GPIO |
| 气压计 | I2C2 | XGZP6847D (addr=0x6D) |
