# 静态分析与参数调整报告

## 0. 结论摘要

本次按“**test 全开**”重新分析并完成如下调整：

1. `pc-feedback` 已从原来的约 `1 kHz` 降到 **`200 Hz`**  
   位置：`UserCode/protocol/PCProtocol.cpp`，`osDelay(FeedbackPeriodMs)`，其中 `FeedbackPeriodMs = 5U`

2. Arena 已从 `72 KiB` 收敛到 **`33 KiB`**  
   依据：当前 full-feature + test 全开构建下，Arena 实际高水位仍为 **`30108 B`**，按 `+10%` 得 `33118.8 B`，取整到 `33 KiB = 33792 B`

3. FreeRTOS 各任务栈已按“**静态最坏峰值 + 约 20% 裕量**”重新收敛  
   说明：这里的“20% 裕量”按 `配置栈 ≈ 峰值 / 0.8` 的思路取整，尽量靠近但不过分抠到边界

4. 以 test 全开重新编译后的整体 RAM 占用为：
   - `RAM: 76864 B / 128 KB = 58.64%`
   - `FLASH: 102252 B / 512 KB = 19.50%`

本报告以下所有分析，均以 **2026-05-07** 当前工作区、当前代码、当前 `Debug` 构建产物、并且 **5 个测试线程全部启用** 为前提。

---

## 1. 分析范围与方法

### 1.1 本次构建方式

本次分析基于以下 test 全开构建：

```sh
cmake --preset Debug \
  -DCMAKE_C_FLAGS='-DTEST_ENABLE_CHASSIS_LIFT=1 -DTEST_ENABLE_GRIP_STANDALONE=1 -DTEST_ENABLE_STEP_MANUAL=1 -DTEST_ENABLE_SPEAR_GRAB_MANUAL=1 -DTEST_ENABLE_AUTO_MAPPING=1' \
  -DCMAKE_CXX_FLAGS='-DTEST_ENABLE_CHASSIS_LIFT=1 -DTEST_ENABLE_GRIP_STANDALONE=1 -DTEST_ENABLE_STEP_MANUAL=1 -DTEST_ENABLE_SPEAR_GRAB_MANUAL=1 -DTEST_ENABLE_AUTO_MAPPING=1'

cmake --build --preset Debug
```

### 1.2 使用依据

- 源码控制流：`UserCode/`、`Core/Src/freertos.c`、`Core/Inc/FreeRTOSConfig.h`
- `-fstack-usage` 生成的 `.su`
- `-fcallgraph-info=su` 生成的 `.ci`
- ELF / map / nm / size 结果

### 1.3 口径说明

1. 任务栈深是**静态最坏情况估算**，不是板上高水位实测。
2. `.su` 只给出函数自身静态栈帧；报告中的峰值是结合 `.ci` 调用链逐层叠加。
3. 已额外计入 Cortex-M4F 上下文压栈：
   - 整数任务：`+68 B`
   - 可能触发 FPU 上下文的任务：`+204 B`
4. CPU 占用是**静态稳态估算**，不是 DWT / trace 实测。
5. `build/Debug/.../UserCode/auto_mapping_app.cpp.*`、`rtos_stack_monitor.cpp.*` 这类旧残留构建产物不在当前 `build.ninja / compile_commands.json` 中，**已忽略**。

---

## 2. 当前运行时任务集合

在本次 test 全开构建下，实际存在的任务如下。

### 2.1 应用任务

| 任务名 | 来源 |
| --- | --- |
| `defaultTask` | `Core/Src/freertos.c` |
| `init` | `UserCode/app.cpp` |
| `pc-cmd-processor` | `UserCode/protocol/PCProtocol.cpp` |
| `pc-feedback` | `UserCode/protocol/PCProtocol.cpp` |
| `action-state` | `UserCode/protocol/ActionState.cpp` |
| `Step` | `UserCode/chassis/actions/Step.cpp` |
| `SpearGrab` | `UserCode/grip/actions/spear_grab.cpp` |
| `KfsStore` | `UserCode/grip/actions/roller_store.cpp` |
| `i2c2-mgr` | `UserCode/i2c.cpp` / `I2CUpdateManager` |
| `test-cl` | `UserCode/tests/chassis_lift.cpp` |
| `test-grip` | `UserCode/tests/grip_standalone.cpp` |
| `test-step` | `UserCode/tests/step_manual.cpp` |
| `test-spear` | `UserCode/tests/spear_grab_manual.cpp` |
| `test-map` | `UserCode/tests/auto_mapping.cpp` |

### 2.2 内核任务

| 任务名 | 来源 |
| --- | --- |
| `Idle` | FreeRTOS |
| `Timer service` | FreeRTOS software timer task |

### 2.3 两个容易混淆的点

1. `Step::up/down()`、`SpearGrab::grab()`、`KfsStore::store()/release()` 是**命令入口**，运行在触发它们的线程栈上，不在各自后台状态机线程栈上。
2. test 线程不仅占自己的 `TestTask()` 栈帧，还要计入它们间接触发的控制器 / 动作函数链。

---

## 3. Task 栈深分析

## 3.1 当前配置值

### 3.1.1 应用任务当前配置

| 任务 | 当前配置栈 |
| --- | ---: |
| `defaultTask` | `32 * 4 = 128 B` |
| `init` | `232 * 4 = 928 B` |
| `pc-cmd-processor` | `544 * 4 = 2176 B` |
| `pc-feedback` | `104 * 4 = 416 B` |
| `action-state` | `88 * 4 = 352 B` |
| `Step` | `352 * 4 = 1408 B` |
| `SpearGrab` | `352 * 4 = 1408 B` |
| `KfsStore` | `152 * 4 = 608 B` |
| `i2c2-mgr` | `120 * 4 = 480 B` |
| `test-cl` | `352 * 4 = 1408 B` |
| `test-grip` | `168 * 4 = 672 B` |
| `test-step` | `320 * 4 = 1280 B` |
| `test-spear` | `320 * 4 = 1280 B` |
| `test-map` | `336 * 4 = 1344 B` |

### 3.1.2 内核任务当前配置

从当前 map 文件可直接读到：

- `Idle_Stack = 0x80 = 128 B`
- `Timer_Stack = 0xC0 = 192 B`

对应配置来源：

- `configMINIMAL_STACK_SIZE = 32 words`
- `configTIMER_TASK_STACK_DEPTH = 48 words`

---

## 3.2 静态最坏峰值估算

### 3.2.1 应用任务

| 任务 | 主要最深调用链 | 估算峰值 | 当前配置 | 余量 | 结论 |
| --- | --- | ---: | ---: | ---: | --- |
| `defaultTask` | `StartDefaultTask(8) + osDelay(8) + 整数上下文68` | `~84 B` | `128 B` | `~44 B` | 合格 |
| `init` | `Init(16) + Device::init(64)` / `Chassis::init(8)+IndLiftMecanum4(120)+LiftSide(376)` / `Protocol::init(16)+PCProtocol(88)` / `Grip::init(16)+Grip(312)`，取最深并计 FPU | `~724 B` | `928 B` | `~204 B` | 合格 |
| `pc-cmd-processor` | `TaskEntry(8)+receiveLoop(8)+cmdHandler(744)+max(Step::up/down≈824, SpearGrab::grab≈824, initLocCtrl.part.0≈360)` + FPU | `~1776 B` | `2176 B` | `~400 B` | 合格 |
| `pc-feedback` | `FeedbackTaskEntry(8)+feedbackLoop(8)+transmitFeedbackFrame(72)+姿态/高度读取` + FPU | `~324 B` | `416 B` | `~92 B` | 合格 |
| `action-state` | `loop(8)+updateTable(24)+hasDetectedObject()->SuctionCup::hasObject(40)` + FPU | `~276 B` | `352 B` | `~76 B` | 合格 |
| `Step` | `TaskEntry(8)+loop(8)+update(176)+max(Master::setTargetPostureInWorld 736, LiftSide::to 256)` + FPU | `~1132 B` | `1408 B` | `~276 B` | 合格 |
| `SpearGrab` | `TaskEntry(8)+loop(8)+update(136)+max(currentRelativeToTarget 88, postureRelativeToTargetInWorld 64, setTargetPostureInWorld 736, Grip::to*Pose≈248)` + FPU | `~1088 B` | `1408 B` | `~320 B` | 合格 |
| `KfsStore` | `TaskEntry(8)+loop(8)+update(8)+Grip::toKfs*/toStandby≈240` + FPU | `~460 B` | `608 B` | `~148 B` | 合格 |
| `i2c2-mgr` | `run(24)+serviceEntry(16)+XGZP6847D::onRead(32)+I2CBusDMA::memRead(40)+waitForTransfer(24)` + FPU | `~340 B` | `480 B` | `~140 B` | 合格 |
| `test-cl` | `TestTask(184)+max(setTargetPostureInWorld 736, LiftSide::to 256)` + FPU | `~1124 B` | `1408 B` | `~284 B` | 合格 |
| `test-grip` | `TestTask(80)+Grip::to*Pose(≤8)+planPose(16)+HomingMotorTrajectory::setTarget(224)` + FPU | `~528 B` | `672 B` | `~144 B` | 合格 |
| `test-step` | `TestTask(48)+Step::up/down(48)+prepare(40)+setTargetPostureInWorld 736` + FPU | `~1076 B` | `1280 B` | `~204 B` | 合格 |
| `test-spear` | `TestTask(72)+SpearGrab::grab(48)+Grip::toPrepareGrabPose≈248 / setTargetPostureInWorld 736` + FPU | `~1060 B` | `1280 B` | `~220 B` | 合格 |
| `test-map` | `TestTask(40)+runAutoMapping(48)+turnInBody(32)+BodyPosture2WorldPosture(184)+setTargetPostureInWorld(736)` + FPU | `~1244 B` | `1344 B` | `~100 B` | 合格，最紧 |

### 3.2.2 内核任务

| 任务 | 主要最深调用链 | 估算峰值 | 当前配置 | 余量 | 结论 |
| --- | --- | ---: | ---: | ---: | --- |
| `Idle` | `prvIdleTask(8) + 整数上下文68` | `~76 B` | `128 B` | `~52 B` | 合格 |
| `Timer service` | `prvTimerTask(72) + 整数上下文68` | `~140 B` | `192 B` | `~52 B` | 合格 |

---

## 3.3 栈调整前后对比

### 3.3.1 应用任务

| 任务 | 调整前 | 调整后 |
| --- | ---: | ---: |
| `defaultTask` | `512 B` | `128 B` |
| `init` | `4096 B` | `928 B` |
| `pc-cmd-processor` | `8192 B` | `2176 B` |
| `pc-feedback` | `2048 B` | `416 B` |
| `action-state` | `1024 B` | `352 B` |
| `Step` | `4096 B` | `1408 B` |
| `SpearGrab` | `1024 B` | `1408 B` |
| `KfsStore` | `1024 B` | `608 B` |
| `i2c2-mgr` | `1536 B` | `480 B` |
| `test-cl` | `2048 B` | `1408 B` |
| `test-grip` | `2048 B` | `672 B` |
| `test-step` | `4096 B` | `1280 B` |
| `test-spear` | `2048 B` | `1280 B` |
| `test-map` | `4096 B` | `1344 B` |

### 3.3.2 内核任务

| 任务 | 调整前 | 调整后 |
| --- | ---: | ---: |
| `Idle` | `512 B` | `128 B` |
| `Timer service` | `1024 B` | `192 B` |

### 3.3.3 总结

本轮调整的效果是：

- 原先唯一明确不足的 `SpearGrab` 线程已补足；
- 其余任务栈都从“过大保守值”收敛到“接近 20% 余量”的量级；
- `test-map` 是当前所有任务里**最紧**的一项，但仍保留了约 `100 B` 静态余量。

---

## 4. Arena 内存区域占用分析

## 4.1 先区分三类 RAM

本项目当前 RAM 里有三块概念上完全不同的区域：

1. **Arena**
   - 即 `UserCode/arena.cpp` 里的 `g_boot_arena`
   - 负责全局 `new`

2. **FreeRTOS heap**
   - 即 `ucHeap`
   - 负责任务栈、TCB、队列、互斥量等 RTOS 动态对象

3. **普通 `.bss/.data`**
   - 普通全局 / 静态变量
   - 包括 `Idle_Stack`、`Timer_Stack` 等静态内核栈

这三块不能混为一谈。  
**Arena 够不够**，看 `g_boot_arena`；**RTOS 任务栈够不够**，看 `ucHeap` 和各任务配置；**整体 RAM 够不够**，看链接后的总 RAM 使用。

---

## 4.2 Arena 当前容量

当前代码：

- `StaticArena<33 * 1024>`
- 即 **`33792 B`**

从 map 中可见：

- `.bss._ZL12g_boot_arena = 0x8408 = 33800 B`

这里比 `33792 B` 多出的 `8 B`，来自 `StaticArena` 内部的：

- `std::atomic<size_t> offset_`

因此：

- **Arena 可分配有效容量**：`33792 B`
- **Arena 对应静态 RAM 占位**：`33800 B`

---

## 4.3 Arena 实际分配对象

当前 full-feature + test 全开构建下，进入 Arena 的对象仍然是这批业务单例：

| 对象组 | 数量 | 单个大小 | 小计 |
| --- | ---: | ---: | ---: |
| `sensors::gyro::HWT101CT` | 1 | `76 B` | `76 B` |
| `XGZP6847DDevice` | 1 | `72 B` | `72 B` |
| `motors::DJIMotor` | 10 | `68 B` | `680 B` |
| `Chassis::IndLiftMecanum4` | 1 | `1296 B` | `1296 B` |
| `Protocol::PCProtocol` | 1 | `440 B` | `440 B` |
| `Grip::Grip` | 1 | `756 B` | `756 B` |
| `Chassis::ChassisLocEKF` | 1 | `25976 B` | `25976 B` |
| `Chassis::ChassisController` | 1 | `764 B` | `764 B` |
| 对齐填充 | - | - | `48 B` |
| **合计** | - | - | **`30108 B`** |

### 4.3.1 为什么 test 全开后 Arena 没继续涨

因为当前 5 个测试功能只会：

- 额外创建 FreeRTOS 线程
- 触发已有业务单例 / 控制器 / 动作对象

它们**不会新增新的 `new` 大对象**。  
所以 test 全开后，Arena 高水位仍然是业务单例那一套，**没有因为测试线程开启而增长**。

---

## 4.4 Arena 高水位与目标容量

### 4.4.1 当前高水位

- 实际高水位：`30108 B`

### 4.4.2 按 “+10%” 推导目标值

```text
30108 * 1.10 = 33118.8 B
```

因此：

- 理论最小目标值：`33119 B`
- 当前取整值：`33 KiB = 33792 B`

### 4.4.3 当前余量

```text
33792 - 30108 = 3684 B
```

占比：

```text
30108 / 33792 = 89.10%
```

即：

- **Arena 使用率约 89.1%**
- **留有约 10.9% 余量**

这正好符合“实际最大占用 + 10%”的目标。

---

## 4.5 其他 RAM 关键块

从当前 map 可见：

| 区域 | 大小 |
| --- | ---: |
| `g_boot_arena` | `0x8408 = 33800 B` |
| `ucHeap` | `0x8000 = 32768 B` |
| `Idle_Stack` | `0x80 = 128 B` |
| `Timer_Stack` | `0xC0 = 192 B` |

因此当前静态 RAM 大头实际上是：

1. `g_boot_arena`
2. `ucHeap`
3. 各类全局对象 / 表 / BSS

---

## 5. CPU 占用流程分析

## 5.1 分析前提

按用户要求，本节稳态前提固定为：

- 上位机雷达 `LidarPosture = 10 Hz`
- 其他上位机命令稀疏，`< 5 Hz`
- `pc-feedback = 200 Hz`
- 其余控制 / 中断频率保持当前工程默认：
  - `TIM_Callback_1kHz_1`: `1 kHz`
  - `TIM_Callback_1kHz_2`: `1 kHz`
  - `TIM_Callback_100Hz`: `100 Hz`
  - `ActionState`: `50 Hz`
  - 吸盘压力采样：`20 ms` 周期，即 `50 Hz`
  - 连接表 I2C 发送：`200 ms` 周期，即 `5 Hz`

说明：

- `Step` / `SpearGrab` / `KfsStore` / 各 test 线程虽然内部是 `osDelay(1)` 推进，但**空闲时基本都在等待事件或只是轮询触发变量**，不构成主要 CPU 大头
- 首帧 `LidarPosture` 触发 `LocEKF + Controller` 延迟构造是**一次性尖峰**，不计入稳态平均

---

## 5.2 系统流程拆解

### 5.2.1 高频主路径

1. `TIM_Callback_1kHz_1`
   - `Chassis::update_1kHz()`
   - `LocEKF / JustEncoder`
   - `Master::controllerUpdate()`
   - `Master::errorUpdate()` 每 500 Hz
   - 4 轮速度环
   - 2 侧 lift 轨迹 / 速度环
   - grip `update_1kHz()` 和 `update_500Hz()`

2. `TIM_Callback_1kHz_2`
   - `Device::update_1kHz()`
   - 3 组 DJI CAN 电流帧发送
   - `Connection::updateTable()`
   - watchdog 喂狗
   - 上位机断连保护

3. CAN 中断
   - 电机反馈接收
   - CAN 发送完成

### 5.2.2 中低频路径

1. `TIM_Callback_100Hz`
   - `Chassis::update_100Hz()`
   - `Master::profileUpdate(0.01f)`
   - lift `update_100Hz()`
   - grip `update_100Hz()`

2. 协议路径
   - `pc-cmd-processor`：命令解析、动作启动、10 Hz 雷达位姿更新
   - `pc-feedback`：200 Hz 反馈或识别阶段 `0xAA`
   - `action-state`：50 Hz 打包状态字

3. I2C 路径
   - 压力传感器 50 Hz 采样
   - 连接表 5 Hz 发送

---

## 5.3 稳态 CPU 占用估算

以下仍然是静态估算，不是板上实测。

| 来源 | 触发频率 | 中心估计 | 估计区间 | 说明 |
| --- | ---: | ---: | ---: | --- |
| `TIM_Callback_1kHz_1` 控制半拍 | `1000 Hz` | `4.8%` | `4.0% ~ 6.0%` | 底盘 / lift / grip 高频控制主路径 |
| `TIM_Callback_1kHz_2` 总线与管理半拍 | `1000 Hz` | `1.9%` | `1.5% ~ 2.5%` | CAN 集中发送、连接表刷新、watchdog |
| CAN RX 中断 | 约 `10000 frame/s` | `3.8%` | `3.0% ~ 5.0%` | DJI 电机反馈仍是主要中断负载 |
| CAN TX 完成中断 | 约 `3000 frame/s` | `0.7%` | `0.5% ~ 1.2%` | 3 组周期电流帧 |
| `pc-feedback` 发送链 | `200 Hz` | `0.2%` | `0.15% ~ 0.35%` | 由原先约 `1.0%` 明显降下 |
| `pc-cmd-processor` + 10 Hz 雷达融合 | `10 Hz + 稀疏命令` | `0.2%` | `0.1% ~ 0.4%` | `cmdHandler` 很深，但频率很低 |
| I2C2 manager + I2C DMA | `50 Hz + 5 Hz` | `0.1%` | `0.05% ~ 0.2%` | 压力采样 + connection table |
| `action-state` | `50 Hz` | `0.03%` | `<0.05%` | 仅状态打包 |
| RTOS / SysTick / 调度 | `1 kHz` 基础调度 | `1.3%` | `0.9% ~ 1.8%` | 比旧版略低，因 `pc-feedback` 唤醒频率下降 |
| **总忙碌占用** | - | **`13.0%`** | **`10.2% ~ 16.0%`** | 稳态估算 |
| **Idle** | - | **`87.0%`** | **`84.0% ~ 89.8%`** | 稳态估算 |

### 5.3.1 与旧版 `pc-feedback ≈ 1 kHz` 相比的变化

旧口径里，`pc-feedback` 约占：

- `~1.0%`

本次降到 `200 Hz` 后，变成约：

- `~0.2%`

因此可认为：

- **直接回收约 `0.8%` 左右 CPU 忙碌占用**
- 同时显著降低 UART TX DMA 事务密度和调度噪声

---

## 5.4 占用排序

稳态平均占用从大到小，大致是：

1. `TIM_Callback_1kHz_1`
2. CAN RX 中断
3. `TIM_Callback_1kHz_2`
4. RTOS / SysTick / 调度
5. CAN TX 完成中断
6. `pc-feedback`
7. 低频协议 / I2C / ActionState

也就是说，当前 CPU 的主要消耗仍然来自：

- **1 kHz 控制链**
- **高频 CAN 中断**

而不是测试线程，也不是 10 Hz 雷达消息本身。

---

## 6. 编译结果与 RAM 复核

## 6.1 当前构建结果

重新编译通过，结果如下：

```text
RAM:   76864 B / 128 KB = 58.64%
FLASH: 102252 B / 512 KB = 19.50%
```

`arm-none-eabi-size`：

```text
text = 102056
data = 188
bss  = 76672
```

### 6.1.1 与上轮 test 全开旧结果相比

旧结果：

- `RAM: 116800 B / 128 KB = 89.11%`

本次结果：

- `RAM: 76864 B / 128 KB = 58.64%`

差值：

```text
116800 - 76864 = 39936 B
```

这部分下降主要来自：

1. 应用线程栈大幅收敛
2. 内核 `Idle / Timer service` 栈收敛
3. Arena 从 `72 KiB` 收敛到 `33 KiB`

---

## 7. 最终结论

### 7.1 关于任务栈

- 当前所有 FreeRTOS 任务在“test 全开”下都已经重新按静态最坏峰值收敛
- 原先明显不足的 `SpearGrab` 已经补足
- 当前最紧的是 `test-map`，但仍有约 `100 B` 静态余量
- 整体已经达到“**大致 20% 左右余量**”的目标

### 7.2 关于 Arena

- Arena 当前有效容量：`33792 B`
- 当前高水位：`30108 B`
- 当前余量：`3684 B`
- 相当于 **高水位 + 10.9%**

因此：

- **Arena 已达到“实际最大占用 + 10%”目标**

### 7.3 关于 CPU

- `pc-feedback` 改为 `200 Hz` 后，稳态 CPU 忙碌占用约从 `14%` 下降到 `13%`
- 主要 CPU 大头仍然是 `1 kHz` 控制链与 CAN 中断
- 在“雷达 10 Hz、其他命令稀疏”的前提下，系统 CPU 仍有较大空闲

### 7.4 当前建议

1. 当前参数可以作为新的默认值继续使用。
2. 如果后续还要继续压栈，优先观察 `test-map`、`pc-cmd-processor`、`Step` 三项，不建议先动更紧的那些。
3. 若需要把静态估算进一步坐实，下一步应在板上补：
   - `uxTaskGetStackHighWaterMark()`
   - DWT `CYCCNT`
   - 关键 IRQ / callback 周期统计
