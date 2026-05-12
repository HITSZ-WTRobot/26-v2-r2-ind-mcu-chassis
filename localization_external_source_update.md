# 外部定位源更新

## 概述
此更改增加了外部定位源在运行时的切换，并支持基于视觉的相对位姿输入。EKF 仍然只消费单一路外部观测流；活动来源可以是 Lidar、Vision（视觉）或 None（无）。视觉输入的语义为“机器人在目标系中的位姿”，通过在首次视觉帧建立目标系锚点，将其映射到 EKF 的世界坐标系。

## 修改的组件
- 协议命令集：新增视觉位姿和定位源切换命令。
- 底盘定位：增加源切换、目标系锚定以及统一的 EKF 更新路径。
- 启动（Init）门控：是否需要外部初始位姿现在由运行时活动源决定。
- 连接位（connection bit14）：反映当前活动的外部定位流。

## 核心逻辑
1. 外部源选择
   - 系统跟踪活动源：`None`、`Lidar` 或 `Vision`。
   - 切换源时会重置该源的锚点，并启动一个短时的速度保持窗口（在该窗口内临时将底盘速度设为零以避免控制冲击）。

2. 视觉锚定（目标系中的机器人位姿）
   - 在切换到视觉后收到的第一帧视觉数据用于在当前世界位姿下建立目标系锚点。此锚点用于把后续的视觉观测映射到世界坐标系。

3. 统一的外部观测路径
   - Lidar 和 Vision 的观测都会被映射为世界坐标系下的位姿，并经由同一路径送入 EKF 更新接口（保持现有 EKF 不变）。

4. 启动初始位姿门控
   - 仅当活动源为 Lidar 或 Vision 时，系统启动阶段才会等待首个外部位姿；当活动源为 None 时则不阻塞启动。

## 新增/更新命令
- 0x21 LidarPosture
  - payload: x, y, yaw, timestamp
- 0x22 VisionPosture
  - payload: x, y, yaw, timestamp
  - 语义：机器人在目标系中的位姿，坐标轴与世界系对齐。
- 0x23 SetLocalizationSource
  - payload: mode (uint16)：0=None，1=Lidar，2=Vision

## 使用指南
### 之前
- 仅接受 `LidarPosture` 作为外部观测输入。
- 外部定位隐式为 Lidar-only（当 PC 定位启用时）。
- `connection` 中的 bit14 仅代表 Lidar 定位流。
- 启动阶段始终等待首个 Lidar 位姿（当 PC 定位启用时）。

### 之后
- 需要通过 `SetLocalizationSource` 选择活动外部定位源。
- 可在运行时选择 Lidar、Vision 或 None。
- `connection` 的 bit14 将反映当前活动的外部流（Lidar 或 Vision）。
- 启动仅在活动源为 Lidar 或 Vision 时等待首帧外部位姿。

### 推荐顺序
1. Lidar 模式
   - 发送 `SetLocalizationSource(mode=1)`
   - 以低频发送 `LidarPosture` 帧

2. Vision 模式
   - 发送 `SetLocalizationSource(mode=2)`
   - 以低频发送 `VisionPosture` 帧
   - 切换后收到的第一帧 `VisionPosture` 将定义目标系锚点

3. None 模式（仅预测 + 陀螺仪）
   - 发送 `SetLocalizationSource(mode=0)`
   - 无需外部位姿帧

## 行为差异与注意事项
- 切换来源会触发一个短时的速度保持窗口以避免控制冲击；该处理不会调用 `Stop()`，且不依赖定位结果。
- 视觉目标变化不会被自动检测；若目标物发生更换，需要重新切换到视觉模式以重新锚定。
- 外部位姿帧仅在来自主协议且时间同步正常时被接受，行为与原先 Lidar 一致。

## 验证清单
- 确认 `connection` 的 bit14 随活动外部流切换。
- 验证在 Lidar 与 Vision 之间切换时底盘的平滑性（无突跳）。
- 确认 None 模式不会阻塞启动，也不会要求外部位姿帧。
