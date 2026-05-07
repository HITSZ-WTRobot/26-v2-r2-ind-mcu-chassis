/**
 * @file    PCProtocol.cpp
 * @author  syhanjin
 * @date    2026-04-10
 */
#include "PCProtocol.hpp"

#include "ActionState.hpp"
#include "connection.hpp"
#include "chassis/Config.hpp"
#include "chassis/actions/Step.hpp"
#include "chassis/chassis.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "grip/Config.hpp"
#include "grip/actions/roller_store.hpp"
#include "grip/actions/spear_grab.hpp"
#include "project_parts.hpp"
#include "system.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>

namespace Protocol
{
namespace
{
uint32_t read_u32(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) << 24 | static_cast<uint32_t>(data[1]) << 16 |
           static_cast<uint32_t>(data[2]) << 8 | static_cast<uint32_t>(data[3]);
}

uint16_t read_u16(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) << 8 | static_cast<uint16_t>(data[1]);
}

int16_t read_i16(const uint8_t* data)
{
    return static_cast<int16_t>(static_cast<uint16_t>(data[0]) << 8 |
                                static_cast<uint16_t>(data[1]));
}

float read_positive_or_default(const uint8_t* data, const float scale, const float default_value)
{
    const int16_t raw = read_i16(data);
    if (raw <= 0)
        return default_value;
    return static_cast<float>(raw) / scale;
}

trajectory::LinkMode read_lift_link_mode(const uint8_t* data)
{
    switch (read_u16(data))
    {
    case 1:
        return trajectory::LinkMode::CurrentState;
    case 2:
    case 0:
    default:
        return trajectory::LinkMode::PreviousCurve;
    }
}

void write_u32(uint8_t* data, const uint32_t value)
{
    data[0] = static_cast<uint8_t>(value >> 24);
    data[1] = static_cast<uint8_t>(value >> 16);
    data[2] = static_cast<uint8_t>(value >> 8);
    data[3] = static_cast<uint8_t>(value);
}

void write_u16(uint8_t* data, const uint16_t value)
{
    data[0] = static_cast<uint8_t>(value >> 8);
    data[1] = static_cast<uint8_t>(value);
}

int16_t to_scaled_i16(const float value, const float scale)
{
    const long scaled = lroundf(value * scale);
    return static_cast<int16_t>(std::clamp(scaled,
                                           static_cast<long>(std::numeric_limits<int16_t>::min()),
                                           static_cast<long>(std::numeric_limits<int16_t>::max())));
}

constexpr uint32_t MsgReceived = 1 << 0;

constexpr uint32_t FeedbackStart = 1 << 1;

constexpr uint32_t FeedbackPeriodMs = 5U;

constexpr uint32_t LidarPostureTimeoutTicks = 200U;

constexpr std::array<uint8_t, HeaderLen> FeedbackHeader = { 0xAA, 0xBB };

libs::RingBuffer<Frame, 10> command_buffer_{};

std::array<PCProtocol*, MaxPCProtocolCount> protocols_{};
std::array<uint8_t, FeedbackFrameLen>       feedback_frame_buffer_{};

uint32_t protocol_count_{ 0 };

osThreadId_t command_handler_task_{};
osThreadId_t transmit_task_{};

Sync::Clock global_clock_{};

service::Watchdog lidar_posture_watchdog_{};

struct
{
    struct
    {
        chassis::Posture last_received_posture{};
        uint32_t         last_received_posture_timestamp{};
        uint32_t         last_received_timestamp{};
        int32_t          last_received_delay{};
    } lidar;
} debug_{};

void register_protocol(PCProtocol* protocol)
{
    assert(protocol_count_ < protocols_.size());
    protocols_[protocol_count_++] = protocol;
}

void buildFeedbackFrame(std::array<uint8_t, FeedbackFrameLen>& frame)
{
    chassis::Posture posture{ .x = 0.0f, .y = 0.0f, .yaw = 0.0f };
    if (Chassis::loc != nullptr)
        posture = Chassis::loc->postureInWorld();

    float front_height = Chassis::Config::Lift::GroundingChassisHeight;
    float rear_height  = Chassis::Config::Lift::GroundingChassisHeight;
    if constexpr (ProjectParts::EnableLift)
    {
        if (Chassis::motion != nullptr && Chassis::motion->isReady())
        {
            front_height +=
                    Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Front).getPosition();
            rear_height +=
                    Chassis::motion->lift(Chassis::IndLiftMecanum4::LiftType::Rear).getPosition();
        }
    }

    auto* payload = frame.data() + HeaderLen;

    frame[0] = FeedbackHeader[0];
    frame[1] = FeedbackHeader[1];

    write_u32(&payload[0], HAL_GetTick());
    write_u16(&payload[4], static_cast<uint16_t>(to_scaled_i16(posture.x, 2000.0f)));
    write_u16(&payload[6], static_cast<uint16_t>(to_scaled_i16(posture.y, 2000.0f)));
    write_u16(&payload[8], static_cast<uint16_t>(to_scaled_i16(posture.yaw, 100.0f)));
    write_u16(&payload[10], static_cast<uint16_t>(to_scaled_i16(front_height, 2000.0f)));
    write_u16(&payload[12], static_cast<uint16_t>(to_scaled_i16(rear_height, 2000.0f)));
    write_u16(&payload[14], ActionState::table);
    write_u16(&payload[16], Connection::table);

    const uint16_t crc = CRC16Modbus::calc(payload, FeedbackPayloadLen - 2);
    write_u16(&payload[18], crc);
}

[[noreturn]] void PCCommandHandlerTask(void* argument)
{
    (void)argument;

    for (;;)
    {
        Frame frame{};
        while (command_buffer_.pop(frame))
        {
            handleCommand(frame);
        }
        osThreadFlagsWait(MsgReceived, osFlagsWaitAny, osWaitForever);
    }
}

[[noreturn]] void TransmitTask(void* argument)
{
    (void)argument;

    osThreadFlagsWait(FeedbackStart, osFlagsWaitAny, osWaitForever);
    for (;;)
    {
        buildFeedbackFrame(feedback_frame_buffer_);

        for (uint32_t i = 0; i < protocol_count_; ++i)
        {
            auto* protocol = protocols_[i];
            if (protocol != nullptr)
                protocol->transmitTaskStep(feedback_frame_buffer_);
        }

        osDelay(FeedbackPeriodMs);
    }
}
} // namespace

bool PCProtocol::decode(const uint8_t data[PayloadLen])
{
    const uint16_t crc_in_data = read_u16(&data[PayloadLen - 2]);
    const uint16_t crc         = CRC16Modbus::calc(data, PayloadLen - 2);

    if (crc != crc_in_data)
        return false;

    const bool pushed = command_buffer_.push(
            [&](Frame& frame)
            {
                frame.protocol           = this;
                frame.from_main_protocol = isMainProtocol();
                frame.rx_timestamp       = HAL_GetTick();
                frame.cmd                = static_cast<PCCommand>(data[0]);
                frame.tx_timestamp       = read_u32(&data[13]);
                frame.crc16              = crc_in_data;
                std::memcpy(frame.data.data(), data + 1, frame.data.size());
            });

    if (pushed && command_handler_task_ != nullptr)
        osThreadFlagsSet(command_handler_task_, MsgReceived);

    return pushed;
}

PCProtocol::PCProtocol(UART_HandleTypeDef* huart, const bool is_main_protocol) :
    UartRxSync(huart), is_main_protocol_(is_main_protocol)
{
    register_protocol(this);
}

const Sync::Clock& clock()
{
    return global_clock_;
}

void handleCommand(const Frame& frame)
{
    if (frame.protocol == nullptr)
        return;

    if (frame.from_main_protocol)
    {
        const auto  self_time      = static_cast<float>(frame.rx_timestamp);
        const float target_pc_time = static_cast<float>(frame.tx_timestamp) +
                                     frame.protocol->transitionDelayMS();

        global_clock_.align(self_time, target_pc_time);
    }

    if constexpr (ProjectParts::NeedUpperHostIdentifyInit && ProjectParts::EnableUpperHostProtocol)
    {
        if (frame.from_main_protocol)
            System::Init::upperHostIdentified = true;
    }

    constexpr auto to_pos = [](const int16_t value) { return static_cast<float>(value) / 2000.0f; };
    constexpr auto to_angle = [](const int16_t v) { return static_cast<float>(v) / 100.0f; };

    const auto& data = frame.data;

    switch (frame.cmd)
    {
    case PCCommand::Ping:
        break;
    case PCCommand::StopChassis:
        // StopChassis 属于“上位机控制命令”范畴；
        // 当该能力关闭时，协议层收到该帧也会直接忽略。
        if constexpr (ProjectParts::EnablePcControl)
        {
            if (Chassis::ctrl != nullptr)
                Chassis::ctrl->stop();
        }
        break;
    case PCCommand::SetChassisHeight:
        if constexpr (ProjectParts::EnablePcControl && ProjectParts::EnableLift)
        {
            if (Chassis::motion == nullptr || !Chassis::motion->isReady())
                break;

            const float                  chassis_height = to_pos(read_i16(&data[0]));
            const Chassis::Config::Limit limit{
                .max_spd  = read_positive_or_default(&data[2],
                                                    1000.0f,
                                                    Chassis::Config::Lift::OnloadLimit.max_spd),
                .max_acc  = read_positive_or_default(&data[4],
                                                    100.0f,
                                                    Chassis::Config::Lift::OnloadLimit.max_acc),
                .max_jerk = read_positive_or_default(&data[6],
                                                     1.0f,
                                                     Chassis::Config::Lift::OnloadLimit.max_jerk),
            };
            const trajectory::LinkMode link_mode = read_lift_link_mode(&data[8]);

            Chassis::motion->liftAllTo(Chassis::Config::Lift::chassisHeightToLiftPosition(
                                               chassis_height),
                                       limit,
                                       link_mode);
        }
        break;
    case PCCommand::SlavePushChassisTrajectory:
        break;
    case PCCommand::SetMasterChassisTargetCurrentState:
    case PCCommand::SetMasterChassisTargetPreviousCurve:
        if constexpr (ProjectParts::EnablePcControl && ProjectParts::EnableWheelChassis)
        {
            if (Chassis::ctrl == nullptr)
                break;
            const chassis::Posture target = { .x   = to_pos(read_i16(&data[0])),
                                              .y   = to_pos(read_i16(&data[2])),
                                              .yaw = to_angle(read_i16(&data[4])) };

            const auto xy_vmax_raw = static_cast<uint16_t>(static_cast<uint16_t>(data[6]) << 4 |
                                                           static_cast<uint16_t>(data[7]) >> 4);

            const auto xy_amax_raw = static_cast<uint16_t>(
                    (static_cast<uint16_t>(data[7]) & 0x0F) << 8 | static_cast<uint16_t>(data[8]));

            const auto yaw_vmax_raw = static_cast<uint16_t>(static_cast<uint16_t>(data[9]) << 4 |
                                                            static_cast<uint16_t>(data[10]) >> 4);

            const auto yaw_amax_raw = static_cast<uint16_t>((static_cast<uint16_t>(data[10]) & 0x0F)
                                                                    << 8 |
                                                            static_cast<uint16_t>(data[11]));

            const float xy_vmax  = static_cast<float>(xy_vmax_raw) / 200.0f;
            const float xy_amax  = static_cast<float>(xy_amax_raw) / 200.0f;
            const auto  yaw_vmax = static_cast<float>(yaw_vmax_raw);
            const auto  yaw_amax = static_cast<float>(yaw_amax_raw);

            const Chassis::ChassisController::TrajectoryLimit limit{
                .x = {
                    .max_spd = xy_vmax,
                    .max_acc = xy_amax,
                    .max_jerk = xy_amax * 50.0f,
                },
                .y = {
                    .max_spd = xy_vmax,
                    .max_acc = xy_amax,
                    .max_jerk = xy_amax * 50.0f,
                },
                .yaw = {
                    .max_spd = yaw_vmax,
                    .max_acc = yaw_amax,
                    .max_jerk = yaw_amax * 50.0f,
                },
            };

            const Chassis::ChassisController::TrajectoryLinkMode link_mode =
                    frame.cmd == PCCommand::SetMasterChassisTargetCurrentState
                            ? Chassis::ChassisController::TrajectoryLinkMode::CurrentState
                            : Chassis::ChassisController::TrajectoryLinkMode::PreviousCurve;

            Chassis::ctrl->setTargetPostureInWorld(target, link_mode, limit);
        }
        break;
    case PCCommand::SetMasterChassisVelocity:
        if constexpr (ProjectParts::EnablePcControl && ProjectParts::EnableWheelChassis)
        {
            if (Chassis::ctrl == nullptr)
                break;

            const chassis::Velocity target = { .vx = to_pos(read_i16(&data[0])),
                                               .vy = to_pos(read_i16(&data[2])),
                                               .wz = to_angle(read_i16(&data[4])) };

            Chassis::ctrl->setVelocityInBody(target, false);
        }
        break;
    case PCCommand::LidarPosture:
    {
        // LidarPosture 只在“上位机定位包”能力启用时才参与处理。
        // 否则当前工程使用本地定位，不消费外部位姿观测。
        if constexpr (!ProjectParts::EnablePcLocalization)
            break;

        if (!frame.from_main_protocol || !global_clock_.isStable())
            break;

        // 定位流连接状态统一通过 Watchdog 判定，单位是 EatAll() 消耗的 tick。
        lidar_posture_watchdog_.feed(LidarPostureTimeoutTicks);

        const chassis::Posture pos = { .x   = to_pos(read_i16(&data[0])),
                                       .y   = to_pos(read_i16(&data[2])),
                                       .yaw = to_angle(read_i16(&data[4])) };

        debug_.lidar.last_received_posture = pos;

        const uint32_t lidar_time      = read_u32(data.data() + 6);
        const uint32_t lidar_self_time = global_clock_.pcTime2SelfTime(lidar_time);

        debug_.lidar.last_received_posture_timestamp = lidar_self_time;
        debug_.lidar.last_received_timestamp         = HAL_GetTick();

        debug_.lidar.last_received_delay = static_cast<int32_t>(
                                                   debug_.lidar.last_received_timestamp) -
                                           static_cast<int32_t>(lidar_self_time);

        if (Chassis::motion == nullptr || !Chassis::motion->isReady())
            return;

        // 第一帧外部位姿的职责是“定义系统初值”：
        // - 记录 posture
        // - 触发 `System::Init::initPostureReceive()`
        // - 完成 Loc / Controller 的延迟构造
        if (!System::Init::postureReceived)
        {
            if (Device::Sensor::gyro_yaw == nullptr || !Device::Sensor::gyro_yaw->isConnected())
                return;

            System::Init::posture = pos;
            System::Init::initPostureReceive();
            System::Init::postureReceived = true;
            return;
        }

        Chassis::updateLidar(pos, lidar_self_time);
        break;
    }
    case PCCommand::StepUp:
    {
        // 台阶动作不是单一协议能力，而是“控制命令 + 底盘 + 升降”的组合能力。
        if constexpr (!ProjectParts::EnableStepAction)
            break;

        const float             startDistance = to_pos(read_i16(&data[0]));
        const float             endDistance   = to_pos(read_i16(&data[2]));
        const uint16_t          dir           = read_u16(&data[4]);
        const bool              willTake      = static_cast<bool>(read_u16(&data[6]));
        Action::Step::Direction direction;
        if (dir == 0)
            direction = Action::Step::Direction::Forward;
        else if (dir == 1)
            direction = Action::Step::Direction::Backward;
        else
            break;
        Action::Step::inst().up(startDistance, endDistance, direction, willTake);
        break;
    }
    case PCCommand::StepUpResume:
    {
        // 与 StepUp 相同，只有完整动作链启用时才处理恢复命令。
        if constexpr (!ProjectParts::EnableStepAction)
            break;

        auto& step = Action::Step::inst();
        if (step.isWaitingTake())
        {
            step.resume_up();
        }
        break;
    }
    case PCCommand::StepDown:
    {
        // 与 StepUp 相同，只有完整动作链启用时才处理下台阶命令。
        if constexpr (!ProjectParts::EnableStepAction)
            break;

        const float             startDistance = to_pos(read_i16(&data[0]));
        const float             endDistance   = to_pos(read_i16(&data[2]));
        const uint16_t          dir           = read_u16(&data[4]);
        const bool              shouldReset   = static_cast<bool>(read_u16(&data[6]));
        Action::Step::Direction direction;
        if (dir == 0)
            direction = Action::Step::Direction::Forward;
        else if (dir == 1)
            direction = Action::Step::Direction::Backward;
        else
            break;
        Action::Step::inst().down(startDistance, endDistance, direction, shouldReset);
        break;
    }
    case PCCommand::TakeSpear:
    {
        if constexpr (!ProjectParts::EnableSpearGrabAction)
            break;

        const chassis::Posture target = { .x   = to_pos(read_i16(&data[0])),
                                          .y   = to_pos(read_i16(&data[2])),
                                          .yaw = to_angle(read_i16(&data[4])) };
        const chassis::Posture end    = { .x   = to_pos(read_i16(&data[6])),
                                          .y   = to_pos(read_i16(&data[8])),
                                          .yaw = to_angle(read_i16(&data[10])) };

        Grip::Action::SpearGrab::inst().grab(target, end);
        break;
    }
    case PCCommand::TakeSpearById:
    {
        if constexpr (!ProjectParts::EnableSpearGrabAction)
            break;

        const uint16_t spear_id = read_u16(&data[0]);
        if (spear_id >= Grip::Config::SpearGrab::TargetPosCount)
            break;

        const chassis::Posture end = { .x   = to_pos(read_i16(&data[2])),
                                       .y   = to_pos(read_i16(&data[4])),
                                       .yaw = to_angle(read_i16(&data[6])) };

        Grip::Action::SpearGrab::inst().grab(Grip::Config::SpearGrab::TargetPoses[spear_id], end);
        break;
    }
    case PCCommand::StoreKFS:
        if constexpr (ProjectParts::EnableKfsAction)
            Grip::Action::KfsStore::inst().store();
        break;
    case PCCommand::ReleaseKFS:
        if constexpr (ProjectParts::EnableKfsAction)
            Grip::Action::KfsStore::inst().release();
        break;
    default:
        break;
    }
}

void PCProtocol::transmitFeedbackFrame(const std::array<uint8_t, FeedbackFrameLen>& frame)
{
    if (HAL_UART_Transmit_DMA(huart(), frame.data(), frame.size()) == HAL_OK)
        tx_state_ = TxState::DMAActive;
}

void PCProtocol::transmitIdentifyByte()
{
    if (HAL_UART_Transmit_DMA(huart(), identify_tx_buffer_.data(), identify_tx_buffer_.size()) ==
        HAL_OK)
    {
        tx_state_ = TxState::DMAActive;
    }
}

void PCProtocol::transmitCallback()
{
    tx_state_ = TxState::Idle;
}

void PCProtocol::transmitTaskStep(const std::array<uint8_t, FeedbackFrameLen>& feedback_frame)
{
    if (tx_state_ == TxState::Stopped || tx_state_ == TxState::DMAActive ||
        huart()->gState != HAL_UART_STATE_READY)
        return;

    if constexpr (ProjectParts::NeedUpperHostIdentifyInit)
    {
        if (isMainProtocol() && !isUpperHostIdentified())
            transmitIdentifyByte();
        else
            transmitFeedbackFrame(feedback_frame);
    }
    else
    {
        transmitFeedbackFrame(feedback_frame);
    }
}

bool PCProtocol::startTransmit()
{
    if (huart()->hdmatx == nullptr || huart()->hdmatx->Init.Mode != DMA_NORMAL)
        return false;

    tx_state_ = TxState::Idle;
    return true;
}

void PCProtocol::errorHandler()
{
    const bool has_tx_dma_error = (huart()->ErrorCode & HAL_UART_ERROR_DMA) != 0U &&
                                  huart()->hdmatx != nullptr &&
                                  huart()->hdmatx->ErrorCode != HAL_DMA_ERROR_NONE;

    if (has_tx_dma_error)
    {
        HAL_UART_AbortTransmit(huart());
        huart()->hdmatx->ErrorCode = HAL_DMA_ERROR_NONE;
        tx_state_                  = TxState::Idle;
    }

    protocol::UartRxSync<HeaderLen, FrameLen>::errorHandler();
}

bool isPcLocalizationConnected()
{
    return lidar_posture_watchdog_.isFed();
}

bool isUpperHostIdentified()
{
    return !ProjectParts::NeedUpperHostIdentifyInit || System::Init::upperHostIdentified;
}

void init()
{
    // 若完全不需要上位机协议，则不创建串口接收对象和处理线程。
    if constexpr (!ProjectParts::EnableUpperHostProtocol)
        return;

    if (pc_rx != nullptr)
        return;

    assert(config::uart::UpperHost->Init.BaudRate == 230400);

    constexpr osThreadAttr_t processor_attr{
        .name       = "pc-cmd-processor",
        .stack_size = 4096 * 4,
        .priority   = osPriorityRealtime,
    };

    constexpr osThreadAttr_t feedback_attr{
        .name       = "pc-feedback",
        .stack_size = 512 * 4,
        .priority   = osPriorityLow,
    };

    command_handler_task_ = osThreadNew(PCCommandHandlerTask, nullptr, &processor_attr);
    transmit_task_        = osThreadNew(TransmitTask, nullptr, &feedback_attr);

    pc_rx = new PCProtocol(config::uart::UpperHost, true);

    HAL_UART_RegisterCallback(config::uart::UpperHost,
                              HAL_UART_RX_COMPLETE_CB_ID,
                              [](UART_HandleTypeDef* huart) { pc_rx->receiveCallback(); });
    HAL_UART_RegisterCallback(config::uart::UpperHost,
                              HAL_UART_ERROR_CB_ID,
                              [](UART_HandleTypeDef* huart) { pc_rx->errorHandler(); });
    HAL_UART_RegisterCallback(config::uart::UpperHost,
                              HAL_UART_TX_COMPLETE_CB_ID,
                              [](UART_HandleTypeDef* huart) { pc_rx->transmitCallback(); });

    if (!pc_rx->startReceive())
        Error_Handler();

    if (!pc_rx->startTransmit())
        Error_Handler();

    static PCProtocol ctrl_rx{ config::uart::AuxControllerHost, false };

    HAL_UART_RegisterCallback(config::uart::AuxControllerHost,
                              HAL_UART_RX_COMPLETE_CB_ID,
                              [](UART_HandleTypeDef* huart) { ctrl_rx.receiveCallback(); });
    HAL_UART_RegisterCallback(config::uart::AuxControllerHost,
                              HAL_UART_ERROR_CB_ID,
                              [](UART_HandleTypeDef* huart) { ctrl_rx.errorHandler(); });
    HAL_UART_RegisterCallback(config::uart::AuxControllerHost,
                              HAL_UART_TX_COMPLETE_CB_ID,
                              [](UART_HandleTypeDef* huart) { ctrl_rx.transmitCallback(); });

    if (!ctrl_rx.startReceive())
        Error_Handler();

    if (!ctrl_rx.startTransmit())
        Error_Handler();

    osThreadFlagsSet(transmit_task_, FeedbackStart);
}

} // namespace Protocol
