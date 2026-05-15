/**
 * @file    PCCommandHandler.cpp
 * @author  syhanjin
 * @date    2026-05-08
 */
#include "PCCommandHandler.hpp"

#include "RingBuffer.hpp"
#include "chassis/Config.hpp"
#include "chassis/actions/Step.hpp"
#include "chassis/chassis.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "grip/Config.hpp"
#include "grip/actions/roller_store.hpp"
#include "grip/actions/spear_grab.hpp"
#include "grip/grip.hpp"
#include "motor_trajectory.hpp"
#include "project_parts.hpp"
#include "system.hpp"
#include "watchdog.hpp"

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

constexpr uint32_t MsgReceived = 1 << 0;

constexpr uint32_t LidarPostureTimeoutTicks = 200U;
constexpr uint32_t VisionPostureTimeoutTicks = 200U;

libs::RingBuffer<Frame, 10> command_buffer_{};

osThreadId_t command_handler_task_{};

Sync::Clock global_clock_{};

service::Watchdog lidar_posture_watchdog_{};
service::Watchdog vision_posture_watchdog_{};

struct
{
    struct
    {
        chassis::Posture last_received_posture{};
        uint32_t         last_received_posture_timestamp{};
        uint32_t         last_received_timestamp{};
        int32_t          last_received_delay{};
    } lidar;
    struct
    {
        chassis::Posture last_received_posture{};
        uint32_t         last_received_posture_timestamp{};
        uint32_t         last_received_timestamp{};
        int32_t          last_received_delay{};
    } vision;
} debug_{};

bool to_grip_preset_pose(const uint16_t preset_id)
{
    if (Grip::grip == nullptr || !Grip::grip->enabled())
        return false;

    switch (preset_id)
    {
    case 0:
        return Grip::grip->toStandbyPose();
    case 1:
        return Grip::grip->toPrepareGrabPose();
    case 2:
        return Grip::grip->toGrabPose();
    case 3:
        return Grip::grip->toDockingPose();
    case 4:
        return Grip::grip->toKfsPickupPose();
    case 5:
        return Grip::grip->toKfsStorePose();
    case 6:
        return Grip::grip->toKfsReleasePose();
    default:
        return false;
    }
}

void handle_external_posture(const Chassis::ExternalSource source,
                             const chassis::Posture&       posture,
                             const uint32_t                self_time)
{
    if (Chassis::motion == nullptr || !Chassis::motion->isReady())
        return;

    if (Chassis::externalSource() != source)
        return;

    if (!System::Init::postureReceived)
    {
        if (Device::Sensor::gyro_yaw == nullptr || !Device::Sensor::gyro_yaw->isConnected())
            return;

        System::Init::posture = Chassis::externalObservationToWorldForInit(source, posture);
        System::Init::initPostureReceive();
        System::Init::postureReceived = true;
        return;
    }

    Chassis::updateExternalObservation(source, posture, self_time);
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
    case PCCommand::SetGripPose:
        if constexpr (ProjectParts::EnablePcControl && ProjectParts::EnableGrip)
        {
            if (Grip::grip == nullptr || !Grip::grip->enabled())
                break;

            const Grip::Config::JointPose pose{ .arm_pos  = to_angle(read_i16(&data[0])),
                                                .turn_pos = to_angle(read_i16(&data[2])) };

            switch (read_u16(&data[4]))
            {
            case 0:
                break;
            case 1:
                Grip::grip->openClaw();
                break;
            case 2:
                Grip::grip->closeClaw();
                break;
            default:
                break;
            }

            Grip::grip->toJointPose(pose);
        }
        break;
    case PCCommand::SetGripPresetPose:
        if constexpr (ProjectParts::EnablePcControl && ProjectParts::EnableGrip)
        {
            to_grip_preset_pose(read_u16(&data[0]));
        }
        break;
    case PCCommand::SetLocalizationSource:
        if constexpr (ProjectParts::EnablePcLocalization)
        {
            const uint16_t mode = read_u16(&data[0]);
            Chassis::ExternalSource source = Chassis::ExternalSource::None;
            bool valid = true;
            switch (mode)
            {
            case 0:
                source = Chassis::ExternalSource::None;
                break;
            case 1:
                source = Chassis::ExternalSource::Lidar;
                break;
            case 2:
                source = Chassis::ExternalSource::Vision;
                break;
            default:
                valid = false;
                break;
            }

            if (valid)
                Chassis::switchExternalSource(source);
        }
        break;
    case PCCommand::ResetLocalizationFrame:
        if constexpr (ProjectParts::EnablePcLocalization)
        {
            const auto source = Chassis::externalSource();
            if (source == Chassis::ExternalSource::Lidar ||
                source == Chassis::ExternalSource::Vision)
            {
                Chassis::switchExternalSource(source, true);
            }
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
        // 原本的updateLidar()被封装为 handle_external_posture()中的updateExternalObservation
        handle_external_posture(Chassis::ExternalSource::Lidar, pos, lidar_self_time);
        break;
    }
    case PCCommand::VisionPosture:
    {
        if constexpr (!ProjectParts::EnablePcLocalization)
            break;

        if (!frame.from_main_protocol || !global_clock_.isStable())
            break;

        vision_posture_watchdog_.feed(VisionPostureTimeoutTicks);

        const chassis::Posture pos = { .x   = to_pos(read_i16(&data[0])),
                                       .y   = to_pos(read_i16(&data[2])),
                                       .yaw = to_angle(read_i16(&data[4])) };

        debug_.vision.last_received_posture = pos;

        const uint32_t vision_time      = read_u32(data.data() + 6);
        const uint32_t vision_self_time = global_clock_.pcTime2SelfTime(vision_time);

        debug_.vision.last_received_posture_timestamp = vision_self_time;
        debug_.vision.last_received_timestamp         = HAL_GetTick();
        debug_.vision.last_received_delay = static_cast<int32_t>(
                                                    debug_.vision.last_received_timestamp) -
                                            static_cast<int32_t>(vision_self_time);

        handle_external_posture(Chassis::ExternalSource::Vision, pos, vision_self_time);
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
} // namespace

const Sync::Clock& clock()
{
    return global_clock_;
}

bool isPcLidarLocalizationConnected()
{
    return Chassis::externalSource() == Chassis::ExternalSource::Lidar &&
           lidar_posture_watchdog_.isFed();
}

bool isPcVisionLocalizationConnected()
{
    return Chassis::externalSource() == Chassis::ExternalSource::Vision &&
           vision_posture_watchdog_.isFed();
}

bool isPcLocalizationConnected()
{
    return isPcLidarLocalizationConnected() || isPcVisionLocalizationConnected();
}

bool isUpperHostIdentified()
{
    return !ProjectParts::NeedUpperHostIdentifyInit || System::Init::upperHostIdentified;
}

namespace CommandHandler
{

bool enqueueFrame(const Frame& frame)
{
    const bool pushed = command_buffer_.push(frame);
    if (pushed && command_handler_task_ != nullptr)
        osThreadFlagsSet(command_handler_task_, MsgReceived);

    return pushed;
}

void startTask()
{
    if (command_handler_task_ != nullptr)
        return;

    constexpr osThreadAttr_t processor_attr{
        .name       = "pc-cmd-processor",
        .stack_size = 4096 * 4,
        .priority   = osPriorityRealtime,
    };

    command_handler_task_ = osThreadNew(PCCommandHandlerTask, nullptr, &processor_attr);
}

} // namespace CommandHandler
} // namespace Protocol
