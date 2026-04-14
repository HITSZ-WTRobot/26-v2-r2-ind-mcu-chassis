#include "grip.hpp"
#include "motor_trajectory.hpp"
#include "cmsis_os2.h"
#include "can.h"
#include "main.h"
#include "motor_vel_controller.hpp"
#include <cstdint>
#include <cstdlib>

using GripState = Action::Grip::GripState;

namespace Gripping {
motors::DJIMotor *parm_motor;
motors::DJIMotor *pturn_motor;
controllers::MotorVelController *parm_vel_controller;
controllers::MotorVelController *pturn_vel_controller;
trajectory::MotorTrajectory<1> *parm_trajectory;
trajectory::MotorTrajectory<1> *pturn_trajectory;

controllers::MotorVelController *parm_locked_vel;
controllers::MotorVelController *pturn_locked_vel;
}

namespace Action {

namespace Grip_param {
    /* 待定 */
    const float Arm_nowork_pos = 130.0f;  ///< 臂的不工作位置,OK
    const float Arm_grip_pos = 54.0f;  ///< 臂的准备夹取位置, OK
    const float Arm_out_pos = 100.0f;  ///< 臂上摆取出端头,OK
    const float Turn_grip_pos = 235.0f;  ///< 夹爪旋转的准备夹取位置, OK
    const float Turn_docking_pos = 145.0f;  ///< 夹爪旋转的对接位置, OK

    // 电机配置参数
    motors::DJIMotor::Config arm_motor_cfg = {
        .hcan = &hcan1,
        .type = motors::DJIMotor::Type::M3508_C620,
        .id1 = 3,
    };
    motors::DJIMotor::Config turn_motor_cfg = {
        .hcan = &hcan1,
        .type = motors::DJIMotor::Type::M2006_C610,
        .id1 = 4,
    };
    // 电机速度环配置参数
    controllers::MotorVelController::Config arm_vel_controller_cfg = {
        .pid = {
            .Kp = 370.0f,
            .Ki = 30.0f,
            .Kd = 0.0f,
            .abs_output_max = 12000.0f
        },
    };
    controllers::MotorVelController::Config turn_vel_controller_cfg = {
        .pid = {
            .Kp = 400.0f,
            .Ki = 5.0f,
            .Kd = 0.0f,
            .abs_output_max = 2000.0f
        },
    };
    // S曲线配置参数
    velocity_profile::SCurveProfile::Config arm_trajectory_cfg = {
        .max_spd = 360.0f, // deg/s
        .max_acc = 720.0f, // deg/s^2
        .max_jerk = 1440.0f // deg/s^3
    };
    PD::Config arm_trajectory_pd_cfg = {
        .Kp = 50.0f,
        .Kd = 5.0f,
    };
    velocity_profile::SCurveProfile::Config turn_trajectory_cfg = {
        .max_spd = 360.0f, // deg/s
        .max_acc = 720.0f, // deg/s^2
        .max_jerk = 1440.0f // deg/s^3
    };
    PD::Config turn_trajectory_pd_cfg = {
        .Kp = 50.0f,
        .Kd = 1.0f,
    };
} // namespace Grip_param

Grip::Grip()
{
    grip_state_ = GripState::NOWORK;
    
    const osThreadAttr_t grip_test_Task_attributes = {
        .name       = "grip_test_Task",
        .stack_size = 256 * 8,
        .priority   = (osPriority_t)osPriorityNormal
    };
    task_ = osThreadNew(m_grip_test_Task, nullptr, &grip_test_Task_attributes);
}

void Grip::ready()
{
    grip_state_ = GripState::READY;
    osThreadFlagsSet(task_, flag_start);
}

void Grip::gripping()
{
    grip_state_ = GripState::GRIPPING;
    osThreadFlagsSet(task_, flag_start);
}

void Grip::docking()
{
    grip_state_ = GripState::DOCKING;
    osThreadFlagsSet(task_, flag_start);
}

void Grip::open()
{
    grip_state_ = GripState::OPEN;
    osThreadFlagsSet(task_, flag_start);
}

void Grip::nowork()
{
    grip_state_ = GripState::NOWORK;
    osThreadFlagsSet(task_, flag_start);
}

/* 待具体写 */
// 矛头夹取和对接操作
void Grip::start()
{
    switch (grip_state_) {
        // 不工作，夹爪保持张开
        case GripState::NOWORK:
            Gripping::parm_trajectory->setTarget(Grip_param::Arm_nowork_pos);
            Gripping::pturn_trajectory->setTarget(Grip_param::Turn_grip_pos);
            setGrip(GripState::NOWORK);   // 夹爪保持张开
            break;
        // 夹爪到位，准备夹取
        case GripState::READY:
            Gripping::parm_trajectory->setTarget(Grip_param::Arm_grip_pos);
            Gripping::pturn_trajectory->setTarget(Grip_param::Turn_grip_pos);
            setGrip(GripState::NOWORK);   // 夹爪保持张开
            break;
        // 夹取，并取出端头
        case GripState::GRIPPING:
            setGrip(GripState::GRIPPING);
            Gripping::parm_trajectory->setTarget(Grip_param::Arm_out_pos);
            Gripping::pturn_trajectory->setTarget(Grip_param::Turn_grip_pos);
            break;  
        // 对接
        case GripState::DOCKING:
            Gripping::parm_trajectory->setTarget(Grip_param::Arm_grip_pos);
            Gripping::pturn_trajectory->setTarget(Grip_param::Turn_docking_pos);
            break;
        // 夹爪张开, 让R1取走端头
        case GripState::OPEN:
            setGrip(GripState::NOWORK);
            break;
        case GripState::NOSTART:
             break;
        default:
            break;
    }
}

// 电机是否到达目标位置
bool Grip::isFinished()
{
    return Gripping::parm_trajectory->isFinished() && Gripping::pturn_trajectory->isFinished();
}
// 等待电机到达目标位置
void Grip::waitForFinish()
{
    while (!isFinished())
        osDelay(10);
}

/* 需要R1发来消息，具体实现待定 */
// 等待对接完成
void Grip::waitForDocking()
{

}
// 所有电机堵转到限位处进行初始化
void Grip::locked_all_init() {
    // 电机堵转到限位处
    locked_set_initPosition(Gripping::parm_motor, Gripping::parm_locked_vel, 
        (Grip::locked_config){.locked_output = 1000.0f, .locked_deadAngle = 0.1f, .locked_speed = -30.0f, .locked_test_interval = 100});
    locked_set_initPosition(Gripping::pturn_motor, Gripping::pturn_locked_vel, 
        (Grip::locked_config){.locked_output = 500.0f, .locked_deadAngle = 0.1f, .locked_speed = -30.0f, .locked_test_interval = 100});
}

/* 待具体写 */
// 判断R2是否到达目标位置 
/* 或许这个判断可以仅放在决策层 */
bool Grip::isInPlace() {
    return false;
}

// 末端夹爪夹取动作
void Grip::setGrip(GripState grip_state) {
    HAL_GPIO_WritePin(GRIP_OUT_GPIO_Port, GRIP_OUT_Pin, static_cast<int>(grip_state) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// 电机堵转到限位处
void Grip::locked_set_initPosition(motors::DJIMotor* motor, controllers::MotorVelController* locked_vel_controller, const locked_config& cfg) {
    // 以较小的速度向夹取位置运动，直到电机堵转
    float now_angle = 0.0f;
    bool is_locked = false;

    locked_vel_controller->enable();
    locked_vel_controller->setRef(cfg.locked_speed); // 设置一个较小的速度
    while (!is_locked) { 
        if (std::abs(motor->getIqCMD()) > cfg.locked_output) {
            now_angle = motor->getAngle();
            osDelay(cfg.locked_test_interval); // 等待一段时间再检测
            if (std::abs(motor->getAngle() - now_angle) < cfg.locked_deadAngle) { // 如果角度变化小于某个阈值，认为电机堵转
                is_locked = true;
                break;
            }
        }
    }
    locked_vel_controller->getMotor()->resetAngle(); // 将当前位置设为零，即夹取位置
    locked_vel_controller->setRef(0.0f); // 停止电机
    locked_vel_controller->disable();        // 禁止用于堵转检测的速度控制器输出
}
// 任务函数
void m_grip_test_Task(void *argument)
{

    for(;;)
    {
        osThreadFlagsWait(flag_start, osFlagsWaitAll, osWaitForever);
        Gripping::grip->start();      // 夹爪到位，准备夹取
        osDelay(1);
    }
}

} // namespace Action

namespace Gripping {

void motor_init() {
    parm_motor = new motors::DJIMotor(Action::Grip_param::arm_motor_cfg);
    pturn_motor = new motors::DJIMotor(Action::Grip_param::turn_motor_cfg);
    parm_vel_controller = new controllers::MotorVelController(parm_motor, Action::Grip_param::arm_vel_controller_cfg);
    pturn_vel_controller = new controllers::MotorVelController(pturn_motor, Action::Grip_param::turn_vel_controller_cfg);
    parm_trajectory = new trajectory::MotorTrajectory<1>(parm_vel_controller, Action::Grip_param::arm_trajectory_cfg, Action::Grip_param::arm_trajectory_pd_cfg);
    pturn_trajectory = new trajectory::MotorTrajectory<1>(pturn_vel_controller, Action::Grip_param::turn_trajectory_cfg, Action::Grip_param::turn_trajectory_pd_cfg);

    pturn_locked_vel = new controllers::MotorVelController(pturn_motor, Action::Grip_param::arm_vel_controller_cfg);
    parm_locked_vel = new controllers::MotorVelController(parm_motor, Action::Grip_param::turn_vel_controller_cfg);

    grip = new Action::Grip();
    Gripping::grip->setGrip(Action::Grip::GripState::NOWORK);   // 默认夹爪打开
}

void init() {
    // DJI_Control_Init();
    motor_init();
}

void Wait_all_motor_connected() {
    while (!parm_motor->isConnected() || !pturn_motor->isConnected()) {
        osDelay(10);
    }
}

void update_1kHz() {
    parm_trajectory->profileUpdate(0.001f);
    parm_trajectory->controllerUpdate();
    pturn_trajectory->profileUpdate(0.001f);
    pturn_trajectory->controllerUpdate();

    pturn_locked_vel->update();
    parm_locked_vel->update();
}

void enable() {
    parm_trajectory->enable();
    pturn_trajectory->enable();
}

}
