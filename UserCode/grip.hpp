#ifndef GRIP_PNEUMATIC_H_
#define GRIP_PNEUMATIC_H_
#include "motor_vel_controller.hpp"
#include "motor_trajectory.hpp"
#include "dji.hpp"
#include <cstdint>
#include "cmsis_os2.h"

namespace Action {

constexpr uint32_t flag_start = 1;

class Grip
{
public:
    Grip();

    struct locked_config {
        float locked_output = 2000.0f; // 用于判断堵转的输出阈值
        float locked_deadAngle = 0.1f; // 用于判断堵转的角度变化阈值(°)
        float locked_speed = 30.0f; // 堵转时的速度(rpm)
        uint32_t locked_test_interval = 100; // 堵转检测的时间间隔(ms)
    };

    enum class GripState
    {
        NOWORK = 0,
        READY,
        GRIPPING,
        DOCKING,
        OPEN, 
        NOSTART
    };
    void ready();
    void gripping();
    void docking();
    void open();
    void nowork();

    void start();
    void setGrip(GripState grip_state);
    bool isFinished();
    void waitForFinish();
    void waitForDocking();
    void locked_all_init();

    /* R2定位是否到达目标位置 */
    bool isInPlace();

    GripState getState() const { return grip_state_; }  // 返回当前状态
    void locked_set_initPosition(motors::DJIMotor* motor, controllers::MotorVelController* locked_vel_controller, const locked_config& cfg);
    static void m_grip_test_Task(void *argument);

private:
    osThreadId_t task_;
    GripState grip_state_ = GripState::NOSTART;
};

}

namespace Gripping {

inline Action::Grip *grip;

void DJI_Control_Init();
void motor_init(); 
void init();
void Wait_all_motor_connected();
void update_1kHz();
void enable();

}

#endif
