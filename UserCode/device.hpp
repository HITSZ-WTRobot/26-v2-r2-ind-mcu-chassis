/**
 * @file    device.hpp
 * @author  syhanjin
 * @date    2026-03-02
 */
#pragma once
#include "ActionOPS.hpp"
#include "HWT101CT.hpp"
#include "dji.hpp"
#include "usart.h"

// 传感器声明
/**
 * 陀螺仪
 * HWT101CT， UART2
 */
extern sensors::gyro::HWT101CT* sensor_gyro_yaw;
#define DEVICE_SENSOR_GYRO_YAW_UART (&huart2)

/**
 * 码盘
 * Action-OPS, UART6
 */
extern sensors::ops::ActionOPS* sensor_ops;
#define DEVICE_SENSOR_OPS_UART (&huart5)

// 电机声明

/**
 * 底盘使用的轮子
 * 大疆电机 3508, CAN1 ID: 1 ~ 4
 */
extern motors::DJIMotor* motor_wheel[4];

/**
 * 升降电机
 * 以使得底盘抬升的方向为正
 */
extern motors::DJIMotor* motor_lift_front; // 前抬升
extern motors::DJIMotor* motor_lift_rear;  // 后抬升

void Device_Init();
bool Device_isAllConnected();
void Device_WaitAllConnections();
void Device_Update_1kHz();