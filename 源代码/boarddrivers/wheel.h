#ifndef __WHEEL_H__
#define __WHEEL_H__
#include <stm32f1xx.h>
#include <board.h>
#include "pid.h"
#include "motor.h"

#define PI 3.1415926


#define WHEEL_CIRCLE_ENCODE     1560

#define PID_KP_VALUE            1.1
#define PID_KI_VALUE            0.4
#define PID_KD_VALUE            0.5

#define PID_KP_CH4B             0.26
#define PID_KI_CH4B             0.2
#define PID_KD_CH4B             0.01

#define PID_KP_CH4F              0.765
#define PID_KI_CH4F              0.33
#define PID_KD_CH4F              0.1

#define PID_LIMIT_MAX           80
#define PID_LIMIT_MIN          -80
#define PID_SAMPLE_TIM          10


#define CHX_PID_KX_TABLE      \
{ \
    {MOTOR_CH1, {1.100, 0.400, 0.500}, {1.100, 0.400, 0.500}},\
    {MOTOR_CH2, {1.100, 0.400, 0.500}, {1.100, 0.400, 0.500}},\
    {MOTOR_CH3, {1.100, 0.400, 0.500}, {1.100, 0.400, 0.500}},\
    {MOTOR_CH4, {0.765, 0.330, 0.100}, {0.260, 0.200, 0.010}},\
}


/* 车轮结构体，表示一个车轮 */
typedef struct
{
    motor_chx       m_motor_ch;             /* 电机通道 */
    pid_control_t   m_pid;                  /* PID控制 */
    int16_t         m_current_encode;       /* 当前编码计数值 */
    float           m_current_speed;        /* 当前车轮速度: m/s、rpm 、rps 单位根据实际使用而定 */
    float           m_target_speed;         /* 用户设置速度 */
}wheel_t;

typedef struct
{
    motor_chx       m_motor_ch;
    pid_kx_param_t  pid_kx_forward; /* 前进时PID参数 */
    pid_kx_param_t  pid_kx_reverse; /* 后退时PID参数 */
}wheel_chx_pid_t;




void        wheel_init(wheel_t *wheel, motor_chx chx);
float       wheel_encode_to_rpm(int16_t encode, uint32_t start_time, uint32_t end_time);
float       wheel_encode_to_velocity(int16_t encode, uint32_t start_time, uint32_t end_time);
rt_err_t    wheel_select_pid_kx(wheel_t *wheel);


#endif
