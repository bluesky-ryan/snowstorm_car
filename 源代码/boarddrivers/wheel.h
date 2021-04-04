#ifndef __WHEEL_H__
#define __WHEEL_H__
#include <stm32f1xx.h>
#include <board.h>
#include "pid.h"
#include "motor.h"
#include "kalman.h"
#include "common.h"

#define PI 3.1415926


#define WHEEL_CIRCLE_ENCODE     1560

#define PID_KP_VALUE            1.1
#define PID_KI_VALUE            0.4
#define PID_KD_VALUE            0.5

#define PID_KP_CH4B             0.26
#define PID_KI_CH4B             0.2
#define PID_KD_CH4B             0.01

#define PID_KP_CH4F             0.765
#define PID_KI_CH4F             0.33
#define PID_KD_CH4F             0.1

#define PID_LIMIT_MAX           80
#define PID_LIMIT_MIN          -80
#define PID_SAMPLE_TIM          10

#define KALMAN_COV_R            0.1
#define KALMAN_COV_Q            0


#define CHX_PID_KX_TABLE      \
{ \
    {MOTOR_CH1, {1.100, 0.400, 0.500}, {1.100, 0.400, 0.500}},\
    {MOTOR_CH2, {1.100, 0.400, 0.500}, {1.100, 0.400, 0.500}},\
    {MOTOR_CH3, {1.100, 0.400, 0.500}, {1.100, 0.400, 0.500}},\
    {MOTOR_CH4, {0.765, 0.330, 0.100}, {0.260, 0.200, 0.010}},\
}


/* ���ֽṹ�壬��ʾһ������ */
typedef struct
{
    motor_chx       m_motor_ch;             /* ���ͨ�� */
    pid_control_t   m_pid;                  /* PID���� */
    int16_t         m_current_encode;       /* ��ǰ�������ֵ */
    float           m_current_speed;        /* ��ǰ�����ٶ�: m/s��rpm ��rps ��λ����ʵ��ʹ�ö��� */
    float           m_target_speed;         /* �û������ٶ� */
    KalmanInfo      m_kalman;               /* �������˲� */
}wheel_t;

typedef struct
{
    motor_chx       m_motor_ch;
    pid_kx_param_t  pid_kx_forward; /* ǰ��ʱPID���� */
    pid_kx_param_t  pid_kx_reverse; /* ����ʱPID���� */
}wheel_chx_pid_t;




void        wheel_init(wheel_t *wheel, motor_chx chx);
float       wheel_encode_to_rpm(int16_t encode, uint32_t start_time, uint32_t end_time);
float       wheel_encode_to_velocity(int16_t encode, uint32_t start_time, uint32_t end_time);
rt_err_t    wheel_select_pid_kx(wheel_t *wheel);


#endif
