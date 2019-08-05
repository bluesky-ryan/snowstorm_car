#include "wheel.h"
#include "pid.h"
#include <math.h>


wheel_chx_pid_t chx_pid_kx_table[] = CHX_PID_KX_TABLE;


/**
 * @ingroup motor
 *
 * 计算轮子前进速度
 *
 * @param  wheel        轮子结构体
 * @param  motor_chx    初始化通道
 * @retrun void
 */
void wheel_init(wheel_t *wheel, motor_chx motor_chx)
{
    if (NULL == wheel)
        return;

    wheel->m_motor_ch       = motor_chx;
    wheel->m_target_speed   = 0;
    wheel->m_current_speed   = 0;
    wheel->m_current_encode = 0;

    /* 初始化PID */
    if (MOTOR_CH4 == motor_chx)
        pid_init(&wheel->m_pid, PID_KP_CH4F, PID_KI_CH4F, PID_KD_CH4F, PID_LIMIT_MAX, PID_LIMIT_MIN, PID_SAMPLE_TIM);
    else
        pid_init(&wheel->m_pid, PID_KP_VALUE, PID_KI_VALUE, PID_KD_VALUE, PID_LIMIT_MAX, PID_LIMIT_MIN, PID_SAMPLE_TIM);
    
    
}

/**
 * @ingroup motor
 *
 * 计算轮子前进速度
 *
 * @param  encode       编码器计数
 * @param  start_time   计数开始时间
 * @param  end_time     计数结束时间
 * @retrun float        返回计数后的转速，单位:rp/min
 */
float wheel_encode_to_rpm(int16_t encode, uint32_t start_time, uint32_t end_time)
{
    uint32_t used_time = end_time - start_time;
    float    rpm = 0.0f;

    if (used_time <= 0)
        return 0.0f;
    
    /* rpm = ((encode / time) * 1000 * 60 ) / circle_encode */
    rpm = (float)(encode * 1000 * 60) / (float)(used_time * WHEEL_CIRCLE_ENCODE);

    return rpm;
    
}

/**
 * @ingroup motor
 *
 * 计算轮子前进速度
 *
 * @param  encode       编码器计数
 * @param  start_time   计数开始时间
 * @param  end_time     计数结束时间
 * @retrun float        返回计数后的速度，单位:m/s
 */
float wheel_encode_to_velocity(int16_t encode, uint32_t start_time, uint32_t end_time)
{
    uint32_t used_time = end_time - start_time;
    float    velocity  = 0;

    if (used_time <= 0)
        return 0.0f;
    
    /* velocity = (encode / used_time) * 1000 * 2PI * (60 / 1000) 单位: m/s */
    velocity = ((float)(encode / used_time) * 2 * PI * 60) / WHEEL_CIRCLE_ENCODE;

    return velocity; 
}


/**
 * @ingroup wheel
 *
 * 根据轮子的速度选择合适的PID参数
 *
 * @param  wheel        轮子结构体
 * @param  speed        当前测量速度
 * @retrun rt_err_t     选择成功返回RT_EOK
 */
rt_err_t  wheel_select_pid_kx(wheel_t *wheel)
{
    for (int i = 0; i < ARRAY_SIZE(chx_pid_kx_table); i++)
    {
        if (wheel->m_motor_ch == chx_pid_kx_table[i].m_motor_ch)
        {
            if (wheel->m_current_speed >= 0)
            {
                wheel->m_pid.kp = chx_pid_kx_table[i].pid_kx_forward.kp;
                wheel->m_pid.ki = chx_pid_kx_table[i].pid_kx_forward.ki;
                wheel->m_pid.kd = chx_pid_kx_table[i].pid_kx_forward.kd;
            }
            else
            {
                wheel->m_pid.kp = chx_pid_kx_table[i].pid_kx_reverse.kp;
                wheel->m_pid.ki = chx_pid_kx_table[i].pid_kx_reverse.ki;
                wheel->m_pid.kd = chx_pid_kx_table[i].pid_kx_reverse.kd;
            }

            return RT_EOK;
        }
    }
    
    return -RT_EINVAL;
}


