#include "pid.h"


#define DBG_SECTION_NAME  "pid"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

rt_err_t pid_init(pid_control_t* pid, float kp, float ki, float kd, float limit_max, float limit_min, rt_uint16_t sample_time)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    pid->maximum = limit_max;
    pid->minimum = limit_min;
    pid->anti_windup_value = pid->maximum * 2.0f;

    pid->sample_time = sample_time;

    pid->p_error = 10.0f;
    pid->i_error = 1.0f;
    pid->d_error = 0.2f;

    pid->integral = 0.0f;
    pid->error = 0.0f;
    pid->error_l = 0.0f;
    pid->error_ll = 0.0f;

    pid->out = 0.0f;
    pid->last_out = 0.0f;
    
    return RT_EOK;
}

rt_err_t pid_reset(pid_control_t* pid)
{
    // TODO
    rt_memset(pid, 0, sizeof(pid_control_t));
    return RT_EOK;
}

rt_err_t pid_set_kp(pid_control_t* pid, float kp)
{
    pid->kp = kp;
    return RT_EOK;
}

rt_err_t pid_set_ki(pid_control_t* pid, float ki)
{
    pid->ki = ki;
    return RT_EOK;
}

rt_err_t pid_set_kd(pid_control_t* pid, float kd)
{
    pid->kd = kd;
    return RT_EOK;
}
rt_err_t pid_set_kpid(pid_control_t* pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    return RT_EOK;
}
rt_err_t pid_set_target(pid_control_t* pid, float target_point)
{
    // TODO
    pid->target_point = target_point;

    return RT_EOK;
}

rt_err_t pid_set_anti_windup_value(pid_control_t* pid, float anti_windup_value)
{
    pid->anti_windup_value = anti_windup_value;
    return RT_EOK;
}

rt_err_t pid_set_sample_time(pid_control_t* pid, rt_uint16_t sample_time)
{
    // TODO
    pid->sample_time = sample_time;
    return RT_EOK;
}

float pid_update_sample(pid_control_t* pid, float measure_value)
{
    // TODO
    if((rt_tick_get() - pid->last_time) < rt_tick_from_millisecond(pid->sample_time))
    {
        LOG_D("PID waiting ... ");
        return pid->last_out;
    }

    pid->last_time = rt_tick_get();
    pid->error = pid->target_point - measure_value;

    pid->integral += pid->error;

    //Perform integral value capping to avoid internal PID state to blows up
    //when actuators saturate:
    if(pid->integral > pid->anti_windup_value) {
        pid->integral = pid->anti_windup_value;
    } else if (pid->integral < -pid->anti_windup_value) {
        pid->integral = -pid->anti_windup_value;
    }

    pid->p_error = pid->kp * pid->error;
    pid->i_error = pid->ki * pid->integral;
    pid->d_error = pid->kd * (pid->error - 2 * pid->error_l + pid->error_ll);

    pid->out = pid->p_error + pid->i_error + pid->d_error;
    if (pid->out > pid->maximum)
    {
        pid->out = pid->maximum;
    }
    if (pid->out < pid->minimum)
    {
        pid->out = pid->minimum;
    }

    pid->last_out = pid->out;
    pid->error_ll = pid->error_l;
    pid->error_l = pid->error;

    rt_kprintf("%d - %d - %d\n", measure_value, pid->target_point, (int)pid->out);
    //LOG_D("PID current: %d : setpoint %d - P%d I%d D%d - [%d]", current_point, pid->set_point, (int)(pid->p_error + 0.5f), (int)(pid->i_error + 0.5f), (int)(pid->d_error + 0.5f), (int)(pid->out + 0.5f));
    // LOG_D("PID P Error: %d", (int)(pid->p_error + 0.5f));
    // LOG_D("PID I Error: %d", (int)(pid->i_error + 0.5f));
    // LOG_D("PID D Error: %d", (int)(pid->d_error + 0.5f));
    // LOG_D("PID Last Out: %d", (int)(pid->last_out + 0.5f));
    // LOG_D("PID Out: %d", (int)(pid->out + 0.5f));

    return pid->out;
}

float pid_update(pid_control_t* pid, float measure_value)
{
    //DEBUG
    float waveform[3] = {0};
    
    pid->error = pid->target_point - measure_value;

    pid->integral += pid->error;

    //Perform integral value capping to avoid internal PID state to blows up
    //when actuators saturate:
    if(pid->integral > pid->anti_windup_value) {
        pid->integral = pid->anti_windup_value;
    } else if (pid->integral < -pid->anti_windup_value) {
        pid->integral = -pid->anti_windup_value;
    }

    pid->p_error = pid->kp * pid->error;
    pid->i_error = pid->ki * pid->integral;
    pid->d_error = pid->kd * (pid->error - 2 * pid->error_l + pid->error_ll);

    pid->out = pid->p_error + pid->i_error + pid->d_error;
    if (pid->out > pid->maximum)
    {
        pid->out = pid->maximum;
    }
    if (pid->out < pid->minimum)
    {
        pid->out = pid->minimum;
    }

    pid->last_out = pid->out;
    pid->error_ll = pid->error_l;
    pid->error_l = pid->error;

    //DEBUG
    //rt_kprintf("%d - %d - %d\n", measure_value, pid->target_point, (int)pid->out);
    waveform[0] = measure_value;
    waveform[1] = pid->target_point;
    waveform[2] = pid->out;
    send_waveform_fomate(waveform, sizeof(waveform));
    
    return pid->out;
}

rt_err_t send_waveform_fomate(void *buf, uint32_t size)
{
    const char start[2] = {0x03, 0xfc};
    const char end[2]   = {0xfc, 0x03};
    rt_device_t console = rt_console_get_device();
    
    rt_device_write(console, -1, start, 2);
    rt_device_write(console, -1, buf, size);
    rt_device_write(console, -1, end, 2);

    return RT_EOK;
}
