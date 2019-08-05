#ifndef __PID_H__
#define __PID_H__

#include <rtthread.h>

typedef struct pid_control pid_control_t;

struct pid_control
{
    float       kp;
    float       ki;
    float       kd;
    float       minimum;
    float       maximum;
    float       anti_windup_value;
    float       target_point;
    rt_uint16_t sample_time;    // unit:ms
    float       p_error;
    float       i_error;
    float       d_error;
    float       integral;
    float       error;
    float       error_l;
    float       error_ll;
    rt_tick_t   last_time;
    float       out;
    float       last_out;
};

typedef struct
{
    float       kp;
    float       ki;
    float       kd;
}pid_kx_param_t;

rt_err_t        pid_init(pid_control_t* pid, float kp, float ki, float kd, float limit_max, float limit_min, rt_uint16_t sample_time);
rt_err_t        pid_reset(pid_control_t* pid);
rt_err_t        pid_set_kp(pid_control_t* pid, float kp);
rt_err_t        pid_set_ki(pid_control_t* pid, float ki);
rt_err_t        pid_set_kd(pid_control_t* pid, float kd);
rt_err_t        pid_set_kpid(pid_control_t* pid, float kp, float ki, float kd);
rt_err_t        pid_set_target(pid_control_t* pid, float set_point);
rt_err_t        pid_set_sample_time(pid_control_t* pid, rt_uint16_t sample_time);
rt_err_t        pid_set_anti_windup_value(pid_control_t* pid, float anti_windup_value);
float           pid_update_sample(pid_control_t* pid, float measure_value);
float           pid_update(pid_control_t* pid, float measure_value);

rt_err_t        send_waveform_fomate(void *buf, uint32_t size);


#endif
