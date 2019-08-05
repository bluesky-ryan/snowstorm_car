#include <stdio.h>
#include <stdlib.h>

#include "motor.h"
#include "pid.h"
#include "wheel.h"
#include "ps2.h"

#define MOTOR_TASK_TACK_SIZE    1048
#define MOTOR_TASK_PRIORITY     3
#define MOTOR_TASK_TICK         100


typedef enum
{
    CAR_CMD_INVALID,            /* 无效 */
    CAR_CMD_FORWARD,            /* 前进 */
    CAR_CMD_BACK,               /* 后退 */
    CAR_CMD_STOP,               /* 停止 */
    CAR_CMD_RIGHT,              /* 右平移 */
    CAR_CMD_LEFT,               /* 左平移 */
    CAR_CMD_TURN_RIGHT,         /* 右拐 */
    CAR_CMD_TURN_LEFT,          /* 左拐 */
    CAR_CMD_FORWARD_LEFT,       /* 前左 */
    CAR_CMD_FORWARD_RIGHT,      /* 前右 */
    CAR_CMD_BACK_LEFT,          /* 后左 */
    CAR_CMD_BACK_RIGHT,         /* 后右 */
}car_cmd;

typedef struct
{
    float       ch1_speed;
    float       ch2_speed;
    float       ch3_speed;
    float       ch4_speed;
}car_math_t;

/* PS2命令关联小车命令 */
typedef struct
{
    uint16_t      ps2_keys_value;
    car_cmd       cmd;
}car_ps2_cmd_t;

/* 小车命令关联数学控制变量 */
typedef struct
{
    car_cmd       cmd;
    car_math_t    math;
}car_cmd_math_t;


typedef struct
{
    rt_thread_t   h_thread;
    wheel_t       m_wheel[4];
    rt_tick_t     pid_last_time;
    rt_tick_t     vct_last_time;
    int32_t       pid_sample_time; //ms
    int32_t       vct_sample_time; //ms
    car_math_t    cur_math;
}car_ctl_t;

/* PS映射car cmd表格，组合键命令在头添加，单键命令放后面 */
car_ps2_cmd_t ps2_to_cmd_table[] = {
    {PS2_BTN_RIGHT| PS2_BTN_UP,     CAR_CMD_FORWARD_RIGHT},
    {PS2_BTN_LEFT | PS2_BTN_UP,     CAR_CMD_FORWARD_LEFT},
    {PS2_BTN_RIGHT| PS2_BTN_DOWN,   CAR_CMD_BACK_RIGHT},
    {PS2_BTN_LEFT | PS2_BTN_DOWN,   CAR_CMD_BACK_LEFT},
    {PS2_BTN_UP,                    CAR_CMD_FORWARD},
    {PS2_BTN_DOWN,                  CAR_CMD_BACK},
    {PS2_BTN_RIGHT,                 CAR_CMD_RIGHT},
    {PS2_BTN_LEFT,                  CAR_CMD_LEFT},
    {PS2_BTN_CICLE,                 CAR_CMD_TURN_RIGHT},
    {PS2_BTN_SQUARE,                CAR_CMD_TURN_LEFT},
};

/* 命令映射到几何控制参数 */
car_cmd_math_t cmd_to_math_table[] = {
    {CAR_CMD_INVALID,           {   0,     0,      0,      0}},
    {CAR_CMD_STOP,              {   0,     0,      0,      0}},
    {CAR_CMD_FORWARD_LEFT,      {   0,   120,    120,      0}},
    {CAR_CMD_FORWARD_RIGHT,     { 120,     0,      0,    120}},
    {CAR_CMD_BACK_LEFT,         {-120,     0,      0,   -120}},
    {CAR_CMD_BACK_RIGHT,        {   0,  -120,   -120,      0}}, 
    {CAR_CMD_FORWARD,           { 120,   120,    120,    120}},
    {CAR_CMD_BACK,              {-120,  -120,   -120,   -120}},
    {CAR_CMD_RIGHT,             { 120,  -120,   -120,    120}},
    {CAR_CMD_LEFT,              {-120,   120,    120,   -120}},
    {CAR_CMD_TURN_RIGHT,        { 120,  -120,    120,   -120}},
    {CAR_CMD_TURN_LEFT,         {-120,   120,   -120,    120}},
};

/**
 * @ingroup motor
 *
 * 获取当前PS2遥控值对应的命令
 *
 * @param  none
 * @retrun car_cmd 当前对应的CAR命令
 */
car_cmd car_ps2_cmd_get(void)
{
    ps2_ctrl_data_t ps2_data = ps2_get_current_ctrl();

    if (PS2_GREEN_MODE != ps2_get_mode())
        return CAR_CMD_INVALID;

    if (0xFF == ps2_data.button)
        return CAR_CMD_STOP;
    
    for (int i = 0; i < ARRAY_SIZE(ps2_to_cmd_table); i++)
    {
        if ((uint16_t)(~ps2_data.button) == ps2_to_cmd_table[i].ps2_keys_value)
            return ps2_to_cmd_table[i].cmd;
    }

    return CAR_CMD_INVALID;
}

/**
 * @ingroup motor
 *
 * 获取当前命令对应的几何参数
 *
 * @param  cmd      命令
 * @param  math     几何参数
 * @retrun rt_err_t RT_EOK成功，其他失败
 */
rt_err_t car_cmd_to_math(car_cmd cmd, car_math_t *math)
{
    if (RT_NULL == math)
        return -RT_EINVAL;

    for (int i = 0; i < ARRAY_SIZE(cmd_to_math_table); i++)
    {
        if (cmd_to_math_table[i].cmd == cmd)
        {
            *math = cmd_to_math_table[i].math;
            return RT_EOK;
        }
    }

    return -RT_EINVAL;
}


static car_ctl_t *g_car_ctl  = RT_NULL;


static void motor_task_entry(void *parameter)
{
    car_ctl_t   *p_car      = (car_ctl_t *)parameter;
    int16_t     pwm         = 0;
    /* 电机初始化 */
    motor_init();

    
    /* 初始化4个轮子 */
    wheel_init(&p_car->m_wheel[0], MOTOR_CH1);
    wheel_init(&p_car->m_wheel[1], MOTOR_CH2);
    wheel_init(&p_car->m_wheel[2], MOTOR_CH3);
    wheel_init(&p_car->m_wheel[3], MOTOR_CH4);

    /* 采样间隔 */
    p_car->pid_sample_time = 20;             /* PID刷新间隔 */
    p_car->vct_sample_time = 10;             /* 速度刷新间隔 */
    p_car->pid_last_time = rt_tick_get();
    p_car->vct_last_time = rt_tick_get();
    
    while(1)
    {
        /* 速度设置刷新 */
        car_cmd_to_math(car_ps2_cmd_get(), &p_car->cur_math);
        p_car->m_wheel[0].m_target_speed = p_car->cur_math.ch1_speed;
        p_car->m_wheel[1].m_target_speed = p_car->cur_math.ch2_speed;
        p_car->m_wheel[2].m_target_speed = p_car->cur_math.ch3_speed;
        p_car->m_wheel[3].m_target_speed = p_car->cur_math.ch4_speed;
        
        /* 速度刷新 */
        if((rt_tick_get() - p_car->vct_last_time) > rt_tick_from_millisecond(p_car->vct_sample_time))
        {
            /* 通道1速度刷新 */
            motor_encode_chx_get(p_car->m_wheel[0].m_motor_ch, &p_car->m_wheel[0].m_current_encode);
            p_car->m_wheel[0].m_current_speed = wheel_encode_to_rpm(p_car->m_wheel[0].m_current_encode, (uint32_t)p_car->vct_last_time, (uint32_t)rt_tick_get());

            /* 通道2速度刷新 */
            motor_encode_chx_get(p_car->m_wheel[1].m_motor_ch, &p_car->m_wheel[1].m_current_encode);
            p_car->m_wheel[1].m_current_speed = wheel_encode_to_rpm(p_car->m_wheel[1].m_current_encode, (uint32_t)p_car->vct_last_time, (uint32_t)rt_tick_get());

            /* 通道3速度刷新 */
            motor_encode_chx_get(p_car->m_wheel[2].m_motor_ch, &p_car->m_wheel[2].m_current_encode);
            p_car->m_wheel[2].m_current_speed = wheel_encode_to_rpm(p_car->m_wheel[2].m_current_encode, (uint32_t)p_car->vct_last_time, (uint32_t)rt_tick_get());

            /* 通道4速度刷新 */
            motor_encode_chx_get(p_car->m_wheel[3].m_motor_ch, &p_car->m_wheel[3].m_current_encode);
            p_car->m_wheel[3].m_current_speed = wheel_encode_to_rpm(p_car->m_wheel[3].m_current_encode, (uint32_t)p_car->vct_last_time, (uint32_t)rt_tick_get());
            
            p_car->vct_last_time = rt_tick_get();
        }
        
        /* PID刷新 */
        if((rt_tick_get() - p_car->pid_last_time) > rt_tick_from_millisecond(p_car->pid_sample_time))
        {
            /* 通道1 PID 刷新 */
            pid_set_target(&p_car->m_wheel[0].m_pid, p_car->m_wheel[0].m_target_speed);             /* 设置目标值 */
            wheel_select_pid_kx(&p_car->m_wheel[0]);                                                /* 根据速度设置PID参数 */
            pwm = (int16_t)pid_update(&p_car->m_wheel[0].m_pid, p_car->m_wheel[0].m_current_speed); /* 更新PID参数 */
            motor_pwm_set(p_car->m_wheel[0].m_motor_ch, pwm);                                       /* 执行控制量 */
            
            /* 通道2 PID 刷新 */
            pid_set_target(&p_car->m_wheel[1].m_pid, p_car->m_wheel[1].m_target_speed);
            wheel_select_pid_kx(&p_car->m_wheel[1]);
            pwm = (int16_t)pid_update(&p_car->m_wheel[1].m_pid, p_car->m_wheel[1].m_current_speed);
            motor_pwm_set(p_car->m_wheel[1].m_motor_ch, pwm);

            /* 通道3 PID 刷新 */
            pid_set_target(&p_car->m_wheel[2].m_pid, p_car->m_wheel[2].m_target_speed);
            wheel_select_pid_kx(&p_car->m_wheel[2]);
            pwm = (int16_t)pid_update(&p_car->m_wheel[2].m_pid, p_car->m_wheel[2].m_current_speed);
            motor_pwm_set(p_car->m_wheel[2].m_motor_ch, pwm);

            /* 通道4 PID 刷新 */
            pid_set_target(&p_car->m_wheel[3].m_pid, p_car->m_wheel[3].m_target_speed);
            wheel_select_pid_kx(&p_car->m_wheel[3]);
            pwm = (int16_t)pid_update(&p_car->m_wheel[3].m_pid, p_car->m_wheel[3].m_current_speed);
            motor_pwm_set(p_car->m_wheel[3].m_motor_ch, pwm);
            
            p_car->pid_last_time = rt_tick_get();
        }
        
        rt_thread_mdelay(10);
    }
}

static int motor_task_init(void)
{
    if (RT_NULL == g_car_ctl)
    {
        g_car_ctl = rt_malloc(sizeof(car_ctl_t));
        if (RT_NULL == g_car_ctl)
        {
            rt_kprintf("car control struct malloc failed!\r\n");
            return -RT_ENOMEM;
        }
        rt_memset(g_car_ctl, 0, sizeof(car_ctl_t));
    }
    
    g_car_ctl->h_thread = rt_thread_create("motor task",
                            motor_task_entry,
                            (void *)g_car_ctl,
                            MOTOR_TASK_TACK_SIZE, 
                            MOTOR_TASK_PRIORITY,
                            MOTOR_TASK_TICK);
    
    if (RT_NULL != g_car_ctl->h_thread)
    {
        rt_thread_startup(g_car_ctl->h_thread);
    }
    else
    {
        rt_kprintf("create motor task failed!\r\n");
        return -RT_ENOSYS;
    }

    return RT_EOK;
}


/* FINSH 调试函数 */
#ifdef RT_USING_FINSH
#include <finsh.h>

/* FINSH 调试命令 */
#ifdef FINSH_USING_MSH


#endif /* FINSH_USING_MSH */
#endif /* RT_USING_FINSH */

INIT_APP_EXPORT(motor_task_init);

