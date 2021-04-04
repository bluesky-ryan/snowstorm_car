/*==========================================================================
* ���ֱ�źͷ���
*
*        ||      x       ||
*      1 ||      ^       || 2
*        ||      |       ||
*                |
*                |----->y
*
*        ||              ||
*      3 ||              || 4
*        ||              ||
*        
*===========================================================================
*�ٶȲ�����ʽ:
*   Vx = (V1+V2+V2+V4)/4
*   Vy = (V1+V4-V2-V3)/4
*   W  = (V2+V4-V1-V3)/(4(a+b))
*   ע: a=L13/2 b=L12/2 
*===========================================================================
*�˶�ѧ����
*   V1 = Vx+Vy-W(a+b)
*   V2 = Vx-Vy+W(a+b)
*   V3 = Vx-Vy-W(a+b)
*   V4 = Vx+Vy+W(a+b)
============================================================================*/


#include <stdio.h>
#include <stdlib.h>

#include "motor.h"
#include "pid.h"
#include "wheel.h"
#include "ps2.h"
#include "common.h"

#define MOTOR_TASK_TACK_SIZE    1048
#define MOTOR_TASK_PRIORITY     3
#define MOTOR_TASK_TICK         100
#define MOTOR_SPEEK_RPM         150

typedef enum
{
    CAR_CMD_INVALID,            /* ��Ч */
    CAR_CMD_FORWARD,            /* ǰ�� */
    CAR_CMD_BACK,               /* ���� */
    CAR_CMD_STOP,               /* ֹͣ */
    CAR_CMD_RIGHT,              /* ��ƽ�� */
    CAR_CMD_LEFT,               /* ��ƽ�� */
    CAR_CMD_TURN_RIGHT,         /* �ҹ� */
    CAR_CMD_TURN_LEFT,          /* ��� */
    CAR_CMD_FORWARD_LEFT,       /* ǰ�� */
    CAR_CMD_FORWARD_RIGHT,      /* ǰ�� */
    CAR_CMD_BACK_LEFT,          /* ���� */
    CAR_CMD_BACK_RIGHT,         /* ���� */
}car_cmd;

typedef struct
{
    float       ch1_speed;
    float       ch2_speed;
    float       ch3_speed;
    float       ch4_speed;
}car_math_t;

/* PS2�������С������ */
typedef struct
{
    uint16_t      ps2_keys_value;
    car_cmd       cmd;
}car_ps2_cmd_t;

/* С�����������ѧ���Ʊ��� */
typedef struct
{
    car_cmd       cmd;
    car_math_t    math;
}car_cmd_math_t;


car_ros_cmd_t g_ros_cmd_msg = {
    .enable     = RT_FALSE,
    .cur_time   = 0,
    .vel        = {0,0,0},
};


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

/* PSӳ��car cmd�����ϼ�������ͷ��ӣ���������ź��� */
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

/* ����ӳ�䵽���ο��Ʋ��� */
car_cmd_math_t cmd_to_math_table[] = {
    {CAR_CMD_INVALID,       {               0,                0,                0,                0}},
    {CAR_CMD_STOP,          {               0,                0,                0,                0}},
    {CAR_CMD_FORWARD_LEFT,  {               0,  MOTOR_SPEEK_RPM,  MOTOR_SPEEK_RPM,                0}},
    {CAR_CMD_FORWARD_RIGHT, { MOTOR_SPEEK_RPM,                0,                0,  MOTOR_SPEEK_RPM}},
    {CAR_CMD_BACK_LEFT,     {-MOTOR_SPEEK_RPM,                0,                0, -MOTOR_SPEEK_RPM}},
    {CAR_CMD_BACK_RIGHT,    {               0, -MOTOR_SPEEK_RPM, -MOTOR_SPEEK_RPM,                0}}, 
    {CAR_CMD_FORWARD,       { MOTOR_SPEEK_RPM,  MOTOR_SPEEK_RPM,  MOTOR_SPEEK_RPM,  MOTOR_SPEEK_RPM}},
    {CAR_CMD_BACK,          {-MOTOR_SPEEK_RPM, -MOTOR_SPEEK_RPM, -MOTOR_SPEEK_RPM, -MOTOR_SPEEK_RPM}},
    {CAR_CMD_RIGHT,         { MOTOR_SPEEK_RPM, -MOTOR_SPEEK_RPM, -MOTOR_SPEEK_RPM,  MOTOR_SPEEK_RPM}},
    {CAR_CMD_LEFT,          {-MOTOR_SPEEK_RPM,  MOTOR_SPEEK_RPM,  MOTOR_SPEEK_RPM, -MOTOR_SPEEK_RPM}},
    {CAR_CMD_TURN_RIGHT,    { MOTOR_SPEEK_RPM, -MOTOR_SPEEK_RPM,  MOTOR_SPEEK_RPM, -MOTOR_SPEEK_RPM}},
    {CAR_CMD_TURN_LEFT,     {-MOTOR_SPEEK_RPM,  MOTOR_SPEEK_RPM, -MOTOR_SPEEK_RPM,  MOTOR_SPEEK_RPM}},
};

/**
 * @ingroup motor
 *
 * ��ȡ��ǰPS2ң��ֵ��Ӧ������
 *
 * @param  none
 * @retrun car_cmd ��ǰ��Ӧ��CAR����
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
 * ��ȡ��ǰ�����Ӧ�ļ��β���
 *
 * @param  cmd      ����
 * @param  math     ���β���
 * @retrun rt_err_t RT_EOK�ɹ�������ʧ��
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
    double      waveman[3]  = {0};
    car_cmd     cur_cmd;
    sensor_msg  msg;
    
    /* �����ʼ�� */
    motor_init();

    
    /* ��ʼ��4������ */
    wheel_init(&p_car->m_wheel[0], MOTOR_CH1);
    wheel_init(&p_car->m_wheel[1], MOTOR_CH2);
    wheel_init(&p_car->m_wheel[2], MOTOR_CH3);
    wheel_init(&p_car->m_wheel[3], MOTOR_CH4);

    /* ������� */
    p_car->pid_sample_time = 20;             /* PIDˢ�¼�� */
    p_car->vct_sample_time = 10;             /* �ٶ�ˢ�¼�� */
    p_car->pid_last_time = rt_tick_get();
    p_car->vct_last_time = rt_tick_get();
    
    while(1)
    {
        /* �ٶ�����ˢ�� */
        cur_cmd = car_ps2_cmd_get();
        car_cmd_to_math(cur_cmd, &p_car->cur_math);
        p_car->m_wheel[0].m_target_speed = p_car->cur_math.ch1_speed;
        p_car->m_wheel[1].m_target_speed = p_car->cur_math.ch2_speed;
        p_car->m_wheel[2].m_target_speed = p_car->cur_math.ch3_speed;
        p_car->m_wheel[3].m_target_speed = p_car->cur_math.ch4_speed;

        /*
        *����ROS�����ٶ� 
        *V1 = Vx+Vy-W(a+b) ==> rmp = (Vx+Vy-W(a+b)) * 60 /(2 * pi * r)
        *V2 = Vx-Vy+W(a+b) ==> rmp = (Vx-Vy+W(a+b)) * 60 /(2 * pi * r)
        *V3 = Vx-Vy-W(a+b) ==> rmp = (Vx-Vy-W(a+b)) * 60 /(2 * pi * r)
        *V4 = Vx+Vy+W(a+b) ==> rmp = (Vx+Vy+W(a+b)) * 60 /(2 * pi * r)
        */
        if (g_ros_cmd_msg.enable == RT_TRUE)
        {
            p_car->m_wheel[0].m_target_speed = ((g_ros_cmd_msg.vel.x+g_ros_cmd_msg.vel.y-g_ros_cmd_msg.vel.z*0.17)/0.0031415926);
            p_car->m_wheel[1].m_target_speed = ((g_ros_cmd_msg.vel.x-g_ros_cmd_msg.vel.y+g_ros_cmd_msg.vel.z*0.17)/0.0031415926);
            p_car->m_wheel[2].m_target_speed = ((g_ros_cmd_msg.vel.x-g_ros_cmd_msg.vel.y-g_ros_cmd_msg.vel.z*0.17)/0.0031415926);
            p_car->m_wheel[3].m_target_speed = ((g_ros_cmd_msg.vel.x+g_ros_cmd_msg.vel.y+g_ros_cmd_msg.vel.z*0.17)/0.0031415926);
            if((rt_tick_get() - g_ros_cmd_msg.cur_time) > rt_tick_from_millisecond(300)) {
                p_car->m_wheel[0].m_target_speed = 0;
                p_car->m_wheel[1].m_target_speed = 0;
                p_car->m_wheel[2].m_target_speed = 0;
                p_car->m_wheel[3].m_target_speed = 0;
                g_ros_cmd_msg.cur_time = 0;
                g_ros_cmd_msg.enable = RT_FALSE;
            }
        }
          
        
        /* �ٶ�ˢ�� */
        if((rt_tick_get() - p_car->vct_last_time) > rt_tick_from_millisecond(p_car->vct_sample_time))
        {
            /* ͨ��1�ٶ�ˢ�� */
            motor_encode_chx_get(p_car->m_wheel[0].m_motor_ch, &p_car->m_wheel[0].m_current_encode);
            p_car->m_wheel[0].m_current_speed = wheel_encode_to_rpm(p_car->m_wheel[0].m_current_encode, (uint32_t)p_car->vct_last_time, (uint32_t)rt_tick_get());
            
            /* ͨ��2�ٶ�ˢ�� */
            motor_encode_chx_get(p_car->m_wheel[1].m_motor_ch, &p_car->m_wheel[1].m_current_encode);
            p_car->m_wheel[1].m_current_speed = wheel_encode_to_rpm(p_car->m_wheel[1].m_current_encode, (uint32_t)p_car->vct_last_time, (uint32_t)rt_tick_get());

            /* ͨ��3�ٶ�ˢ�� */
            motor_encode_chx_get(p_car->m_wheel[2].m_motor_ch, &p_car->m_wheel[2].m_current_encode);
            p_car->m_wheel[2].m_current_speed = wheel_encode_to_rpm(p_car->m_wheel[2].m_current_encode, (uint32_t)p_car->vct_last_time, (uint32_t)rt_tick_get());

            /* ͨ��4�ٶ�ˢ�� */
            motor_encode_chx_get(p_car->m_wheel[3].m_motor_ch, &p_car->m_wheel[3].m_current_encode);
            p_car->m_wheel[3].m_current_speed = wheel_encode_to_rpm(p_car->m_wheel[3].m_current_encode, (uint32_t)p_car->vct_last_time, (uint32_t)rt_tick_get());
            
            p_car->vct_last_time = rt_tick_get();
        }
        
        /* PIDˢ�� */
        if((rt_tick_get() - p_car->pid_last_time) > rt_tick_from_millisecond(p_car->pid_sample_time))
        {
            /* ͨ��1 PID ˢ�� */
            pid_set_target(&p_car->m_wheel[0].m_pid, p_car->m_wheel[0].m_target_speed);             /* ����Ŀ��ֵ */
            wheel_select_pid_kx(&p_car->m_wheel[0]);                                                /* �����ٶ�����PID���� */
            pwm = (int16_t)pid_update(&p_car->m_wheel[0].m_pid, p_car->m_wheel[0].m_current_speed); /* ����PID���� */
            motor_pwm_set(p_car->m_wheel[0].m_motor_ch, pwm);                                       /* ִ�п����� */
            
            /* ͨ��2 PID ˢ�� */
            pid_set_target(&p_car->m_wheel[1].m_pid, p_car->m_wheel[1].m_target_speed);
            wheel_select_pid_kx(&p_car->m_wheel[1]);
            pwm = (int16_t)pid_update(&p_car->m_wheel[1].m_pid, p_car->m_wheel[1].m_current_speed);
            motor_pwm_set(p_car->m_wheel[1].m_motor_ch, pwm);

            /* ͨ��3 PID ˢ�� */
            pid_set_target(&p_car->m_wheel[2].m_pid, p_car->m_wheel[2].m_target_speed);
            wheel_select_pid_kx(&p_car->m_wheel[2]);
            pwm = (int16_t)pid_update(&p_car->m_wheel[2].m_pid, p_car->m_wheel[2].m_current_speed);
            motor_pwm_set(p_car->m_wheel[2].m_motor_ch, pwm);

            /* ͨ��4 PID ˢ�� */
            pid_set_target(&p_car->m_wheel[3].m_pid, p_car->m_wheel[3].m_target_speed);
            wheel_select_pid_kx(&p_car->m_wheel[3]);
            pwm = (int16_t)pid_update(&p_car->m_wheel[3].m_pid, p_car->m_wheel[3].m_current_speed);
            motor_pwm_set(p_car->m_wheel[3].m_motor_ch, pwm);
            
            p_car->pid_last_time = rt_tick_get();
        }

        /* ����ROS�ٶ���Ϣ */
        msg.x = (p_car->m_wheel[0].m_current_speed +p_car->m_wheel[1].m_current_speed+
                 p_car->m_wheel[2].m_current_speed+p_car->m_wheel[3].m_current_speed)*0.0031415926/4;
        msg.y = (p_car->m_wheel[0].m_current_speed +p_car->m_wheel[3].m_current_speed-
                 p_car->m_wheel[1].m_current_speed-p_car->m_wheel[2].m_current_speed)*0.0031415926/4;
        msg.z = 0;
        send_pack(ROS_DATA_VEL, &msg, sizeof(sensor_msg));
        
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


/* FINSH ���Ժ��� */
#ifdef RT_USING_FINSH
#include <finsh.h>

/* FINSH �������� */
#ifdef FINSH_USING_MSH


#endif /* FINSH_USING_MSH */
#endif /* RT_USING_FINSH */

INIT_APP_EXPORT(motor_task_init);

