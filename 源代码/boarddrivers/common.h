#ifndef __COMMON_H__
#define __COMMON_H__
#include <stm32f1xx.h>

//���ݲ�ֺ궨�壬�ڷ��ʹ���1�ֽڵ���������ʱ������int16��float�ȣ���Ҫ�����ݲ�ֳɵ����ֽڽ��з���
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)      ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
    
/* �������� */
typedef enum {
    ROS_DATA_GYRO   = 1,        /* ���������� */
    ROS_DATA_ACCEL  = 2,        /* ���ټ����� */
    ROS_DATA_MEGY   = 3,        /* ���������� */
    ROS_DATA_EULER  = 4,        /* ŷ�������� */
    ROS_DATA_VEL    = 5,        /* ����ֱ���ٶ� */
    ROS_CMD_VEL     = 100,      /* ������λ���·����� */
}ros_msg_type;

/* ���������� */
typedef struct {
    float       x;
    float       y;
    float       z;
}sensor_msg;


/* С�����������ѧ���Ʊ��� */
typedef struct
{
    sensor_msg     vel;
    rt_uint32_t    cur_time;
    rt_base_t      enable;
}car_ros_cmd_t;

rt_err_t send_waveform_fomate(void *buf, uint32_t size);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, rt_int32_t alt, rt_uint8_t fly_model, rt_uint8_t armed);
int send_pack(ros_msg_type type, void *msg, rt_uint32_t len);

#endif

