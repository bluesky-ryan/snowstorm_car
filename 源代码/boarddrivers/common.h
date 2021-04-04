#ifndef __COMMON_H__
#define __COMMON_H__
#include <stm32f1xx.h>

//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)      ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
    
/* 数据类型 */
typedef enum {
    ROS_DATA_GYRO   = 1,        /* 陀螺仪数据 */
    ROS_DATA_ACCEL  = 2,        /* 加速计数据 */
    ROS_DATA_MEGY   = 3,        /* 磁力计数据 */
    ROS_DATA_EULER  = 4,        /* 欧拉角数据 */
    ROS_DATA_VEL    = 5,        /* 发布直线速度 */
    ROS_CMD_VEL     = 100,      /* 处理上位机下发命令 */
}ros_msg_type;

/* 传感器数据 */
typedef struct {
    float       x;
    float       y;
    float       z;
}sensor_msg;


/* 小车命令关联数学控制变量 */
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

