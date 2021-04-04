/*==================================================
 *                   通讯协议                       |
 *===================================================
 *|  0xAA  | 0xBC  |  type | length | datas |  CRC  |
 *===================================================
 *|  1byte | 1byte | 1byte | 1byte  | ...   | 1byte |
 *===================================================
*/


#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include "common.h"


//#define DRV_DEBUG
#define LOG_TAG                "uart4"
#include <drv_log.h>

#define SAMPLE_UART_NAME       "uart4"  /* 串口设备名称 */


static rt_device_t serial = NULL;                           /* 串口设备句柄 */
static struct rt_semaphore rx_sem;                          /* 用于接收消息的信号量 */
static rt_mutex_t   tx_mutex = NULL;                        /* 用于接收消息的信号量 */
struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */



int send_pack(ros_msg_type type, void *msg, rt_uint32_t len)
{
    unsigned char start[4] = {0XAA, 0XBC};
    unsigned char crc = 0;
    int  ret = 0;
    unsigned char *ptr = msg;
    
    if (serial == NULL || tx_mutex == NULL)
        return -RT_EIO;

    start[2] = type;
    start[3] = (rt_uint8_t)len;
    for (int i = 0; i < len; i++)
    {
        crc ^= ptr[i];
    }
    
    rt_mutex_take(tx_mutex, RT_WAITING_FOREVER);
    ret = rt_device_write(serial, 0, start, 4);
    ret += rt_device_write(serial, 0, ptr, len);
    ret += rt_device_write(serial, 0, &crc, 1);
    rt_mutex_release(tx_mutex);
    return ret;
}

extern car_ros_cmd_t g_ros_cmd_msg;
void parse_callback(ros_msg_type type, void *msg)
{
    sensor_msg tmp;
    
    switch(type)
    {
    case ROS_CMD_VEL:
        rt_enter_critical();
        g_ros_cmd_msg.vel.x = ((sensor_msg *)msg)->x;
        g_ros_cmd_msg.vel.y = ((sensor_msg *)msg)->y;
        g_ros_cmd_msg.vel.z = ((sensor_msg *)msg)->z;
        g_ros_cmd_msg.cur_time = rt_tick_get();
        g_ros_cmd_msg.enable = RT_TRUE;
        rt_exit_critical();
        LOG_I("Received ROS_CMD_VEL x:%f y:%f rz:%f", g_ros_cmd_msg.vel.x, g_ros_cmd_msg.vel.y, g_ros_cmd_msg.vel.z);
        break;
    default:
        break;
    }
}

/* 获取一个字节数据 */
static unsigned char uart_sample_get_char(void)
{
    unsigned char ch;

    while (rt_device_read(serial, 0, &ch, 1) == 0)
    {
        rt_sem_control(&rx_sem, RT_IPC_CMD_RESET, RT_NULL);
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
    }
    return ch;
}

/* 数据解析线程 */
static void serial_rx_parsing(void *ptr)
{
    unsigned char ch;
    unsigned char data[300];
    unsigned char step = 0;
    unsigned char i    = 0;
    unsigned char crc  = 0;

    while (1)
    {
        ch = uart_sample_get_char();
        if(step == 0 && 0xaa == ch)     /* 协议头 */
        {
            step++;
            continue;
        }
        else if(step == 1 && 0xbc == ch)/* 协议头 */
        {
            step++;
            continue;
        }
        else if(step == 2)              /* 数据类型 */
        {
            step++;
            data[i++] = ch;
            continue;
        }
        else if(step == 3)              /* 数据长度 */
        {
            step++;
            data[i++] = ch;
            continue;
        }
        else if(step == 4 && i-2 < data[1])/* 具体数据 */
        {
            data[i++] = ch;
            continue;
        }
        else if(step == 4 && i-2 == data[1])/* 数据校验 */
        {
            data[i] = ch;
            for (int j = 0; j < data[1]; j++)
            {
                crc ^= data[j+2];
            }
            /* 数据校验成功 */
            if (crc == data[i])
            {
                //LOG_I("ros base serial rx crc ok!\r\n");
                parse_callback((ros_msg_type)data[0], (void *)&data[2]);
            }
            else
            {
                LOG_I("ros base serial rx crc error!\r\n");
            }
            crc = 0;
            step=0;
            i=0;
        }
        else {
            crc = 0;
            step=0;
            i=0;
        }
        //rt_thread_mdelay(50);
    }
}

/* 接收数据回调函数 */
static rt_err_t uart_input_event(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&rx_sem);

    return RT_EOK;
}


int ros_base_ctl_init(void)
{
    /* step1：查找串口设备 */
    serial = rt_device_find(SAMPLE_UART_NAME);
    if (NULL == serial)
    {
        LOG_E("can't find ros base urat!");
        return -RT_ENOSYS;
    }

    /* step2：修改串口配置参数 */
    config.bufsz     = 400;                   //修改缓冲区 buff size 为 128
    
    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    /* step4：打开串口设备。以中断接收发送 */
    if (RT_EOK != rt_device_open(serial, RT_DEVICE_FLAG_INT_RX|RT_DEVICE_FLAG_INT_TX))
    {
        LOG_E("open ros base urat failed!");
        return -RT_ENOSYS;
    }

    /* 初始化信号量 */
    rt_sem_init(&rx_sem, "ros_rx_sem", 0, RT_IPC_FLAG_FIFO);
    /* 以中断接收及轮询发送模式打开串口设备 */
    rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_input_event);
    /* 创建发送字符互斥量 */
    tx_mutex = rt_mutex_create("ros_tx_mutex", RT_IPC_FLAG_FIFO);

    /* 创建 serial 线程 */
    rt_thread_t thread = rt_thread_create("ros base process", serial_rx_parsing, RT_NULL, 1024, 25, 10);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        return -RT_ERROR;
    }

    /* 发送字符串 */
    //const char *test = "hello world!";
    //int sends = rt_device_write(serial, 0, test, strlen(test));
    //rt_kprintf("send numbers:%d", sends);
    //sensor_msg msg = {1.2, 2.2, 3.3};
    //send_pack(ROS_DATA_GYRO, &msg, sizeof(msg));
    
    return 0;
}


INIT_ENV_EXPORT(ros_base_ctl_init);

