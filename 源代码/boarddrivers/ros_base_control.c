/*==================================================
 *                   ͨѶЭ��                       |
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

#define SAMPLE_UART_NAME       "uart4"  /* �����豸���� */


static rt_device_t serial = NULL;                           /* �����豸��� */
static struct rt_semaphore rx_sem;                          /* ���ڽ�����Ϣ���ź��� */
static rt_mutex_t   tx_mutex = NULL;                        /* ���ڽ�����Ϣ���ź��� */
struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* ��ʼ�����ò��� */



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

/* ��ȡһ���ֽ����� */
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

/* ���ݽ����߳� */
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
        if(step == 0 && 0xaa == ch)     /* Э��ͷ */
        {
            step++;
            continue;
        }
        else if(step == 1 && 0xbc == ch)/* Э��ͷ */
        {
            step++;
            continue;
        }
        else if(step == 2)              /* �������� */
        {
            step++;
            data[i++] = ch;
            continue;
        }
        else if(step == 3)              /* ���ݳ��� */
        {
            step++;
            data[i++] = ch;
            continue;
        }
        else if(step == 4 && i-2 < data[1])/* �������� */
        {
            data[i++] = ch;
            continue;
        }
        else if(step == 4 && i-2 == data[1])/* ����У�� */
        {
            data[i] = ch;
            for (int j = 0; j < data[1]; j++)
            {
                crc ^= data[j+2];
            }
            /* ����У��ɹ� */
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

/* �������ݻص����� */
static rt_err_t uart_input_event(rt_device_t dev, rt_size_t size)
{
    /* ���ڽ��յ����ݺ�����жϣ����ô˻ص�������Ȼ���ͽ����ź��� */
    rt_sem_release(&rx_sem);

    return RT_EOK;
}


int ros_base_ctl_init(void)
{
    /* step1�����Ҵ����豸 */
    serial = rt_device_find(SAMPLE_UART_NAME);
    if (NULL == serial)
    {
        LOG_E("can't find ros base urat!");
        return -RT_ENOSYS;
    }

    /* step2���޸Ĵ������ò��� */
    config.bufsz     = 400;                   //�޸Ļ����� buff size Ϊ 128
    
    /* step3�����ƴ����豸��ͨ�����ƽӿڴ�����������֣�����Ʋ��� */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    /* step4���򿪴����豸�����жϽ��շ��� */
    if (RT_EOK != rt_device_open(serial, RT_DEVICE_FLAG_INT_RX|RT_DEVICE_FLAG_INT_TX))
    {
        LOG_E("open ros base urat failed!");
        return -RT_ENOSYS;
    }

    /* ��ʼ���ź��� */
    rt_sem_init(&rx_sem, "ros_rx_sem", 0, RT_IPC_FLAG_FIFO);
    /* ���жϽ��ռ���ѯ����ģʽ�򿪴����豸 */
    rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);
    /* ���ý��ջص����� */
    rt_device_set_rx_indicate(serial, uart_input_event);
    /* ���������ַ������� */
    tx_mutex = rt_mutex_create("ros_tx_mutex", RT_IPC_FLAG_FIFO);

    /* ���� serial �߳� */
    rt_thread_t thread = rt_thread_create("ros base process", serial_rx_parsing, RT_NULL, 1024, 25, 10);
    /* �����ɹ��������߳� */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        return -RT_ERROR;
    }

    /* �����ַ��� */
    //const char *test = "hello world!";
    //int sends = rt_device_write(serial, 0, test, strlen(test));
    //rt_kprintf("send numbers:%d", sends);
    //sensor_msg msg = {1.2, 2.2, 3.3};
    //send_pack(ROS_DATA_GYRO, &msg, sizeof(msg));
    
    return 0;
}


INIT_ENV_EXPORT(ros_base_ctl_init);

