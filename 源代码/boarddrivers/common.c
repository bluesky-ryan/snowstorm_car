#include <board.h>
#include "common.h"

rt_size_t write_to_console(void *buf, uint32_t size)
{
    rt_device_t console = rt_console_get_device();
    return rt_device_write(console, -1, buf, size);
}


rt_err_t send_waveform_fomate(void *buf, uint32_t size)
{
    const char start[2] = {0x03, 0xfc};
    const char end[2]   = {0xfc, 0x03};
    
    write_to_console((void*)start, 2);
    write_to_console((void*)buf, size);
    write_to_console((void*)end, 2);

    return RT_EOK;
}

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, rt_int32_t alt, rt_uint8_t fly_model, rt_uint8_t armed)
{
    rt_uint8_t  _cnt=0;
    rt_uint8_t  sum = 0;
    rt_int16_t  _temp;
    rt_int32_t  _temp2 = alt;
    rt_uint8_t  data_to_send[50];    //·¢ËÍÊý¾Ý»º´æ
    
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x01;
    data_to_send[_cnt++]=0;
    
    _temp = (int)(angle_rol*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(angle_pit*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(angle_yaw*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    
    data_to_send[_cnt++]=BYTE3(_temp2);
    data_to_send[_cnt++]=BYTE2(_temp2);
    data_to_send[_cnt++]=BYTE1(_temp2);
    data_to_send[_cnt++]=BYTE0(_temp2);
    
    data_to_send[_cnt++] = fly_model;
    
    data_to_send[_cnt++] = armed;
    
    data_to_send[3] = _cnt-4;
    
    
    for(rt_uint8_t i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++]=sum;
    
    write_to_console((void*)data_to_send, _cnt);
}


