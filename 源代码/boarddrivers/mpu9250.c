#include "mpu9250.h"
#include <string.h>

//#define DRV_DEBUG
#define LOG_TAG             "mpu"
#include <drv_log.h>

typedef enum {
    GYRO_FS_250DPS      =  0X00,
    GYRO_FS_500DPS      =  0X08,
    GYRO_FS_1000DPS     =  0X10,
    GYRO_FS_2000DPS     =  0X18,
}GYRO_FS;

typedef enum {
    ACCEL_FS_2G         =  0X00,
    ACCEL_FS_4G         =  0X08,
    ACCEL_FS_8G         =  0X10,
    ACCEL_FS_16G        =  0X18,
}ACCEL_FS;

/* 加速计量程配置 */
#define ACCEL_FS_CFG  ACCEL_FS_8G
#define GYRO_FS_CFG   GYRO_FS_1000DPS

/*
*  磁力计校准参数，实测得来
*  比例因子: x:0.952974  y:1.078247 z: 0.977305
*  偏差    : x:39.768749 y:6.924024 z:-2.216602
*/
#define MEGY_BIAS_X    39.768749
#define MEGY_BIAS_Y     6.924024
#define MEGY_BIAS_Z    -2.216602
#define MEGY_FACTOR_X   0.952974
#define MEGY_FACTOR_Y   1.078247
#define MEGY_FACTOR_Z   0.977305


const mpu_reg_data_t mpu_regs_init[] = {
    { MPU6500_PWR_MGMT_1,       0x80 },     /* Reset Device */ 
    { MPU6500_PWR_MGMT_1,       0x03 },     /* Clock Source - Gyro-Z */ 
    { MPU6500_PWR_MGMT_1,       0x00 },     /* Enable Device */ 
    { MPU6500_PWR_MGMT_2,       0x00 },     /* Enable Acc & Gyro */ 
    { MPU6500_CONFIG,           0x04 },     /* LPF 41Hz 低通滤波 */ 
    { MPU6500_GYRO_CONFIG,      GYRO_FS_CFG  },     /* +-2000dps */ 
    { MPU6500_ACCEL_CONFIG,     ACCEL_FS_CFG },     /* +-8G */ 
    { MPU6500_ACCEL_CONFIG_2,   0x02 },     /* enable LowPassFilter  Set Acc LPF */ 
//    { MPU6500_USER_CTRL,        0x20 },     /* Enable AUX MPU I2C主控模式 */
    { MPU6500_INT_PIN_CFG,      0X02 },     /* 启用Pass through mode,I2C直连AK8963 */
};


/* MPU 驱动数据 */
typedef struct {
    rt_bool_t                   initialized;   /* 传感器初始化状态 */
    uint8_t                     mpu_addr;      /* 设备I2C地址 */
    uint8_t                     meg_addr;      /* 磁力计I2C从地址 */
    float                       accel_fs_unit; /* 量程单位1G=nFS */
    float                       gyro_fs_unit;  /* 量程单位1°=nFS */
    mpu_sensor_value_t          gyro_bias;     /* 陀螺仪偏差 */
    mpu_sensor_value_t          accel_bias;    /* 加速计偏差 */
    mpu_sensor_value_t          megy_asa;      /* 磁力计灵敏度 */
    mpu_sensor_value_t          megy_bias;     /* 磁力计计偏差 */
    mpu_sensor_value_t          megy_factor;   /* 磁力计比例因子 */
    struct rt_i2c_bus_device    *i2c_bus;      /* I2C总线设备句柄 */
}mpu_handle_t;

mpu_handle_t *mpu = RT_NULL;

rt_err_t mpu_gyro_accel_calibrate(mpu_sensor_value_t *gyro_bias, mpu_sensor_value_t *accel_bias);
rt_err_t mpu_megy_calibrate(mpu_sensor_value_t *megy_bias);


/* 写传感器寄存器 */
static rt_err_t mpu_write_regs(const rt_uint8_t i2c_addr, const rt_uint8_t reg, void *buf, rt_uint16_t len)
{
    struct rt_i2c_msg msgs  = {0};
    rt_uint8_t        *tbuf = RT_NULL;

    if (RT_NULL == mpu || RT_FALSE == mpu->initialized)
        return -RT_ENOSYS;
    
    if (RT_NULL == (tbuf = rt_calloc(1, len+1))) {
        return -RT_EOK;
    }
    tbuf[0] = reg;
    rt_memcpy(tbuf+1, buf, len);
    
    msgs.addr = i2c_addr;
    msgs.flags = RT_I2C_WR;
    msgs.buf = tbuf;
    msgs.len = len+1;

    /* 调用I2C设备接口传输数据 */
    if (rt_i2c_transfer(mpu->i2c_bus, &msgs, 1) == 1)
    {
        rt_free(tbuf);
        return RT_EOK;
    }
    else
    {
        rt_free(tbuf);
        LOG_I("mpu write reg%s failed!", (len > 1 ? "s" : ""));
        return -RT_ERROR;
    }
}

/* 读传感器寄存器数据 */
static rt_err_t mpu_read_regs(const rt_uint8_t i2c_addr, const rt_uint8_t reg, void *buf, rt_uint16_t len)
{
    struct rt_i2c_msg   msgs[2] = {0};
    rt_uint8_t tmp = reg;

    if (RT_NULL == mpu || RT_FALSE == mpu->initialized)
        return -RT_ENOSYS;

    msgs[0].addr = i2c_addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = &tmp;
    msgs[0].len = 1;
    
    msgs[1].addr = i2c_addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = buf;
    msgs[1].len = len;

    /* 调用I2C设备接口传输数据 */
    if (rt_i2c_transfer(mpu->i2c_bus, msgs, 2) == 2)
    {
        return RT_EOK;
    }
    else
    {
        LOG_I("mpu read reg%s failed!", len > 1 ? "s" : "");
        return -RT_ERROR;
    }
}

/* 获取量程单位值: 1G = FS */
float mpu_get_accel_fs_unit(ACCEL_FS fs_cfg)
{
    float value = 0;
    
    if (fs_cfg == ACCEL_FS_2G)
        value = 16384.0f;
    else if (fs_cfg == ACCEL_FS_4G)
        value = 8129.0f;
    else if (fs_cfg == ACCEL_FS_8G)
        value = 4096.0f;
    else if (fs_cfg == ACCEL_FS_16G)
        value = 2048.0f;
    else
        value = 0;

    return value;
}

/* 获取量程单位值: 1° = FS */
float mpu_get_gyro_fs_unit(GYRO_FS fs_cfg)
{
    float value = 0;
    
    if (fs_cfg == GYRO_FS_250DPS)
        value = 131.072f;
    else if (fs_cfg == GYRO_FS_500DPS)
        value = 65.536f;
    else if (fs_cfg == GYRO_FS_1000DPS)
        value = 32.368f;
    else if (fs_cfg == GYRO_FS_2000DPS)
        value = 16.384f;
    else
        value = 0;

    return value;
}


rt_err_t mpu_init(mpu_device_t *device)
{
    rt_int8_t   value = 0;
    
    /* 检测设备是否已经处在 */
    if (RT_NULL == mpu)
    {
        mpu = rt_calloc(1, sizeof(mpu_handle_t));
        if (RT_NULL == mpu)
        {
            LOG_E("can't malloc mpu memory!");
            return -RT_ENOSYS;
        }
    }
    else
    {
        LOG_E("mpu already initialized, can't again!");
        return -RT_EBUSY;
    }
    
    /* 查找I2C总线设备，获取I2C总线设备句柄 */
    mpu->i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(device->i2c_bus_name);
    if (mpu->i2c_bus == RT_NULL)
    {
        LOG_E("can't find %s device!", device->i2c_bus_name);
        goto error;
    }
    else
    {
        mpu->mpu_addr = device->mpu_addr;
        mpu->meg_addr = device->meg_addr;
        mpu->initialized = RT_TRUE;
        mpu->gyro_fs_unit = mpu_get_gyro_fs_unit(GYRO_FS_CFG);
        mpu->accel_fs_unit = mpu_get_accel_fs_unit(ACCEL_FS_CFG);
        
        /* 初始化原始寄存器 */
        for (int i = 0; i < ARRAY_SIZE(mpu_regs_init); i++)
        {
            if(RT_EOK != mpu_write_regs(device->mpu_addr, mpu_regs_init[i].reg, (void *)&mpu_regs_init[i].data, 1))
            {
                mpu->initialized = RT_FALSE;
                LOG_E("write initialized register failed!");
                goto error;
            }
        }
        
        /* 获取MPU ID */
        if (RT_EOK != mpu_read_regs(device->mpu_addr, MPU6500_WHO_AM_I, &value, 1))
        {
            mpu->initialized = RT_FALSE;
            goto error;
        }
        LOG_I("MPU ID: 0X%X", value);

        
        /* 获取MEGR ID */
        if (RT_EOK != mpu_read_regs(device->meg_addr, AK8963_WIA, &value, 1))
        {
            mpu->initialized = RT_FALSE;
            goto error;
        }
        LOG_I("MEGY ID: 0X%X", value);

        /* 校准 */
        if (RT_EOK != mpu_gyro_accel_calibrate(&mpu->gyro_bias, &mpu->accel_bias))
        {
            mpu->initialized = RT_FALSE;
            goto error;
        }
        LOG_I("GYRO BISA : x:%f y:%f z:%f", mpu->gyro_bias.x, mpu->gyro_bias.y, mpu->gyro_bias.z);
        LOG_I("ACCEL BISA: x:%f y:%f z:%f", mpu->accel_bias.x, mpu->accel_bias.y, mpu->accel_bias.z);

        /* 校准磁力计 */
        if (RT_EOK != mpu_megy_calibrate(&mpu->megy_asa))
        {
            mpu->initialized = RT_FALSE;
            goto error;
        }

        mpu->megy_bias.x = MEGY_BIAS_X;
        mpu->megy_bias.y = MEGY_BIAS_Y;
        mpu->megy_bias.z = MEGY_BIAS_Z;
        mpu->megy_factor.x = MEGY_FACTOR_X;
        mpu->megy_factor.y = MEGY_FACTOR_Y;
        mpu->megy_factor.z = MEGY_FACTOR_Z;
        
        LOG_I("MEGY ASA : x:%f y:%f z:%f", mpu->megy_asa.x, mpu->megy_asa.y, mpu->megy_asa.z);
        
        /* 使能磁力计连续读取模式 */
        value = 0x16;
        if (RT_EOK != mpu_write_regs(mpu->meg_addr, AK8963_CNTL1, (void *)&value, 1))
        {
            mpu->initialized = RT_FALSE;
            goto error;
        }
        
        LOG_I("mpu initialized completed!");
        return RT_EOK;
    }
    
error:
    rt_free(mpu);
    mpu = RT_NULL;
    return -RT_EIO;
}

rt_err_t mpu_read_accel(mpu_sensor_data_t *accel)
{
    char buf[6] = {0};

    if (RT_NULL == accel)
        return -RT_EINVAL;
    
    if (RT_EOK == mpu_read_regs(mpu->mpu_addr, MPU6500_ACCEL_XOUT_H, buf, 6))
    {
        accel->x =  buf[0] << 8;
        accel->x |= buf[1];
        accel->y =  buf[2] << 8;
        accel->y |= buf[3];
        accel->z =  buf[4] << 8;
        accel->z |= buf[5];
        //实际数据与手册相反
        //accel->x = -accel->x;
        //accel->y = -accel->y;
        //accel->z = -accel->z;
        return RT_EOK;
    }
    
    return -RT_ENOSYS;
}

rt_err_t mpu_read_gyro(mpu_sensor_data_t *gyro)
{
    char        buf[6] = {0};

    if (RT_NULL == gyro)
        return -RT_EINVAL;
    
    if (RT_EOK == mpu_read_regs(mpu->mpu_addr, MPU6500_GYRO_XOUT_H, buf, 6))
    {
        gyro->x =  buf[0] << 8;
        gyro->x |= buf[1];
        gyro->y =  buf[2] << 8;
        gyro->y |= buf[3];
        gyro->z =  buf[4] << 8;
        gyro->z |= buf[5];
        return RT_EOK;
    }
    
    return -RT_ENOSYS;
}


/*
*  磁力计读取注意事项:
*  1.读取前必须使能连续读模式，0x0A写入0X16
*  2.每次读取完后必须读取一下ST2状态寄存器，否则数据不再刷新
*/
rt_err_t mpu_read_megy(mpu_sensor_data_t *megy)
{
    rt_uint8_t  buf[7] = {0};

    if (RT_NULL == megy)
        return -RT_EINVAL;

    /* 这里一定要读7个数据，第7个数据为状态寄存器，要读取一下才会更新磁力计值 */
    if (RT_EOK == mpu_read_regs(mpu->meg_addr, AK8963_HXL, buf, 7))
    {
        //磁力计的x,y和加速计是相反的，故这里反转一下
        megy->x = buf[0];
        megy->x |=  buf[1] << 8;
        megy->y = buf[2];
        megy->y |=  buf[3] << 8;
        megy->z = buf[4];
        megy->z |=  buf[5] << 8;
        return RT_EOK;
    }
    
    //mpu_read_regs(mpu->meg_addr, AK8963_ST2, buf, 1);
    return -RT_ENOSYS;
}


static int mpu9250_driver_init(void)
{
    mpu_device_t mpu9250 = {0};

    mpu9250.i2c_bus_name = MPU9250_I2C_BUS_NAME;
    mpu9250.mpu_addr     = MPU9250_I2C_ADDR;
    mpu9250.meg_addr     = AK8963_I2C_ADDR>>1;

    return mpu_init(&mpu9250);
}

/* 单位: g * m/s2 */
rt_err_t mpu_read_accel_value(mpu_sensor_value_t *accel)
{
    mpu_sensor_data_t accel_org = {0};

    if (RT_NULL == accel)
        return -RT_EINVAL;
    
    if (RT_EOK == mpu_read_accel(&accel_org))
    {
        accel->x = (float)accel_org.x / mpu->accel_fs_unit - mpu->accel_bias.x;
        accel->y = (float)accel_org.y / mpu->accel_fs_unit - mpu->accel_bias.y;
        accel->z = (float)accel_org.z / mpu->accel_fs_unit - mpu->accel_bias.z;

        return RT_EOK;
    }

    return -RT_EIO;
}

/* 单位: rad/s */
rt_err_t mpu_read_gyro_value(mpu_sensor_value_t *gyro)
{
    mpu_sensor_data_t gyrol_org = {0};

    if (RT_NULL == gyro)
        return -RT_EINVAL;
    
    if (RT_EOK == mpu_read_gyro(&gyrol_org))
    {
        
        gyro->x = ((float)gyrol_org.x / mpu->gyro_fs_unit - mpu->gyro_bias.x) / 57.29572f;
        gyro->y = ((float)gyrol_org.y / mpu->gyro_fs_unit - mpu->gyro_bias.y) / 57.29572f;
        gyro->z = ((float)gyrol_org.z / mpu->gyro_fs_unit - mpu->gyro_bias.z) / 57.29572f;
        
        return RT_EOK;
    }

    return -RT_EIO;
}

/* 单位: uT */
rt_err_t mpu_read_megy_value(mpu_sensor_value_t *megy)
{
    mpu_sensor_data_t megy_org = {0};

    if (RT_NULL == megy)
        return -RT_EINVAL;
    
    if (RT_EOK == mpu_read_megy(&megy_org))
    {
        megy->x = ((float)megy_org.x * 0.15 * mpu->megy_asa.x - mpu->megy_bias.x) * mpu->megy_factor.x;
        megy->y = ((float)megy_org.y * 0.15 * mpu->megy_asa.y - mpu->megy_bias.y) * mpu->megy_factor.y;
        megy->z = ((float)megy_org.z * 0.15 * mpu->megy_asa.z - mpu->megy_bias.z)  * mpu->megy_factor.z;
        
        return RT_EOK;
    }

    return -RT_EIO;
}

/* 校准加速计、陀螺仪并返回偏差 */
rt_err_t mpu_gyro_accel_calibrate(mpu_sensor_value_t *gyro_bias, mpu_sensor_value_t *accel_bias)
{
//    rt_uint8_t  data[12]; // data array to hold accelerometer and gyro x, y, z, data
    rt_int32_t  tgyro_bias[3]  = {0, 0, 0};
    rt_int32_t  taccel_bias[3] = {0, 0, 0};
    
    /* 取100个样本解算偏差 */
    for (int i = 0; i < 100; i++)
    {
        mpu_sensor_data_t accel_temp = {0, 0, 0};
        mpu_sensor_data_t gyro_temp  = {0, 0, 0};

        if (RT_EOK == mpu_read_accel(&accel_temp) && RT_EOK == mpu_read_gyro(&gyro_temp))
        {
            // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
            taccel_bias[0] += (int32_t) accel_temp.x; 
            taccel_bias[1] += (int32_t) accel_temp.y;
            taccel_bias[2] += (int32_t) accel_temp.z;
            tgyro_bias[0]  += (int32_t) gyro_temp.x;
            tgyro_bias[1]  += (int32_t) gyro_temp.y;
            tgyro_bias[2]  += (int32_t) gyro_temp.z;
        }
        else
        {
            return -RT_EIO;
        }
    }

    // 取平均值
    taccel_bias[0] /= 100;
    taccel_bias[1] /= 100;
    taccel_bias[2] /= 100;
    tgyro_bias[0]  /= 100;
    tgyro_bias[1]  /= 100;
    tgyro_bias[2]  /= 100;

    // 移除Z轴方向的重力影响
    if(taccel_bias[2] > 0L)
        taccel_bias[2] -= mpu->accel_fs_unit;  
    else
        taccel_bias[2] += mpu->accel_fs_unit;

    // 将偏差写到偏差寄存器，这个寄存器会在 重启的时候清0
//    data[0] = (-tgyro_bias[0]/4  >> 8) & 0xFF; // 4分频写入
//    data[1] = (-tgyro_bias[0]/4)       & 0xFF; // 偏差是用来加到测量值上面的，所以要取反
//    data[2] = (-tgyro_bias[1]/4  >> 8) & 0xFF;
//    data[3] = (-tgyro_bias[1]/4)       & 0xFF;
//    data[4] = (-tgyro_bias[2]/4  >> 8) & 0xFF;
//    data[5] = (-tgyro_bias[2]/4)       & 0xFF;

    // 写入陀螺仪偏差到偏差寄存器,实测无用
//    if (RT_EOK != mpu_write_regs(mpu->mpu_addr, MPU6500_XG_OFFSET_H, (void *)data, 6))
//        return -RT_EIO;

    //输出陀螺仪偏差值
    if (RT_NULL != gyro_bias)
    {
        gyro_bias->x = (float) tgyro_bias[0]/(float) mpu->gyro_fs_unit;  
        gyro_bias->y = (float) tgyro_bias[1]/(float) mpu->gyro_fs_unit;
        gyro_bias->z = (float) tgyro_bias[2]/(float) mpu->gyro_fs_unit;
    }

    //输出加速计偏差值
    if (RT_NULL != accel_bias)
    {
        accel_bias->x = (float) taccel_bias[0]/(float) mpu->accel_fs_unit;  
        accel_bias->y = (float) taccel_bias[1]/(float) mpu->accel_fs_unit;
        accel_bias->z = (float) taccel_bias[2]/(float) mpu->accel_fs_unit;
    }

    return RT_EOK;
}

/* 磁力计校准 */
rt_err_t mpu_megy_calibrate(mpu_sensor_value_t *megy_bias)
{
    // 首先提取每个磁力计轴的工厂校准
    rt_uint8_t rawData[3];    // x/y/z gyro calibration data stored here
    rt_uint8_t value = 0x00;
    
    // Power down magnetometer  
    if (RT_EOK != mpu_write_regs(mpu->meg_addr, AK8963_CNTL1, (void *)&value, 1))
        return -RT_EIO;
    rt_thread_mdelay(10);
    
    // Enter Fuse ROM access mode
    value = 0x0f;
    if (RT_EOK != mpu_write_regs(mpu->meg_addr, AK8963_CNTL1, (void *)&value, 1))
        return -RT_EIO;
    rt_thread_mdelay(10);

    // 读取x轴，y轴和z轴校准值
    if (NULL != megy_bias && RT_EOK == mpu_read_regs(mpu->meg_addr, AK8963_ASAX, (void *)&rawData, 3))
    {
        // Return x-axis sensitivity adjustment values, etc.
        megy_bias->x =  (float)(rawData[0]);
        megy_bias->y =  (float)(rawData[1]);  
        megy_bias->z =  (float)(rawData[2]);
    }

    value = 0x00;
    // Power down magnetometer  
    if (RT_EOK != mpu_write_regs(mpu->meg_addr, AK8963_CNTL1, (void *)&value, 1))
        return -RT_EIO;
    rt_thread_mdelay(10);

    return RT_EOK;
}

/* 1.测量用户环境磁力计的基准偏差值和比例因子
 * 2.线程中不断刷新这个函数并不断旋转传感器
 * Debug:
 *  比例因子: x:0.952974  y:1.078247 z: 0.977305
 *  偏差    : x:39.768749 y:6.924024 z:-2.216602
 */
void mpu_megy_calibrate_measure_update(void) 
{
    rt_int32_t          mag_bias[3]     = {0, 0, 0};    //用户环境偏差
    rt_int32_t          mag_scale[3]    = {0, 0, 0};    //用户环境磁场量程
    mpu_sensor_data_t   mag_temp        = {0, 0, 0};

    /* 不断更新基准值，算出偏差和比例因子 */
    static mpu_sensor_value_t  mag_scale_bias  = {0, 0, 0};    //换算后实际偏差
    static mpu_sensor_value_t  mag_scale_factor = {0, 0, 0};   //用户环境磁场比例因子
    static mpu_sensor_data_t   mag_max         = {-32767, -32767, -32767};
    static mpu_sensor_data_t   mag_min         = {32767, 32767, 32767};
    

    /* 获取磁力计值 */
    mpu_read_megy(&mag_temp);

    /* 获取最大值、最小值 */
    if(mag_temp.x > mag_max.x)
        mag_max.x = mag_temp.x;
    if(mag_temp.x < mag_min.x)
        mag_min.x = mag_temp.x;
    if(mag_temp.y > mag_max.y)
        mag_max.y = mag_temp.y;
    if(mag_temp.y < mag_min.y)
        mag_min.y = mag_temp.y;
    if(mag_temp.z > mag_max.z)
        mag_max.z = mag_temp.z;
    if(mag_temp.z < mag_min.z)
        mag_min.z = mag_temp.z;
    
    // 原始数据中心点值
    mag_bias[0]  = (mag_max.x + mag_min.x)/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max.y + mag_min.y)/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max.z + mag_min.z)/2;  // get average z mag bias in counts

    // 刻度换算后的偏差值
    mag_scale_bias.x = (float) mag_bias[0] * 0.15 * mpu->megy_asa.x;
    mag_scale_bias.y = (float) mag_bias[1] * 0.15 * mpu->megy_asa.y;   
    mag_scale_bias.z = (float) mag_bias[2] * 0.15 * mpu->megy_asa.z; 

    LOG_I("MAG SCALE BIAS x:%f y:%f z:%f", mag_scale_bias.x, mag_scale_bias.y, mag_scale_bias.z);
       
    // 实际量程大小
    mag_scale[0]  = (mag_max.x - mag_min.x)/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max.y - mag_min.y)/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max.z - mag_min.z)/2;  // get average z axis max chord length in counts

    // 量程取均值
    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    //输出比例因子
    mag_scale_factor.x = avg_rad/((float)mag_scale[0]);
    mag_scale_factor.y = avg_rad/((float)mag_scale[1]);
    mag_scale_factor.z = avg_rad/((float)mag_scale[2]);

    LOG_I("MAG SCALE FACTOR x:%f y:%f z:%f", mag_scale_factor.x, mag_scale_factor.y, mag_scale_factor.z);
}



#ifdef RT_USING_FINSH
#include <finsh.h>


/* FINSH 调试命令 */
#ifdef FINSH_USING_MSH
int mpu9250_test(int argc, char **argv)
{
    rt_int8_t value = 0;
    rt_int8_t id    = 0;
    rt_int8_t reg   = 0;
    mpu_sensor_data_t sensor = {0};
    
    if (RT_NULL != strstr(argv[1], "-id"))
    {
        mpu_read_regs(mpu->mpu_addr, MPU6500_WHO_AM_I, &value, 1);
        rt_kprintf("MPU ID: 0X%X\r\n", value);
    }
    else if (RT_NULL != strstr(argv[1], "-r"))
    {
        if (argc > 3)
        {
            id = strtol(argv[2], NULL, 16);
            reg = strtol(argv[3], NULL, 16);
            mpu_read_regs(id, reg, &value, 1);
            rt_kprintf("MPU Read I2C:0X%02X Reg: 0X%02X Value: 0X%02X\r\n", id, reg, value&0xFF);
        }
        else
        {
            reg = strtol(argv[2], NULL, 16);
            mpu_read_regs(mpu->mpu_addr, reg, &value, 1);
            rt_kprintf("MPU Read Reg: 0X%02X Value: 0X%02X\r\n", reg, value&0xFF);
        }
    }
    else if (RT_NULL != strstr(argv[1], "-w"))
    {
        if (argc > 4)
        {
            id = strtol(argv[2], NULL, 16);
            reg = strtol(argv[3], NULL, 16);
            value = strtol(argv[4], NULL, 16);
            mpu_write_regs(id, reg, &value, 1);
            rt_kprintf("MPU Write I2C: 0X%02X Reg: 0X%02X Value: 0X%02X\r\n", id, reg, value);
        }
        else
        {
            reg = strtol(argv[2], NULL, 16);
            value = strtol(argv[3], NULL, 16);
            mpu_write_regs(mpu->mpu_addr, reg, &value, 1);
            rt_kprintf("MPU Write Reg: 0X%02X Value: 0X%02X\r\n", reg, value);
        }
    }
    else if (RT_NULL != strstr(argv[1], "-a"))
    {
        mpu_read_accel(&sensor);
        rt_kprintf("MPU Accel x:%d y:%d z:%d\r\n", sensor.x, sensor.y, sensor.z);
    }
    else if (RT_NULL != strstr(argv[1], "-g"))
    {
        mpu_read_gyro(&sensor);
        rt_kprintf("MPU Gyro x:%d y:%d z:%d\r\n", sensor.x, sensor.y, sensor.z);
    }
    else if (RT_NULL != strstr(argv[1], "-m"))
    {
        mpu_read_megy(&sensor);
        rt_kprintf("MPU Megy x:%d y:%d z:%d\r\n", sensor.x, sensor.y, sensor.z);
    }
        
    return RT_EOK;
}

MSH_CMD_EXPORT(mpu9250_test, mpu9250_test -id/-q);

#endif /* FINSH_USING_MSH */
#endif /* RT_USING_FINSH */

INIT_DEVICE_EXPORT(mpu9250_driver_init);


