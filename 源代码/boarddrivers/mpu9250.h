#ifndef __MPU9250_H__
#define __MPU9250_H__
#include <rtthread.h>
#include <rtdevice.h>

//mpu Reg -- Map
#define MPU6500_SELF_TEST_XG        (0x00)
#define MPU6500_SELF_TEST_YG        (0x01)
#define MPU6500_SELF_TEST_ZG        (0x02)
#define MPU6500_SELF_TEST_XA        (0x0D)
#define MPU6500_SELF_TEST_YA        (0x0E)
#define MPU6500_SELF_TEST_ZA        (0x0F)
#define MPU6500_XG_OFFSET_H         (0x13)
#define MPU6500_XG_OFFSET_L         (0x14)
#define MPU6500_YG_OFFSET_H         (0x15)
#define MPU6500_YG_OFFSET_L         (0x16)
#define MPU6500_ZG_OFFSET_H         (0x17)
#define MPU6500_ZG_OFFSET_L         (0x18)
#define MPU6500_SMPLRT_DIV          (0x19)
#define MPU6500_CONFIG              (0x1A)
#define MPU6500_GYRO_CONFIG         (0x1B)
#define MPU6500_ACCEL_CONFIG        (0x1C)
#define MPU6500_ACCEL_CONFIG_2      (0x1D)
#define MPU6500_LP_ACCEL_ODR        (0x1E)
#define MPU6500_MOT_THR             (0x1F)
#define MPU6500_FIFO_EN             (0x23)
#define MPU6500_I2C_MST_CTRL        (0x24)
#define MPU6500_I2C_SLV0_ADDR       (0x25)
#define MPU6500_I2C_SLV0_REG        (0x26)
#define MPU6500_I2C_SLV0_CTRL       (0x27)
#define MPU6500_I2C_SLV1_ADDR       (0x28)
#define MPU6500_I2C_SLV1_REG        (0x29)
#define MPU6500_I2C_SLV1_CTRL       (0x2A)
#define MPU6500_I2C_SLV2_ADDR       (0x2B)
#define MPU6500_I2C_SLV2_REG        (0x2C)
#define MPU6500_I2C_SLV2_CTRL       (0x2D)
#define MPU6500_I2C_SLV3_ADDR       (0x2E)
#define MPU6500_I2C_SLV3_REG        (0x2F)
#define MPU6500_I2C_SLV3_CTRL       (0x30)
#define MPU6500_I2C_SLV4_ADDR       (0x31)
#define MPU6500_I2C_SLV4_REG        (0x32)
#define MPU6500_I2C_SLV4_DO         (0x33)
#define MPU6500_I2C_SLV4_CTRL       (0x34)
#define MPU6500_I2C_SLV4_DI         (0x35)
#define MPU6500_I2C_MST_STATUS      (0x36)
#define MPU6500_INT_PIN_CFG         (0x37)
#define MPU6500_INT_ENABLE          (0x38)
#define MPU6500_INT_STATUS          (0x3A)
#define MPU6500_ACCEL_XOUT_H        (0x3B)
#define MPU6500_ACCEL_XOUT_L        (0x3C)
#define MPU6500_ACCEL_YOUT_H        (0x3D)
#define MPU6500_ACCEL_YOUT_L        (0x3E)
#define MPU6500_ACCEL_ZOUT_H        (0x3F)
#define MPU6500_ACCEL_ZOUT_L        (0x40)
#define MPU6500_TEMP_OUT_H          (0x41)
#define MPU6500_TEMP_OUT_L          (0x42)
#define MPU6500_GYRO_XOUT_H         (0x43)
#define MPU6500_GYRO_XOUT_L         (0x44)
#define MPU6500_GYRO_YOUT_H         (0x45)
#define MPU6500_GYRO_YOUT_L         (0x46)
#define MPU6500_GYRO_ZOUT_H         (0x47)
#define MPU6500_GYRO_ZOUT_L         (0x48)
#define MPU6500_EXT_SENS_DATA_00    (0x49)
#define MPU6500_EXT_SENS_DATA_01    (0x4A)
#define MPU6500_EXT_SENS_DATA_02    (0x4B)
#define MPU6500_EXT_SENS_DATA_03    (0x4C)
#define MPU6500_EXT_SENS_DATA_04    (0x4D)
#define MPU6500_EXT_SENS_DATA_05    (0x4E)
#define MPU6500_EXT_SENS_DATA_06    (0x4F)
#define MPU6500_EXT_SENS_DATA_07    (0x50)
#define MPU6500_EXT_SENS_DATA_08    (0x51)
#define MPU6500_EXT_SENS_DATA_09    (0x52)
#define MPU6500_EXT_SENS_DATA_10    (0x53)
#define MPU6500_EXT_SENS_DATA_11    (0x54)
#define MPU6500_EXT_SENS_DATA_12    (0x55)
#define MPU6500_EXT_SENS_DATA_13    (0x56)
#define MPU6500_EXT_SENS_DATA_14    (0x57)
#define MPU6500_EXT_SENS_DATA_15    (0x58)
#define MPU6500_EXT_SENS_DATA_16    (0x59)
#define MPU6500_EXT_SENS_DATA_17    (0x5A)
#define MPU6500_EXT_SENS_DATA_18    (0x5B)
#define MPU6500_EXT_SENS_DATA_19    (0x5C)
#define MPU6500_EXT_SENS_DATA_20    (0x5D)
#define MPU6500_EXT_SENS_DATA_21    (0x5E)
#define MPU6500_EXT_SENS_DATA_22    (0x5F)
#define MPU6500_EXT_SENS_DATA_23    (0x60)
#define MPU6500_I2C_SLV0_DO         (0x63)
#define MPU6500_I2C_SLV1_DO         (0x64)
#define MPU6500_I2C_SLV2_DO         (0x65)
#define MPU6500_I2C_SLV3_DO         (0x66)
#define MPU6500_I2C_MST_DELAY_CTRL  (0x67)
#define MPU6500_SIGNAL_PATH_RESET   (0x68)
#define MPU6500_MOT_DETECT_CTRL     (0x69)
#define MPU6500_USER_CTRL           (0x6A)
#define MPU6500_PWR_MGMT_1          (0x6B)
#define MPU6500_PWR_MGMT_2          (0x6C)
#define MPU6500_FIFO_COUNTH         (0x72)
#define MPU6500_FIFO_COUNTL         (0x73)
#define MPU6500_FIFO_R_W            (0x74)
#define MPU6500_WHO_AM_I            (0x75)	// mpu6500 id = 0x71
#define MPU6500_XA_OFFSET_H         (0x77)
#define MPU6500_XA_OFFSET_L         (0x78)
#define MPU6500_YA_OFFSET_H         (0x7A)
#define MPU6500_YA_OFFSET_L         (0x7B)
#define MPU6500_ZA_OFFSET_H         (0x7D)
#define MPU6500_ZA_OFFSET_L         (0x7E)
	
#define MPU6050_ID				    (0x68)
#define MPU6500_ID			        (0x71)			// mpu6500 id = 0x70
#define MPU9250_ID					(0x70)			// mpu9250 id = 0x71
/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */
#define AK8963_I2C_ADDR             (0x18)
#define AK8963_Device_ID            (0x48)
// Read-only Reg
#define AK8963_WIA                  (0x00)
#define AK8963_INFO                 (0x01)
#define AK8963_ST1                  (0x02)
#define AK8963_HXL                  (0x03)
#define AK8963_HXH                  (0x04)
#define AK8963_HYL                  (0x05)
#define AK8963_HYH                  (0x06)
#define AK8963_HZL                  (0x07)
#define AK8963_HZH                  (0x08)
#define AK8963_ST2                  (0x09)
// Write/Read Reg
#define AK8963_CNTL1                (0x0A)
#define AK8963_CNTL2                (0x0B)
#define AK8963_ASTC                 (0x0C)
#define AK8963_TS1                  (0x0D)
#define AK8963_TS2                  (0x0E)
#define AK8963_I2CDIS               (0x0F)
// Read-only Reg ( ROM )
#define AK8963_ASAX                 (0x10)
#define AK8963_ASAY                 (0x11)
#define AK8963_ASAZ                 (0x12)

#define ARRAY_SIZE(ar)     (sizeof(ar)/sizeof(ar[0]))


typedef struct {
    char        *i2c_bus_name;      /* I2C总线设备句柄 */
    char        mpu_addr;           /* mpu从机地址 */
    char        meg_addr;           /* 磁力计从机地址 */
}mpu_device_t;

typedef struct {
    rt_uint8_t  reg;
    rt_uint8_t  data;
}mpu_reg_data_t;

/* 传感器原始数据 */
typedef struct {
    rt_int16_t  x;
    rt_int16_t  y;
    rt_int16_t  z;
}mpu_sensor_data_t;

/* 传感器换算后的值 */
typedef struct {
    float       x;
    float       y;
    float       z;
}mpu_sensor_value_t;


/* ------------user config----------*/
#define MPU9250_I2C_BUS_NAME          "i2c1"  /* 传感器连接的I2C总线设备名称 */
#define MPU9250_I2C_ADDR              0x68    /* 从机地址 */
#define MPU9250_


/*
AK8963 ID读取
mpu9250_test -w 0x6a 0x20
mpu9250_test -w 0x25 0x8c
mpu9250_test -w 0x26 0x00
mpu9250_test -w 0x27 0x81
mpu9250_test -r 0x49

//开启一次数据读取
mpu9250_test -w 0x6a 0x20
mpu9250_test -w 0x25 0x0c
mpu9250_test -w 0x26 0x0a
mpu9250_test -w 0x27 0x81
mpu9250_test -w 0x63 0x16
mpu9250_test -w 0x25 0x8c
mpu9250_test -w 0x26 0x0a
mpu9250_test -w 0x27 0x81
mpu9250_test -r 0x49

//延时读取6个磁力计数据
mpu9250_test -w 0x25 0x8c
mpu9250_test -w 0x26 0x03
mpu9250_test -w 0x27 0x86
mpu9250_test -r 0x49

*/


rt_err_t mpu_read_accel(mpu_sensor_data_t *accel);
rt_err_t mpu_read_gyro(mpu_sensor_data_t *gyro);
rt_err_t mpu_read_megy(mpu_sensor_data_t *megy);
rt_err_t mpu_read_accel_value(mpu_sensor_value_t *accel);
rt_err_t mpu_read_gyro_value(mpu_sensor_value_t *gyro);
rt_err_t mpu_read_megy_value(mpu_sensor_value_t *megy);

void mpu_megy_calibrate_measure_update(void);

#endif
