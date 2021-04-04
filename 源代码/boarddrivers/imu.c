#include "mpu9250.h"
#include "common.h"
#include <math.h>
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

//#define DRV_DEBUG
#define LOG_TAG             "imu"
#include <drv_log.h>

/* IMU调节参数 */
#define Kp 10.0f                            // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.008f                           // integral gain governs rate of convergence of gyroscope biases
//#define halfT 0.001f                        // half the sample period采样周期的一半

struct imu_data_t {
    rt_thread_t         hthread;
    rt_thread_t         hros;
    char                cmd[50];
    mpu_sensor_value_t  accel;
    mpu_sensor_value_t  gyro;
    mpu_sensor_value_t  megy;
};

struct imu_data_t m_imu = {
    .hthread = RT_NULL,
    .hros    = RT_NULL,
    .cmd     = {0},
    .accel   = {0},
    .gyro    = {0},
    .megy    = {0},
};



typedef struct {
    float       yaw;
    float       pitch;
    float       roll;
}euler_angle_t;

static euler_angle_t  q_angle = {0};
static float q[4] = {1, 0, 0, 0};                  // quaternion elements representing the estimated orientation
static float exInt = 0, eyInt = 0, ezInt = 0;      // scaled integral error
static rt_uint32_t updata_time_curr   = 0;
static rt_uint32_t updata_time_last   = 0;
static rt_uint32_t updata_time_deltat = 0;


float one_filter_angle = 0;
//一阶互补滤波
float one_filter(float angle_m,float gyro_m, float dt_m)
{

    float K1 = 0.1; // 对加速度计取值的权重
    float dt = dt_m;//注意：dt的取值为滤波器采样时间

    one_filter_angle = K1 * angle_m+ (1-K1) * (one_filter_angle + gyro_m * dt);
    return one_filter_angle;
}

float P[2][2]   = {{1,0},{0,1}};//过程协方差
float Pdot[4]   = {0,0,0,0};
float Q_angle   = 0.001;    //过程噪声协方差
float Q_gyro    = 0.003;    //角速度数据置信度，陀螺仪飘移噪声协方差
float R_angle   = 0.5;      //测量噪声的协方差(即测量偏差)
char  C_0       = 1;
float angle_err = 0;        //q_bias为陀螺仪飘移
float PCt_0     = 0;
float PCt_1     = 0;
float E         = 0;        //计算过程量
float K_0       = 0;
float K_1       = 0;
float t_0       = 0;
float t_1       = 0;
float kalman_filter(float angle_m, float gyro_m, float dt_m)
{

    //滤波参数
    float dt        = dt_m;     //卡尔曼采样时间


    static float q_bias              = 0;   //陀螺仪偏差
    static float kalman_filter_angle = 0;   //下时刻最优估计值角度
    static float kalman_filter_angle_dot = 0;

    
    kalman_filter_angle += (gyro_m - q_bias) * dt;    //卡尔曼预测方程，认为每次飘移相同，

    //计算过程协方差矩阵的微分矩阵
    Pdot[0] = Q_angle - P[0][1] - P[1][0];
    Pdot[1] = -P[1][1];
    Pdot[2] = -P[1][1];
    Pdot[3] = Q_gyro;

    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;

    PCt_0 = C_0 * P[0][0];     //矩阵乘法中间变量
    PCt_1 = C_0 * P[1][0];

    E = R_angle + C_0 * PCt_0; //分母

    K_0 = PCt_0 / E;           //增益值
    K_1 = PCt_1 / E;

    angle_err = angle_m - kalman_filter_angle;    
    kalman_filter_angle += K_0 * angle_err; //对状态的卡尔曼估计，最优角度
    q_bias += K_1 * angle_err;
    kalman_filter_angle_dot = gyro_m - q_bias;//最优角速度

    t_0 = PCt_0;     //矩阵计算中间变量
    t_1 = C_0 * P[0][1];

    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;

    return kalman_filter_angle;
}

void IMUupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    float norm, halfT;
    float hx, hy, hz, bx, bz;
    float wx, wy, wz;
    float vx, vy, vz;
    float ex, ey, ez;
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];   // short name local variable for readability

    // 先把这些用得到的值算好
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    if(ax*ay*az==0)
    return;

    if(mx*my*mz == 0)
    return;

    //刷新时间间隔
    updata_time_curr = rt_tick_get();   //ms
    updata_time_deltat = updata_time_curr > updata_time_last ? updata_time_curr - updata_time_last : 0;
    updata_time_last = updata_time_curr;
    halfT = (float)updata_time_deltat / 2000.0;

    //acc数据归一化
    norm = sqrt(ax*ax + ay*ay + az*az);
    norm = 1.0f / norm;        // use reciprocal for division
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    //mag数据归一化
    norm = sqrt(mx*mx + my*my + mz*mz);
    norm = 1.0f / norm;        // use reciprocal for division
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm;

    //磁力计B系旋转到N系
    hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);  
    hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);  
    hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 -q2q2);          
    bx = sqrt((hx*hx) + (hy*hy));  
    bz = hz;

    // estimated direction of gravity and flux (v and w)估计重力方向和流量/变迁
    vx = 2*(q1q3 - q0q2);                                                   //四元素中xyz的表示
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3 ;

    wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);  
    wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);  
    wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2); 

    // error is sum of cross product between reference direction of fields and direction measured by sensors 向量外积在相减得到差分就是误差
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);  
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);  
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    exInt = exInt + ex * Ki;                                                //对误差进行积分
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    // adjusted gyroscope measurements
    gx = gx + Kp*ex + exInt;                                                //将误差PI后补偿到陀螺仪，即补偿零点漂移
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;                                                //这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

    // integrate quaternion rate and normalise                              //四元素的微分方程
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    // normalise quaternion
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    norm = 1.0f / norm;        // use reciprocal for division
    q[0] = q0 * norm;
    q[1] = q1 * norm;
    q[2] = q2 * norm;
    q[3] = q3 * norm;

    q_angle.yaw   = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1) * 57.3;     // yaw
    q_angle.pitch = -asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;                                   // pitch
    q_angle.roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.3;   // roll

}



static void imu_ros_entry(void *parameter)
{
    sensor_msg msg;
    
    while (1)
    {
        if(RT_EOK != mpu_read_accel_value(&m_imu.accel))
        {
            rt_thread_mdelay(10); 
            continue;
        }
        if(RT_EOK !=mpu_read_gyro_value(&m_imu.gyro)) 
        {
            rt_thread_mdelay(10); 
            continue;
        }
        if(RT_EOK !=mpu_read_megy_value(&m_imu.megy)) 
        { 
            rt_thread_mdelay(10);
            continue;
        }
        IMUupdate(m_imu.accel.x, m_imu.accel.y, m_imu.accel.z, m_imu.gyro.x, m_imu.gyro.y, m_imu.gyro.z, m_imu.megy.y, m_imu.megy.x, m_imu.megy.z); //磁力计和加速X轴和Y是相反的，故这里交叉磁力计x,y轴
        msg.x = m_imu.accel.x;
        msg.y = m_imu.accel.y;
        msg.z = m_imu.accel.z;

        send_pack(ROS_DATA_ACCEL, &msg, sizeof(sensor_msg));
        msg.x = m_imu.gyro.x;
        msg.y = m_imu.gyro.y;
        msg.z = m_imu.gyro.z;
        send_pack(ROS_DATA_GYRO, &msg, sizeof(msg));
        msg.x = m_imu.megy.x;
        msg.y = m_imu.megy.y;
        msg.z = m_imu.megy.z;
        send_pack(ROS_DATA_MEGY, &msg, sizeof(msg));
        msg.x = q_angle.yaw;
        msg.y = q_angle.pitch;
        msg.z = q_angle.roll;
        send_pack(ROS_DATA_EULER, &msg, sizeof(msg));
        rt_thread_mdelay(30);
    }
}

int imu_init(void)
{
    m_imu.hros = rt_thread_create("imu_ros_process",
                                    imu_ros_entry, NULL,
                                    2048,
                                    15,
                                    5);
    if (m_imu.hros != RT_NULL) 
    {
        rt_thread_startup(m_imu.hros);
    }

    return RT_EOK;
}

#ifdef RT_USING_FINSH
#include <finsh.h>

static void imu_test_entry(void *parameter)
{  
    while (1)
    {
        if (0 == strcmp(m_imu.cmd, "-start"))
        {
            mpu_read_accel_value(&m_imu.accel);
            mpu_read_gyro_value(&m_imu.gyro);
            mpu_read_megy_value(&m_imu.megy);
            IMUupdate(m_imu.accel.x, m_imu.accel.y, m_imu.accel.z, m_imu.gyro.x, m_imu.gyro.y, m_imu.gyro.z, m_imu.megy.y, m_imu.megy.x, m_imu.megy.z); //磁力计和加速X轴和Y是相反的，故这里交叉磁力计x,y轴
            //mpu_megy_calibrate_measure_update();  //测量磁力计校准参数
            //send_waveform_fomate(&m_imu.megy, 12);
            //send_waveform_fomate(&q_angle, sizeof(q_angle));
            ANO_DT_Send_Status(q_angle.roll, q_angle.pitch, q_angle.yaw, 0, 0, 0);    //匿名显示姿态
        }
        rt_thread_mdelay(10);
    }
}

int imu_test(int argc, char **argv)
{
    if (0 == strcmp(argv[1], "-start"))
    {  
        if (RT_NULL == m_imu.hthread)
        {
            m_imu.hthread = rt_thread_create("imu_test_process",
                                            imu_test_entry, NULL,
                                            1024,
                                            15,
                                            5);
            if (m_imu.hthread != RT_NULL)
                rt_thread_startup(m_imu.hthread);
            else
                return -RT_ERROR;
        }
    }
    else if (0 == strcmp(argv[1], "-delet"))
    {
        rt_thread_delete(m_imu.hthread);
        m_imu.hthread = RT_NULL;
        LOG_I("imu test process deleted!");
    }
    memset(m_imu.cmd, 0, sizeof(m_imu.cmd));
    strncpy(m_imu.cmd, argv[1], sizeof(m_imu.cmd));
        
    return RT_EOK;
}

MSH_CMD_EXPORT(imu_test, imu_test -start/-stop/-delet);



/* FINSH 调试命令 */
#ifdef FINSH_USING_MSH
//TODO

#endif /* FINSH_USING_MSH */
#endif /* RT_USING_FINSH */

INIT_APP_EXPORT(imu_init);

