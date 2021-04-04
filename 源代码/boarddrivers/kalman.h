#ifndef __KALMAN_H__
#define __KALMAN_H__


typedef  struct{
	double filterValue;     //k-1时刻的滤波值，即是k-1时刻的值
	double kalmanGain;      //   Kalamn增益
	double A;               // x(n)=A*x(n-1)+u(n),u(n)~N(0,Q)
	double H;               // z(n)=H*x(n)+w(n),w(n)~N(0,R)
	double Q;               //预测过程噪声偏差的方差
	double R;               //测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
	double P;               //估计误差协方差
}  KalmanInfo;

void Init_KalmanInfo(KalmanInfo* info, double Q, double R);
double KalmanFilter(KalmanInfo* kalmanInfo, double lastMeasurement);

#endif

