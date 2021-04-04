#include "kalman.h"

/**
* @brief Init_KalmanInfo   初始化滤波器的初始值
* @param info  滤波器指针
* @param Q 预测噪声方差 由系统外部测定给定
* @param R 测量噪声方差 由系统外部测定给定
*/
void Init_KalmanInfo(KalmanInfo* info, double Q, double R)
{
	info->A = 1;            //标量卡尔曼
	info->H = 1;            //
	info->P = 10;           //后验状态估计值误差的方差的初始值（不要为0问题不大）
	info->Q = Q;            //预测（过程）噪声方差 影响收敛速率，可以根据实际需求给出
	info->R = R;            //测量（观测）噪声方差 可以通过实验手段获得
	info->filterValue = 0;  // 测量的初始值
}


double KalmanFilter(KalmanInfo* kalmanInfo, double lastMeasurement)
{
	//预测下一时刻的值
	double predictValue = kalmanInfo->A* kalmanInfo->filterValue;   //x的先验估计由上一个时间点的后验估计值和输入信息给出，此处需要根据基站高度做一个修改
	
	//求协方差
	kalmanInfo->P = kalmanInfo->A*kalmanInfo->A*kalmanInfo->P + kalmanInfo->Q;  //计算先验均方差 p(n|n-1)=A^2*p(n-1|n-1)+q
	double preValue = kalmanInfo->filterValue;  //记录上次实际坐标的值
 
	//计算kalman增益
	kalmanInfo->kalmanGain = kalmanInfo->P*kalmanInfo->H / (kalmanInfo->P*kalmanInfo->H*kalmanInfo->H + kalmanInfo->R);  //Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R)
	//修正结果，即计算滤波值
	kalmanInfo->filterValue = predictValue + (lastMeasurement - predictValue)*kalmanInfo->kalmanGain;  //利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))
	//更新后验估计
	kalmanInfo->P = (1 - kalmanInfo->kalmanGain*kalmanInfo->H)*kalmanInfo->P;//计算后验均方差  P[n|n]=(1-K[n]*H)*P[n|n-1]
 
	return  kalmanInfo->filterValue;
}


