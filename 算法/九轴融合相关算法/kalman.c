/**
 * @file kalman.c
 * @author Segway Inside 
 * @brief 
 * @version 0.1
 * @date 2020-03-24
 * 
 * @copyright Copyright (c) 2019 纳恩博（北京）科技有限公司
 * All rights reserved.
 * 
 */
 
#include "kalman.h"

float Q_angle=0.01;					//> 陀螺仪输出角度的误差协方差
float Q_gyro=0.01;					//> 陀螺仪零点漂移的误差协方差
float R_angle=0.01;					//> 加速度计测量角度的误差协方差
float dt=0.010;   					//> @@@@@@@ kalman滤波器采样时间 @@@@@@@@ 
float P[2][2] = { { 1, 0 },     	//> 误差协方差
                  { 0, 1 } };    

/* 中间变量 */
float Pdot[4] ={0,0,0,0}; 
const char C_0 = 1; 
float PCt_0, PCt_1, E, t_0, t_1;

float angle_err;					//> 观测误差
float K_0, K_1;						//> 卡尔曼增益
float kal_angle=0, gyro_bias=0;		//> 系统状态空间：角度和陀螺零漂

void Kalman_Filter(float gyro,float accel) 
{     
	/* X(k|k-1)=A X(k-1|k-1)+B U(k) ..................................(1)
	*  状态矩阵X为：
	* （ kal_angle
    *    gyro_bias ）
	*  状态转移矩阵A为：
	* （ 1    -dt
    *    0     1 ）
	*/
    kal_angle += (gyro-gyro_bias) * dt;	//> 先验估计   

	/* P(k|k-1)=A P(k-1|k-1) A’+Q ....................................(2)     
	*  误差协方差矩阵Q为：
	* （ Q_kal_angle  0
    *    0   	  	  Q_gyro_bias ）
	*/
    Pdot[0] = Q_angle - P[0][1] - P[1][0];     
    Pdot[1] = - P[1][1];     
    Pdot[2] = - P[1][1];     
    Pdot[3] = Q_gyro;     
    P[0][0] += Pdot[0] * dt; 
    P[0][1] += Pdot[1] * dt;  
    P[1][0] += Pdot[2] * dt;     
    P[1][1] += Pdot[3] * dt; 

	/* Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R) ......................(3)
	*  H:观测系数矩阵，为（ 1  0 ）
	*  R:观测噪声协方差矩阵
	*/
    PCt_0 = C_0 * P[0][0];     
    PCt_1 = C_0 * P[1][0];     
    E = R_angle + C_0 * PCt_0;     
    K_0 = PCt_0 / E; 
    K_1 = PCt_1 / E;   

	/* X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1)) ......................(4)
	*  Z(k):观测量，根据加速度计测量得到的角度值accel
	*/
	angle_err  = accel - kal_angle;
    kal_angle += K_0 * angle_err; 		//> 后验估计     
    gyro_bias += K_1 * angle_err;		//> 后验估计      

	/* P(k|k)=（I-Kg(k) H）P(k|k-1) ..................................(5)
	*/
    t_0 = PCt_0;     
    t_1 = C_0 * P[0][1]; 
    P[0][0] -= K_0 * t_0; 				//> 后验估计的误差协方差    
    P[0][1] -= K_0 * t_1;     
    P[1][0] -= K_1 * t_0;     
    P[1][1] -= K_1 * t_1; 
}