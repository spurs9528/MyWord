/**
 * @file AHRS.c
 * @author Segway Inside 
 * @brief 
 * @version 0.1
 * @date 2020-03-24
 * 
 * @copyright Copyright (c) 2019 纳恩博（北京）科技有限公司
 * All rights reserved.
 * 
 */
#include "AHRS.h"
#include <math.h>
#include "kalman.h"

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;	//> 表示估计方向的四元数
float exInt = 0, eyInt = 0, ezInt = 0;	//> 标度积分误差
int roll, pitch, yaw;					//> 全局姿态解算结果

void AHRSupdate(float gx, float gy, float gz, 
				float ax, float ay, float az, float mx, float my, float mz) 
{
	float q0temp,q1temp,q2temp,q3temp;	//> 四元数暂存变量，求解微分方程时要用
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	float mag_hx, mag_hy;
	double theta, beta, gama;

	/* 辅助变量 */
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

	/* 把加速度计的三维向量转成单位向量 
	   这样即使变更了量程也不需要修改KP参数，因为这里归一化了 */
	norm = sqrt(ax*ax + ay*ay + az*az);
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;

	/* 用当前姿态计算出重力场在三个轴上的分量(v) */
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;

	/* 总误差包括磁场方向误差和重力方向误差 */
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);

	/* 积分误差 */
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;
          
	/* 用叉积误差来做PI修正陀螺零偏 */
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;

    /* 下面进行姿态的更新，也就是四元数微分方程的求解
	   暂存当前值用于计算 */
	q0temp=q0;
    q1temp=q1;
    q2temp=q2;
    q3temp=q3;
	
	/* 采用一阶毕卡解法 */
//    q0 = q0temp + (-q1temp*gx - q2temp*gy -q3temp*gz)*halfT;
//    q1 = q1temp + (q0temp*gx + q2temp*gz -q3temp*gy)*halfT;
//    q2 = q2temp + (q0temp*gy - q1temp*gz +q3temp*gx)*halfT;
//    q3 = q3temp + (q0temp*gz + q1temp*gy -q2temp*gx)*halfT;
   
	/* 采用二阶毕卡解法 */
	float delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);
    q0 = (1-delta_2/8)*q0temp + (-q1temp*gx - q2temp*gy -q3temp*gz)*halfT;		
    q1 = (1-delta_2/8)*q1temp + (q0temp*gx + q2temp*gz -q3temp*gy)*halfT;
    q2 = (1-delta_2/8)*q2temp + (q0temp*gy - q1temp*gz +q3temp*gx)*halfT;
    q3 = (1-delta_2/8)*q3temp + (q0temp*gz + q1temp*gy -q2temp*gx)*halfT;

	/* 单位化四元数在空间旋转时不会拉伸，仅有旋转角度，这类似线性代数里的正交变换 */
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;

	/* 四元数到欧拉角的转换 */
	beta = atan2f(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
	theta  = asinf (2 * (q0 * q2 - q3 * q1));
	//gama = atan2f(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q3 * q3 + q2 * q2));
	roll  = beta * (180 / 3.14159265358979);
	pitch = theta * (180 / 3.14159265358979);
	//yaw   = gama * (180 / 3.14159265358979);
	
	mag_hx = mx * cos(theta) - mz * sin(theta);
	mag_hy = mx * sin(theta) * sin(beta) + my * cos(beta) - mz * cos(theta)*sin(beta);
		
	int mag_yaw = (atan2(mag_hy, mag_hx) * (180 / 3.14159265358979));
	
	Kalman_Filter(gz, mag_yaw);	//> 磁力计航向角作为观测量进行陀螺仪Yaw积分角度的卡尔曼滤波
	yaw = kal_angle;

	if(yaw < 0)
			yaw += 360;
}