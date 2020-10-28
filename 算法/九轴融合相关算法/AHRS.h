/**
 * @file AHRS.h
 * @author Segway Inside 
 * @brief 
 * @version 0.1
 * @date 2020-03-24
 * 
 * @copyright Copyright (c) 2019 纳恩博（北京）科技有限公司
 * All rights reserved.
 * 
 * @description
 * “DCM滤波器”的四元数实现[Mayhony等人]合并了磁失真补偿算法，该算法消除了预先定义参考磁通方向
 *（bx bz）的需要，并仅将磁失真的影响限制在偏航轴上。用户必须将“halfT”定义为（采样周期/2），滤
 * 波器包含“Kp”和“Ki”。全局变量“q0”，“q1”、“q2”、“q3”是表示估计方向的四元数元素。用户必须在每
 * 个采样周期调用'AHRSupdate()'，并解析校准陀螺仪（'gx'，'gy'，'gz'）、加速计（'ax'，'ay'，
 * 'ay'）和磁强计（'mx'，'my'，'mz'）数据。陀螺仪单位是弧度/秒，加速度计和磁强计单位是无关的矢
 * 量是正规化。  
 *
 * 本算法采用四元数微分方程融合陀螺仪与加速度计求解姿态角Pitch、Roll，采集磁力计航向角数据并用
 * 卡尔曼滤波对陀螺仪积分角度进行滤波处理得到航向角Yaw
 *
 * [gx,gy,gz]为陀螺仪的测量值 rad/s
 * [ax,ay,az]为加速度的测量值 任意单位
 * [mx,my,mz]为地磁计的测量值 任意单位
 */

#ifndef	__AHRS_H
#define __AHRS_H


#define Kp 2.0f							//> 比例增益控制加速度计/磁强计的收敛速度
#define Ki 0.005f						//> 积分增益控制陀螺偏差收敛速度
#define halfT 0.005f					//> 采样周期10ms的1/2

extern int roll, pitch, yaw;			//> 9轴数据融合后的姿态角

void AHRSupdate(float gx, float gy, float gz, 
				float ax, float ay, float az, float mx, float my, float mz);
					
#endif