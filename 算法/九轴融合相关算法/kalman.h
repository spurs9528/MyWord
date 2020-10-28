/**
 * @file kalman.h
 * @author Segway Inside 
 * @brief 
 * @version 0.1
 * @date 2020-03-24
 * 
 * @copyright Copyright (c) 2019 纳恩博（北京）科技有限公司
 * All rights reserved.
 * 
 * @param
 * dt 滤波器采样时间
 * Q  状态协方差
 * R  观测数据误差协方差
 * 使用者需要在每个采样点调用该函数。通过调整Q 和 R的值来得到满意的滤波效果。

 * 本算法依据加速度计数据解算得到的姿态角作为观测量对陀螺仪积分角度进行卡尔曼滤波
 * （同样可以用磁力计航向角度作为观测量对陀螺仪Yaw积分角度进行滤波）
 */

#ifndef	__KALMAN_H
#define __KALMAN_H

extern float kal_angle, gyro_bias;	//> 系统状态空间：角度和陀螺零漂

void Kalman_Filter(float gyro,float accel);		//> 陀螺角速度gyro、加速度计观测角度accel

#endif
