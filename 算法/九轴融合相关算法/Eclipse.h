/**
 * @file Eclipse.h
 * @author Segway Inside 
 * @brief 
 * @version 0.1
 * @date 2020-03-24
 * 
 * @copyright Copyright (c) 2019 纳恩博（北京）科技有限公司
 * All rights reserved.
 * 
 */
 
#ifndef	__ECLIPSE_H
#define __ECLIPSE_H

#include "stdio.h"
#include "string.h"
#include "math.h"

#define MATRIX_SIZE 6
#define U8 unsigned char

void ResetMatrix(void);
void CalcData_Input(double x, double y, double z);
void CalcData_Input_average(void);
void DispMatrix(void);
U8 Matrix_GaussElimination(void);
void Matrix_RowSimplify(void);
double* Matrix_Solve(double* solve);

/* 整个椭球拟合的过程演示Demo */
void Ellipsoid_fitting_Process(void);
	
#endif
