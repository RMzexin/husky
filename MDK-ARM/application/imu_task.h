#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__
#include "stm32f4xx_hal.h"
#include <stdint.h>

#define Acceleration_Of_Gravity     9.87F

typedef struct
{
	int16_t gyro[3];
	int16_t acc [3];
	
	float ecd_pitch;
	float ecd_yaw  ;
	float ecd_roll ;
}IMU_data_t;

typedef struct
{
	float gyrox;
	float gyroy;
	float gyroz;
	
	float accx;
	float accy;
	float accz;
	//初始角度
	float ecd_pitch;
	float ecd_yaw;
	float ecd_roll;
	//上一次角度
	float last_pitch;
	float last_yaw;
	float last_roll;
	
	float yaw_dri;
	int32_t round_cnt;   //圈数
	//初始角度
	float C_pitch;
	float C_yaw;
	float C_roll;
	
}IMU_t;

void Get_IMU_Data(void);
void IMU_Cali(void);
uint8_t MPU9250_GetGyro(float *X, float *Y, float *Z);
uint8_t MPU9250_GetAccel(float *X, float *Y, float *Z);

#endif

