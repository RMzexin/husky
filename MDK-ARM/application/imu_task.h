#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__
#include "stm32f4xx_hal.h"
#include <stdint.h>

#define Acceleration_Of_Gravity     9.87F

typedef struct
{
	int16_t gyro[3];
	int16_t acc [3];
	
	float pitch;
	float yaw  ;
	float roll ;
}IMU_data_t;

typedef struct
{
	float gyrox;
	float gyroy;
	float gyroz;
	
	float accx;
	float accy;
	float accz;
	
	float pitch;
	float yaw;
	float roll;
	
	float yaw_dri;
}IMU_t;

uint8_t Get_IMU_Data(void);
void IMU_Cali(void);
uint8_t MPU9250_GetGyro(float *X, float *Y, float *Z);
uint8_t MPU9250_GetAccel(float *X, float *Y, float *Z);

#endif

