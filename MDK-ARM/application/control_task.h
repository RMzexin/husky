#ifndef __CONTROL_TASK__
#define __CONTROL_TASK__
#include "stm32f4xx_hal.h"

/***** 遥控器模式相关声明 *****/
//总模式切换
#define AUTONOMY   1
#define FOLLOWING  3
#define TWISTING   2
//底盘模式切换
#define CHASSIS_AUTONOMY   1
#define CHASSIS_FOLLOWING  2
#define CHASSIS_TWISTING   3
//云台模式切换
#define GIMBAL_ENCODER     1
#define GIMBAL_IMU         2

//底盘速度调节
#define GO_FORWARD_INC_FACT        0.22f //遥控器前进通道系数
#define LEFT_RIGHT_INC_FACT        0.22f //遥控器平移通道系数
//云台灵敏度调节
#define YAW_ROTATE_INC_FACT    0.00028f //遥控器云台yaw通道系数
#define PITCH_ROTATE_INC_FACT  0.00028f //遥控器云台pitch通道系数

typedef enum 
{
	true,
	false
} Bool;
//云台角度结构体定义
typedef struct 
{
	float last_angle;
	float max;
	float min;
	float middle;
}Angle_Limit_t;

typedef struct 
{
	Angle_Limit_t angle_limit; //角度限制
	float angle_set;           //给定角度
	float relative_angle;      //相对角度
	float actual_angle;        //实际角度
}Angle_t;
 
//底盘速度结构体定义
typedef struct
{
	float vx;
	float vy;
	float wz;
}chassis_speed_t;

void gimbal_change(void);
void chassis_behavior(void);
uint8_t  gimbal_set(void);
#endif
