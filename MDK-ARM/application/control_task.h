#ifndef __CONTROL_TASK__
#define __CONTROL_TASK__
#include "stm32f4xx_hal.h"

/***** 遥控器模式相关声明 *****/
//总模式切换
#define AUTONOMY   1
#define FOLLOWING  3
#define TWISTING   2
//底盘模式切换
#define CHASSIS_DEFAULT    0
#define CHASSIS_AUTONOMY   1
#define CHASSIS_FOLLOWING  2
#define CHASSIS_TWISTING   3
//云台模式切换
#define GIMBAL_ENCODER     1
#define GIMBAL_IMU         2
//小陀螺旋转角度获得
#define GET_COMPLETE  false
#define GET_INITIAL   true
#define TWISTING_INC_FACT   0.8428f  //小陀螺限幅系数
#define TWISTING_RAMP_TIME  200

//底盘速度调节
#define GO_FORWARD_INC_FACT        0.35f //前进通道系数
#define LEFT_RIGHT_INC_FACT        0.28f //平移通道系数
#define CHASSIS_TWIST_INC_FACT     0.22f //旋转通道系数
//云台灵敏度调节
#define YAW_ROTATE_INC_FACT    0.00028f //云台yaw通道系数
#define PITCH_ROTATE_INC_FACT  0.00028f //云台pitch通道系数
#define GIMBAL_TWIST_INC_FACT  0.00035f //云台旋转通道系数

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
	float actual_angle;        //实际角度
	
	float relative_angle;      //相对角度
	float yaw_fix_set;         //yaw轴固定相对角度
	float gimbal_yaw_set;      //yaw轴固定角度
	float correct_angle;       //修正角度
	float add_angle;           //跟随时增加的角度
	float twist_add_angle;           //跟随时增加的角度
}Angle_t;
 
//底盘速度结构体定义
typedef struct
{
	float vx;
	float vy;
	float wz;
}chassis_speed_t;

extern uint8_t mode_indicator_light;

void gimbal_change(void);
void chassis_behavior(void);
uint8_t  gimbal_set(void);
float Correct_Angle_Feedback(void);
void gimbal_PC_correct(int32_t*PC_yaw_add,int32_t*PC_pitch_add);

#endif
