#ifndef __GIMBALTASK_H__
#define __GIMBALTASK_H__
#include "control_task.h"
#include "can_task.h"

/*********  云台电机 6623 PID参数  *********/

//6623 YAW轴 速度环 PID参数
#define M6623_YAW_SPEED_PID_KP              10.0f
#define M6623_YAW_SPEED_PID_KI               0.0f
#define M6623_YAW_SPEED_PID_KD               0.0f
#define M6623_YAW_SPEED_VOLTAFE_MAX      50000.0f
#define M6623_YAW_SPEED_VOLTAFE_MIN     -50000.0f
#define M6623_YAW_SPEED_MAXOUTPUT         5000.0f
#define M6623_YAW_SPEED_MINOUTPUT        -5000.0f

//6623 PITCH轴 速度环 PID参数
#define M6623_PITCH_SPEED_PID_KP            10.0f
#define M6623_PITCH_SPEED_PID_KI             0.0f
#define M6623_PITCH_SPEED_PID_KD             0.0f
#define M6623_PITCH_SPEED_VOLTAFE_MAX    50000.0f
#define M6623_PITCH_SPEED_VOLTAFE_MIN   -50000.0f
#define M6623_PITCH_SPEED_MAXOUTPUT       5000.0f
#define M6623_PITCH_SPEED_MINOUTPUT      -5000.0f


//6623 YAW轴 角度环 PID参数
#define M6623_YAW_ANGLE_PID_KP               8.0f
#define M6623_YAW_ANGLE_PID_KI               0.0f
#define M6623_YAW_ANGLE_PID_KD               0.0f
#define M6623_YAW_ANGLE_VOLTAFE_MAX      50000.0f
#define M6623_YAW_ANGLE_VOLTAFE_MIN     -50000.0f
#define M6623_YAW_ANGLE_MAXOUTPUT         1000.0f
#define M6623_YAW_ANGLE_MINOUTPUT        -1000.0f

//6623 PITCH轴 角度环 PID参数
#define M6623_PITCH_ANGLE_PID_KP             8.0f
#define M6623_PITCH_ANGLE_PID_KI             0.0f
#define M6623_PITCH_ANGLE_PID_KD             0.0f
#define M6623_PITCH_ANGLE_VOLTAFE_MAX    50000.0f
#define M6623_PITCH_ANGLE_VOLTAFE_MIN   -50000.0f
#define M6623_PITCH_ANGLE_MAXOUTPUT       1000.0f
#define M6623_PITCH_ANGLE_MINOUTPUT      -1000.0f



/*********  云台电机 6020 PID参数  *********/


//6020 YAW轴 速度环 PID参数（由编码器反馈角度）
#define M6020_YAW_SPEED_PID_KP            1822.0f
#define M6020_YAW_SPEED_PID_KI               2.0f
#define M6020_YAW_SPEED_PID_KD               0.0f
#define M6020_YAW_SPEED_VOLTAFE_MAX       5000.0f
#define M6020_YAW_SPEED_VOLTAFE_MIN      -5000.0f
#define M6020_YAW_SPEED_MAXOUTPUT        30000.0f
#define M6020_YAW_SPEED_MINOUTPUT       -30000.0f
//6020 YAW轴 角度环 PID参数
#define M6020_YAW_ANGLE_PID_KP               2.0f
#define M6020_YAW_ANGLE_PID_KI               0.0f
#define M6020_YAW_ANGLE_PID_KD               0.0f
#define M6020_YAW_ANGLE_VOLTAFE_MAX      50000.0f
#define M6020_YAW_ANGLE_VOLTAFE_MIN     -50000.0f
#define M6020_YAW_ANGLE_MAXOUTPUT           10.0f
#define M6020_YAW_ANGLE_MINOUTPUT          -10.0f

//IMU YAW轴 速度环 PID参数（由IMU反馈角度）
#define IMU_YAW_SPEED_PID_KP             592.0f
#define IMU_YAW_SPEED_PID_KI               0.0f
#define IMU_YAW_SPEED_PID_KD               0.0f
#define IMU_YAW_SPEED_VOLTAFE_MAX       5000.0f
#define IMU_YAW_SPEED_VOLTAFE_MIN      -5000.0f
#define IMU_YAW_SPEED_MAXOUTPUT        30000.0f
#define IMU_YAW_SPEED_MINOUTPUT       -30000.0f
//IMU YAW轴 角度环 PID参数
#define IMU_YAW_ANGLE_PID_KP               2.7f
#define IMU_YAW_ANGLE_PID_KI               0.0f
#define IMU_YAW_ANGLE_PID_KD               0.0f
#define IMU_YAW_ANGLE_VOLTAFE_MAX      50000.0f
#define IMU_YAW_ANGLE_VOLTAFE_MIN     -50000.0f
#define IMU_YAW_ANGLE_MAXOUTPUT           15.0f
#define IMU_YAW_ANGLE_MINOUTPUT          -15.0f


//6020 PITCH轴 速度环 PID参数
#define M6020_PITCH_SPEED_PID_KP          1652.0f
#define M6020_PITCH_SPEED_PID_KI             2.0f
#define M6020_PITCH_SPEED_PID_KD             0.0f
#define M6020_PITCH_SPEED_VOLTAFE_MAX     5000.0f
#define M6020_PITCH_SPEED_VOLTAFE_MIN    -5000.0f
#define M6020_PITCH_SPEED_MAXOUTPUT      30000.0f
#define M6020_PITCH_SPEED_MINOUTPUT     -30000.0f
//6020 PITCH轴 角度环 PID参数
#define M6020_PITCH_ANGLE_PID_KP             1.8f
#define M6020_PITCH_ANGLE_PID_KI             0.0f
#define M6020_PITCH_ANGLE_PID_KD             0.0f
#define M6020_PITCH_ANGLE_VOLTAFE_MAX    50000.0f
#define M6020_PITCH_ANGLE_VOLTAFE_MIN   -50000.0f
#define M6020_PITCH_ANGLE_MAXOUTPUT         10.0f
#define M6020_PITCH_ANGLE_MINOUTPUT        -10.0f


/*********  拨弹电机 2006 PID参数  *********/
//2006 速度环 PID参数
#define M2006_SPEED_PID_KP               1.4f
#define M2006_SPEED_PID_KI              0.12f
#define M2006_SPEED_PID_KD               0.0f
#define M2006_SPEED_VOLTAFE_MAX      50000.0f
#define M2006_SPEED_VOLTAFE_MIN     -50000.0f
#define M2006_SPEED_MAXOUTPUT        10000.0f
#define M2006_SPEED_MINOUTPUT       -10000.0f

//2006 角度环 PID参数
#define M2006_ANGLE_PID_KP               1.2f
#define M2006_ANGLE_PID_KI               0.0f
#define M2006_ANGLE_PID_KD               0.0f
#define M2006_ANGLE_VOLTAFE_MAX      50000.0f
#define M2006_ANGLE_VOLTAFE_MIN     -50000.0f
#define M2006_ANGLE_MAXOUTPUT         5000.0f
#define M2006_ANGLE_MINOUTPUT        -5000.0f
typedef struct 
{
	float angle_last;
	float angle_now;
	float angular_speed;
}Gimbal_Motor_t;

typedef struct 
{	
	int16_t yaw;
	int16_t pitch;
	int16_t pluck;
	
	int16_t yaw_cali;
	int16_t pitch_cali;
	int16_t pluck_cali;
}Can_Send_t;

//校准模式切换宏定义
#define GIMBAL_CALI     1
#define GIMBAL_NORMAL   2
//云台yaw轴角度反馈宏定义
#define ENCODER_FEEDBACK 1
#define IMU_FEEDBACK     2

//云台校准中值的时候，发送原始电流值
#define GIMBAL_CALI_MOTOR_SET 6000
#define GIMBAL_CALI_STEP_TIME 1000
#define GIMBAL_CALI_GYRO_LIMIT 0.1f

#define GIMBAL_CALI_YAW_MAX_STEP 1
#define GIMBAL_CALI_YAW_MIN_STEP 2
#define GIMBAL_CALI_PITCH_MAX_STEP 3
#define GIMBAL_CALI_PITCH_MIN_STEP 4
#define GIMBAL_CALI_END_STEP 5

void gimbal_init(void);
void gimbal_pid_calc(volatile float *yaw_angle_set,volatile float *pitch_angle_set,volatile float *pluck_angle_set,
	                   volatile float *yaw_angle    ,volatile float *pitch_angle    ,volatile float *pluck_angle    ,  
										 const uint8_t yaw_angle_feedback_set);
void gimbal_cali(volatile Angle_t *pitch,volatile Angle_t *yaw,Encoder_t *encoder_yaw,Encoder_t *encoder_pitch);
void GIMBAL_CAN_SEND(void);
#endif
