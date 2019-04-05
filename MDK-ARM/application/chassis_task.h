#ifndef __CHASSISTASK_H__
#define __CHASSISTASK_H__
#include "can_task.h"
#include "control_task.h"

//µ×ÅÌÆ½ÒÆPID
#define M3508_MOTOR_SPEED_PID_KP               40.0f
#define M3508_MOTOR_SPEED_PID_KI                0.2f
#define M3508_MOTOR_SPEED_PID_KD                0.0f
#define M3508_MOTOR_SPEED_VOLTAFE_MAX       50000.0f
#define M3508_MOTOR_SPEED_VOLTAFE_MIN      -50000.0f
#define M3508_MOTOR_SPEED_MAXOUTPUT          8000.0f
#define M3508_MOTOR_SPEED_MINOUTPUT         -8000.0f

//µ×ÅÌÐý×ªPID
#define M3508_TWISTING_PID_KP                  40.0f
#define M3508_TWISTING_PID_KI                   0.0f
#define M3508_TWISTING_PID_KD                   0.0f
#define M3508_TWISTING_VOLTAFE_MAX           1000.0f
#define M3508_TWISTING_VOLTAFE_MIN          -1000.0f
#define M3508_TWISTING_MAXOUTPUT             8000.0f
#define M3508_TWISTING_MINOUTPUT            -8000.0f

//µ×ÅÌÐý×ª½Ç¶ÈÐÞÕýPID
#define CHASSIS_TWISTING_CORRECT_PID_KP             8.0f
#define CHASSIS_TWISTING_CORRECT_PID_KI             0.2f
#define CHASSIS_TWISTING_CORRECT_PID_KD             0.0f
#define CHASSIS_TWISTING_CORRECT_VOLTAFE_MAX       80.0f
#define CHASSIS_TWISTING_CORRECT_VOLTAFE_MIN      -80.0f
#define CHASSIS_TWISTING_CORRECT_MAXOUTPUT        130.0f
#define CHASSIS_TWISTING_CORRECT_MINOUTPUT       -130.0f

//µ×ÅÌÐý×ª½Ç¶ÈÐÞÕýPID
#define CHASSIS_FOLLOWING_CORRECT_PID_KP            8.0f
#define CHASSIS_FOLLOWING_CORRECT_PID_KI            0.0f
#define CHASSIS_FOLLOWING_CORRECT_PID_KD            0.0f
#define CHASSIS_FOLLOWING_CORRECT_VOLTAFE_MAX      40.0f
#define CHASSIS_FOLLOWING_CORRECT_VOLTAFE_MIN     -40.0f
#define CHASSIS_FOLLOWING_CORRECT_MAXOUTPUT       130.0f
#define CHASSIS_FOLLOWING_CORRECT_MINOUTPUT      -130.0f


void chassis_init(void);
void chassis_pid_calc(float speed_vx,float speed_vy,float speed_wz,const uint8_t chassis_calc_set);
float Twist_Speed_Calc(volatile float *angle_set,volatile float *correct_angle);
float Follow_Speed_Calc(volatile float *angle_set,volatile float *correct_angle);
#endif

