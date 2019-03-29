#ifndef __CHASSISTASK_H__
#define __CHASSISTASK_H__

//µ×ÅÌÆ½ÒÆPID
#define M3508_MOTOR_SPEED_PID_KP             40.0f
#define M3508_MOTOR_SPEED_PID_KI              0.2f
#define M3508_MOTOR_SPEED_PID_KD              0.0f
#define M3508_MOTOR_SPEED_VOLTAFE_MAX     50000.0f
#define M3508_MOTOR_SPEED_VOLTAFE_MIN    -50000.0f
#define M3508_MOTOR_SPEED_MAXOUTPUT        8000.0f
#define M3508_MOTOR_SPEED_MINOUTPUT       -8000.0f

//µ×ÅÌÐý×ªPID
#define M3508_FOLLOW_GIMBAL_PID_KP           10.0f
#define M3508_FOLLOW_GIMBAL_PID_KI            1.0f
#define M3508_FOLLOW_GIMBAL_PID_KD            0.0f
#define M3508_FOLLOW_GIMBAL_VOLTAFE_MAX     500.0f
#define M3508_FOLLOW_GIMBAL_VOLTAFE_MIN    -500.0f
#define M3508_FOLLOW_GIMBAL_MAXOUTPUT      1000.0f
#define M3508_FOLLOW_GIMBAL_MINOUTPUT     -1000.0f


void chassis_init(void);
void chassis_pid_calc(float speed_vx,float speed_vy,float speed_wz);
#endif

