#include "chassis_task.h"
#include "can_task.h"
#include "can.h"
#include "pid.h"

extern Encoder_t encoder_chassis[4];
static pid_t M3508_motor_speed_pid[4],M3508_follow_gimbal_pid;

void chassis_init(void)
{
	uint8_t i = 0;
	const static float motor_speed_pid[3]   = {M3508_MOTOR_SPEED_PID_KP  ,M3508_MOTOR_SPEED_PID_KI  ,M3508_MOTOR_SPEED_PID_KD  };  
  const static float follow_ginbal_pid[3] = {M3508_FOLLOW_GIMBAL_PID_KP,M3508_FOLLOW_GIMBAL_PID_KI,M3508_FOLLOW_GIMBAL_PID_KD};
	 //底盘麦轮运动PID初始化
	for (i = 0;i < 4; i++)
	{
	 PID_Init(&M3508_motor_speed_pid[i]     ,motor_speed_pid              ,
		         M3508_MOTOR_SPEED_VOLTAFE_MAX,M3508_MOTOR_SPEED_VOLTAFE_MIN,
		         M3508_MOTOR_SPEED_MAXOUTPUT  ,M3508_MOTOR_SPEED_MINOUTPUT   );
	}
   //底盘跟随云台角度闭环PID初始化
	 PID_Init(&M3508_follow_gimbal_pid        ,follow_ginbal_pid              ,
		         M3508_FOLLOW_GIMBAL_VOLTAFE_MAX,M3508_FOLLOW_GIMBAL_VOLTAFE_MIN,
		         M3508_FOLLOW_GIMBAL_MAXOUTPUT  ,M3508_FOLLOW_GIMBAL_MINOUTPUT   );
}

void chassis_pid_calc(float speed_vx,float speed_vy,float speed_wz)
{
	static float wheel_speed[4];
	uint8_t i = 0;
	wheel_speed[0] =  speed_vx + speed_vy  + speed_wz ;
	wheel_speed[1] = -speed_vx + speed_vy  + speed_wz ;
	wheel_speed[2] =  speed_vx + speed_vy  - speed_wz ;
	wheel_speed[3] = -speed_vx + speed_vy  - speed_wz ;
	for (i = 0;i < 4; i++ )
	{
		PID_Calc(&M3508_motor_speed_pid[i],encoder_chassis[i].speed_rpm,wheel_speed[i]);
	}
	Set_CM_Speed(&hcan1,(int16_t)M3508_motor_speed_pid[0].pidout,
	                    (int16_t)M3508_motor_speed_pid[1].pidout,
	                    (int16_t)M3508_motor_speed_pid[2].pidout,
	                    (int16_t)M3508_motor_speed_pid[3].pidout );
	
}

