#include "chassis_task.h"
#include "can_task.h"
#include "can.h"
#include "pid.h"

extern Encoder_t encoder_chassis[4];
pid_t M3508_motor_speed_pid[4],M3508_twisting_pid[4],chassis_twisting_correct_pid,chassis_following_correct_pid;

void chassis_init(void)
{
	uint8_t i , z = 0;
	const static float motor_speed_pid[3]    = {M3508_MOTOR_SPEED_PID_KP  ,M3508_MOTOR_SPEED_PID_KI  ,M3508_MOTOR_SPEED_PID_KD  };  
  const static float motor_twisting_pid[3] = {M3508_TWISTING_PID_KP  ,M3508_TWISTING_PID_KI  ,M3508_TWISTING_PID_KD  };
  const static float twisting_correct_pid[3]   = {CHASSIS_TWISTING_CORRECT_PID_KP ,CHASSIS_TWISTING_CORRECT_PID_KI ,CHASSIS_TWISTING_CORRECT_PID_KD }; 
	const static float following_correct_pid[3]  = {CHASSIS_FOLLOWING_CORRECT_PID_KP,CHASSIS_FOLLOWING_CORRECT_PID_KI,CHASSIS_FOLLOWING_CORRECT_PID_KD};
	//底盘麦轮运动PID初始化
	for (i = 0;i < 4; i++)
	{
	 PID_Init(&M3508_motor_speed_pid[i]     ,motor_speed_pid              ,
		         M3508_MOTOR_SPEED_VOLTAFE_MAX,M3508_MOTOR_SPEED_VOLTAFE_MIN,
		         M3508_MOTOR_SPEED_MAXOUTPUT  ,M3508_MOTOR_SPEED_MINOUTPUT   );
	}
	for (z = 0;z < 4; z++)
	{
	 PID_Init(&M3508_twisting_pid[z]     ,motor_twisting_pid        ,
		         M3508_TWISTING_VOLTAFE_MAX,M3508_TWISTING_VOLTAFE_MIN,
		         M3508_TWISTING_MAXOUTPUT  ,M3508_TWISTING_MINOUTPUT   );
	}
	
   //底盘旋转云台角度闭环PID初始化
	 PID_Init(&chassis_twisting_correct_pid        ,twisting_correct_pid              ,
		         CHASSIS_TWISTING_CORRECT_VOLTAFE_MAX,CHASSIS_TWISTING_CORRECT_VOLTAFE_MIN,
		         CHASSIS_TWISTING_CORRECT_MAXOUTPUT  ,CHASSIS_TWISTING_CORRECT_MINOUTPUT   );
	//底盘跟随云台角度闭环PID初始化
	 PID_Init(&chassis_following_correct_pid        ,following_correct_pid              ,
		         CHASSIS_FOLLOWING_CORRECT_VOLTAFE_MAX,CHASSIS_FOLLOWING_CORRECT_VOLTAFE_MIN,
		         CHASSIS_FOLLOWING_CORRECT_MAXOUTPUT  ,CHASSIS_FOLLOWING_CORRECT_MINOUTPUT   );
}

void chassis_pid_calc(float speed_vx,float speed_vy,float speed_wz,const uint8_t chassis_calc_set)
{
	static float wheel_speed[4];
	uint8_t i = 0;
	wheel_speed[0] =  speed_vx + speed_vy  - speed_wz ;
	wheel_speed[1] = -speed_vx + speed_vy  - speed_wz ;
	wheel_speed[2] =  speed_vx - speed_vy  - speed_wz ;
	wheel_speed[3] = -speed_vx - speed_vy  - speed_wz ;
	if(chassis_calc_set == AUTONOMY)
	{
		for (i = 0;i < 4; i++ )
		{
			PID_Calc(&M3508_motor_speed_pid[i],encoder_chassis[i].speed_rpm,wheel_speed[i]);
		}
		Set_CM_Speed(&hcan1,(int16_t)M3508_motor_speed_pid[0].pidout,
	                      (int16_t)M3508_motor_speed_pid[1].pidout,
	                      (int16_t)M3508_motor_speed_pid[2].pidout,
	                      (int16_t)M3508_motor_speed_pid[3].pidout );
//		Set_CM_Speed(&hcan1,(int16_t)0.0,
//	                      (int16_t)0.0,
//	                      (int16_t)0.0,
//	                      (int16_t)0.0 );		
	}
	if(chassis_calc_set == TWISTING)
	{
				for (i = 0;i < 4; i++ )
		{
			PID_Calc(&M3508_twisting_pid[i],encoder_chassis[i].speed_rpm,wheel_speed[i]);
		}
		Set_CM_Speed(&hcan1,(int16_t)M3508_twisting_pid[0].pidout,
	                      (int16_t)M3508_twisting_pid[1].pidout,
	                      (int16_t)M3508_twisting_pid[2].pidout,
	                      (int16_t)M3508_twisting_pid[3].pidout );	
//		Set_CM_Speed(&hcan1,(int16_t)0.0,
//	                      (int16_t)0.0,
//	                      (int16_t)0.0,
//	                      (int16_t)0.0 );		
	}
	
}
float Twist_Speed_Calc(volatile float *angle_set,volatile float *correct_angle)
{
	PID_Calc(&chassis_twisting_correct_pid , *correct_angle,*angle_set);
	return 	  chassis_twisting_correct_pid .pidout;
}

float Follow_Speed_Calc(volatile float *angle_set,volatile float *correct_angle)
{
	PID_Calc(&chassis_following_correct_pid , *correct_angle,*angle_set);
	return 	  chassis_following_correct_pid .pidout;	
}
