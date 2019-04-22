#include "control_task.h"
#include "keymouse_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "can_task.h"
#include "imu_task.h"
#include "mains.h"
#include "ramp.h"
#include <stdbool.h>
#include <stdlib.h>

extern IMU_t IMU_chassis ,IMU_gimbal;

extern Encoder_t M6623_encoder_yaw,M6623_encoder_pitch,
	               M6020_encoder_yaw,M6020_encoder_pitch,encoder_pluck;

//volatile Angle_t M6623_pitch_angle ,M6623_yaw_angle;
volatile Angle_t M6020_pitch_angle ,M6020_yaw_angle,M2006_angle,chassis_angle;

chassis_speed_t chassis_speed;

uint8_t chassis_mode , gimbal_mode;
volatile const uint8_t Gimbal_Cali_Complete = 0 ;

ramp_t Tw_left_ramp    = RAMP_GEN_DAFAULT;
ramp_t Tw_right_ramp   = RAMP_GEN_DAFAULT;

//角度上下限声明
#define ANGLE_LIMIT(angle,angle_max,angle_min)        \
{                                                     \
	if(angle > angle_max){                              \
		angle = angle_max;                                \
	}else if(angle<angle_min){                          \
	  angle = angle_min;}                               \
}

//速度上下限声明
#define SPEED_LIMIT(speed,speed_max,speed_min)        \
{                                                     \
	if(speed > speed_max){                              \
		return speed_max;                                 \
	}else if(speed<angle_min){                          \
	  return speed_min;                                 \
	}else {return speed;}                               \
}

//放在大循环里
void gimbal_change(void)
{
	if(Gimbal_Cali_Complete)
		{
			switch ( CHOICE_MODE() )
			{
				//模式一 ：分离运动
				case AUTONOMY :
				{
					M6020_yaw_angle .twist_add_angle = 0.0f;
					M6020_yaw_angle .add_angle       = 0.0f;
					//计算电机编码器与陀螺仪相对角度,确定yaw轴固定角度
					M6020_yaw_angle.gimbal_yaw_set = IMU_gimbal .C_yaw ;
					M6020_yaw_angle.relative_angle = M6020_encoder_yaw.ecd_angle - IMU_chassis.C_yaw ;
					M6020_yaw_angle.yaw_fix_set    = IMU_chassis.C_yaw;
					//pitch轴修正 角度限制（由编码器初次校准给定）
					M6020_pitch_angle .angle_set += Pitch_Rotate_Data() * PITCH_ROTATE_INC_FACT;
					ANGLE_LIMIT(M6020_pitch_angle .angle_set,M6020_pitch_angle .angle_limit .max ,M6020_pitch_angle .angle_limit .min ) ;
					//yaw轴角度修正 限制（由编码器初次校准给定）
					M6020_yaw_angle .angle_set   -= Yaw_Rotate_Data() * YAW_ROTATE_INC_FACT;
					ANGLE_LIMIT(M6020_yaw_angle .angle_set,M6020_yaw_angle .angle_limit .max ,M6020_yaw_angle .angle_limit .min ) ;
					//云台 底盘模式标志位，执行云台与底盘分离运动
					chassis_mode = CHASSIS_AUTONOMY;
					gimbal_mode  = GIMBAL_ENCODER;
					chassis_behavior();break;
				}
				//模式二：跟随运动
				case FOLLOWING :
				{
					M6020_yaw_angle .twist_add_angle = 0.0f;
					M6020_yaw_angle.gimbal_yaw_set = IMU_gimbal .C_yaw ;
					//pitch轴修正 角度限制（由编码器初次校准给定）
					M6020_pitch_angle .angle_set += Pitch_Rotate_Data() * PITCH_ROTATE_INC_FACT;
					ANGLE_LIMIT(M6020_pitch_angle .angle_set,M6020_pitch_angle .angle_limit .max ,M6020_pitch_angle .angle_limit .min ) ;
					//陀螺仪偏移设定角度，云台角度限制
					M6020_yaw_angle .add_angle   -= Yaw_Rotate_Data() * YAW_ROTATE_INC_FACT;
					M6020_yaw_angle .angle_set = Correct_Angle_Feedback();
					ANGLE_LIMIT(M6020_yaw_angle .angle_set,M6020_yaw_angle .angle_limit .max ,M6020_yaw_angle .angle_limit .min ) ;
					//云台 底盘模式标志位，执行云台与底盘跟随运动
					chassis_mode = CHASSIS_FOLLOWING;
					gimbal_mode  = GIMBAL_ENCODER ;
					chassis_behavior();break;
				}
				//模式三：扭腰运动
				case TWISTING :
				{
					//pitch轴修正 角度限制（由编码器初次校准给定）
					M6020_pitch_angle .angle_set += Pitch_Rotate_Data() * PITCH_ROTATE_INC_FACT;
					ANGLE_LIMIT(M6020_pitch_angle .angle_set,M6020_pitch_angle .angle_limit .max ,M6020_pitch_angle .angle_limit .min);
					//当陀螺仪偏移设定角度，云台修正角度
					M6020_yaw_angle .add_angle       -= Yaw_Rotate_Data() * YAW_ROTATE_INC_FACT;
					M6020_yaw_angle .twist_add_angle -= Yaw_Rotate_Data() * YAW_ROTATE_INC_FACT;
					M6020_yaw_angle .angle_set = Correct_Angle_Feedback();
					ANGLE_LIMIT(M6020_yaw_angle .angle_set,M6020_yaw_angle .angle_limit .max ,M6020_yaw_angle .angle_limit .min);
					//云台 底盘模式标志位，执行云台固定 底盘扭腰运动
					chassis_mode = CHASSIS_TWISTING;
					gimbal_mode  = GIMBAL_IMU ;
					chassis_behavior();break;
				}
			}
		}
	else 
	{
		//云台校准 角度初始化
		gimbal_cali(&M6020_pitch_angle ,&M6020_yaw_angle,&M2006_angle,&M6020_encoder_yaw ,&M6020_encoder_pitch,&encoder_pluck);
	}	
}
//PC角度修正(放在ROS通信回调函数中）
void gimbal_PC_correct(int32_t *PC_yaw_add,int32_t*PC_pitch_add)
{
	M6020_pitch_angle .angle_set += *PC_pitch_add ;
	ANGLE_LIMIT(M6020_pitch_angle .angle_set,M6020_pitch_angle .angle_limit .max ,M6020_pitch_angle .angle_limit .min);
	if(CHOICE_MODE() == AUTONOMY)
	{
		M6020_yaw_angle .angle_set += *PC_yaw_add ;
		ANGLE_LIMIT(M6020_yaw_angle .angle_set,M6020_yaw_angle .angle_limit .max ,M6020_yaw_angle .angle_limit .min);
	}
	else if(CHOICE_MODE() == FOLLOWING || CHOICE_MODE() == TWISTING )
	{
		M6020_yaw_angle .add_angle += *PC_yaw_add ;
	}
}
//IMU角度修正（放在IMU任务里）
float Correct_Angle_Feedback()
{
	if(CHOICE_MODE() == AUTONOMY || CHOICE_MODE() == FOLLOWING )
	{
	return  (M6020_yaw_angle.yaw_fix_set+M6020_yaw_angle.relative_angle)\
		     -(IMU_chassis.C_yaw - M6020_yaw_angle.yaw_fix_set)           \
	       +M6020_yaw_angle .add_angle;
	}
	else if(CHOICE_MODE() ==TWISTING)
	{
	return  (M6020_yaw_angle.yaw_fix_set+M6020_yaw_angle.relative_angle)                           \
		     -(IMU_chassis.C_yaw - M6020_yaw_angle.yaw_fix_set)                                      \
	       +M6020_yaw_angle .add_angle                                                             \
	       -(M6020_yaw_angle.gimbal_yaw_set - M6020_yaw_angle .twist_add_angle - IMU_gimbal .C_yaw ) *0.5f ;
	}                          
}

//放在大循环里
bool static chassis_max_get = GET_INITIAL ;
bool static chassis_min_get = GET_INITIAL ;
static uint8_t chassis_mode_calc_set;
void chassis_behavior()
{
	switch( chassis_mode )
	{
		case CHASSIS_AUTONOMY :
		{
			ramp_init(&Tw_left_ramp, TWISTING_RAMP_TIME );
			ramp_init(&Tw_right_ramp,TWISTING_RAMP_TIME );
			//控制底盘前后平移，左右转弯；左拨杆控制云台
			chassis_speed .vx = Go_Forward_Data()* GO_FORWARD_INC_FACT ;
		  chassis_speed .vy = 0.0f ;
		  chassis_speed .wz = -Left_Right_Data()* LEFT_RIGHT_INC_FACT;
			chassis_mode_calc_set = AUTONOMY ;break;
		}
		case CHASSIS_FOLLOWING :
		{
			//控制底盘前后平移，左右平移；底盘跟随云台运动
			chassis_speed .vx = Go_Forward_Data()* GO_FORWARD_INC_FACT ;
		  chassis_speed .vy = Left_Right_Data()* LEFT_RIGHT_INC_FACT ;
		  chassis_speed .wz = Follow_Speed_Calc(&M6020_encoder_yaw.ecd_angle,&M6020_yaw_angle .angle_limit .middle);
			chassis_mode_calc_set = AUTONOMY ;break;
		}
		/***** 底盘自动扭腰运动 *****/
		case CHASSIS_TWISTING :
		{
			M6020_yaw_angle.correct_angle = M6020_encoder_yaw.ecd_angle - M6020_yaw_angle .angle_limit .middle;
			//两个判断语句判断底盘达到指定角度即反转
			if((M6020_encoder_yaw.ecd_angle - M6020_yaw_angle .angle_limit .middle) < chassis_angle .angle_set)
			{
				//获取随机转动角度，判断标志位（反转后执行一次）
				if(chassis_min_get == true )
				{
					chassis_angle .angle_set = (M6020_yaw_angle .angle_limit .middle  - M6020_yaw_angle .angle_limit .min ) *((rand()%100000)*0.00001f)*TWISTING_INC_FACT;
				  chassis_min_get = GET_COMPLETE  ;
					chassis_max_get = GET_INITIAL   ;
				}
				//PID计算底盘旋转速度（角度反馈）
				chassis_speed .wz = -Twist_Speed_Calc(&chassis_angle .angle_set,&M6020_yaw_angle.correct_angle)* ramp_calc(&Tw_right_ramp);
			  ramp_init(&Tw_left_ramp, TWISTING_RAMP_TIME );
			}
			//同上
			else if((M6020_encoder_yaw.ecd_angle - M6020_yaw_angle .angle_limit .middle) > chassis_angle .angle_set)
			{
				if(chassis_max_get == true )
				{
					chassis_angle .angle_set = (M6020_yaw_angle .angle_limit .middle  - M6020_yaw_angle .angle_limit .max ) *((rand()%100000)*0.00001f)*TWISTING_INC_FACT;
				  chassis_max_get = GET_COMPLETE ;
					chassis_min_get = GET_INITIAL   ;
				}
				chassis_speed .wz = -Twist_Speed_Calc(&chassis_angle .angle_set,&M6020_yaw_angle.correct_angle)* ramp_calc(&Tw_left_ramp);
				ramp_init(&Tw_right_ramp, TWISTING_RAMP_TIME );
			}
			else 
			{
				chassis_angle .angle_set = (M6020_yaw_angle .angle_limit .middle  - M6020_yaw_angle .angle_limit .max )*TWISTING_INC_FACT;
				chassis_speed .wz = -Twist_Speed_Calc(&chassis_angle .angle_set,&M6020_yaw_angle.correct_angle);
			}
			chassis_mode_calc_set = TWISTING ;break;
		}
		default :
		{
			chassis_speed .vx = 0.0f ;
		  chassis_speed .vy = 0.0f ;
		  chassis_speed .wz = 0.0f ;
		}
	}
	chassis_pid_calc(chassis_speed .vx ,chassis_speed .vy ,chassis_speed .wz,chassis_mode_calc_set);
}

//放在定时器里（用于云台PID计算）
static uint8_t yaw_angle_feedback_set;
uint8_t gimbal_set()
{
	//判断拨弹轮电机是否堵转
	if(M2006_angle .angle_set - M2006_angle .actual_angle < 1000.0f)
	{
		M2006_angle .angle_set += pluck_angle_add();
	}
	switch ( gimbal_mode )
	{
		case GIMBAL_ENCODER :
		{
			M6020_yaw_angle   .actual_angle = M6020_encoder_yaw   .ecd_angle;
		  M6020_pitch_angle .actual_angle = M6020_encoder_pitch .ecd_angle;
		  M2006_angle .actual_angle       = encoder_pluck .ecd_angle      ;
		  yaw_angle_feedback_set = ENCODER_FEEDBACK;break;
		}
		case GIMBAL_IMU     :
		{
			M6020_yaw_angle   .actual_angle = M6020_encoder_yaw   .ecd_angle;
		  M6020_pitch_angle .actual_angle = M6020_encoder_pitch .ecd_angle;
		  M2006_angle .actual_angle       = encoder_pluck .ecd_angle      ;
		  yaw_angle_feedback_set = IMU_FEEDBACK;break;
		}
		default :
		{
			M6020_yaw_angle   .actual_angle = M6020_encoder_yaw   .ecd_angle;
		  M6020_pitch_angle .actual_angle = M6020_encoder_pitch .ecd_angle;
		  M2006_angle .actual_angle       = encoder_pluck .ecd_angle      ;
		}
		  return 0 ;
	}
	gimbal_pid_calc(&M6020_yaw_angle.angle_set     ,&M6020_pitch_angle .angle_set   ,&M2006_angle .angle_set   ,
	                &M6020_yaw_angle .actual_angle ,&M6020_pitch_angle .actual_angle,&M2006_angle .actual_angle,
	                yaw_angle_feedback_set);
	return 1 ;
}
