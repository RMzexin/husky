#include "gimbal_task.h"
#include "control_task.h"
#include "can.h"
#include "pid.h"
#include <arm_math.h>

Can_Send_t  can_send;
extern Encoder_t M6623_encoder_yaw,M6623_encoder_pitch,
	               M6020_encoder_yaw,M6020_encoder_pitch,
                 encoder_pluck;
 pid_t M6623_yaw_speed_pid , M6623_pitch_speed_pid , M6623_yaw_angle_pid , M6623_pitch_angle_pid,
	     M6020_yaw_speed_pid , M6020_pitch_speed_pid , M6020_yaw_angle_pid , M6020_pitch_angle_pid,
       M2006_speed_pid     , M2006_angle_pid       , IMU_yaw_speed_pid   , IMU_yaw_angle_pid;

Gimbal_Motor_t yaw_gimbal_motor,pitch_gimbal_motor,pluck_motor;

static uint8_t can_send_mode;

void gimbal_init(void)
{
	/***** 6623相关参数初始化 *****/
	//6623PID参数数组赋值
	const static float M6623_yaw_speed[3]   = {M6623_YAW_SPEED_PID_KP,  M6623_YAW_SPEED_PID_KI,  M6623_YAW_SPEED_PID_KD  };
	const static float M6623_pitch_speed[3] = {M6623_PITCH_SPEED_PID_KP,M6623_PITCH_SPEED_PID_KI,M6623_PITCH_SPEED_PID_KD};
	const static float M6623_yaw_angle[3]   = {M6623_YAW_ANGLE_PID_KP,  M6623_YAW_ANGLE_PID_KI,  M6623_YAW_ANGLE_PID_KD  };
	const static float M6623_pitch_angle[3] = {M6623_PITCH_ANGLE_PID_KP,M6623_PITCH_ANGLE_PID_KI,M6623_PITCH_ANGLE_PID_KD};
	//6623yaw轴角度环 速度环 PID初始化
	PID_Init(&M6623_yaw_speed_pid          ,M6623_yaw_speed            ,
	          M6623_YAW_SPEED_VOLTAFE_MAX  ,M6623_YAW_SPEED_VOLTAFE_MIN,
	          M6623_YAW_SPEED_MAXOUTPUT    ,M6623_YAW_SPEED_MINOUTPUT   );
	PID_Init(&M6623_yaw_angle_pid          ,M6623_yaw_angle            ,
	          M6623_YAW_ANGLE_VOLTAFE_MAX  ,M6623_YAW_ANGLE_VOLTAFE_MIN,
	          M6623_YAW_ANGLE_MAXOUTPUT    ,M6623_YAW_ANGLE_MINOUTPUT   );
	//6623pitch轴角度环 速度环 PID初始化
	PID_Init(&M6623_pitch_speed_pid        ,M6623_pitch_speed            ,
	          M6623_PITCH_SPEED_VOLTAFE_MAX,M6623_PITCH_SPEED_VOLTAFE_MIN,
	          M6623_PITCH_SPEED_MAXOUTPUT  ,M6623_PITCH_SPEED_MINOUTPUT   );
	PID_Init(&M6623_pitch_angle_pid        ,M6623_pitch_angle            ,
	          M6623_PITCH_ANGLE_VOLTAFE_MAX,M6623_PITCH_ANGLE_VOLTAFE_MIN,
	          M6623_PITCH_ANGLE_MAXOUTPUT  ,M6623_PITCH_ANGLE_MINOUTPUT   );
	
	
	/***** 6020相关参数初始化 *****/
	const static float M6020_yaw_speed[3]   = {M6020_YAW_SPEED_PID_KP,  M6020_YAW_SPEED_PID_KI,  M6020_YAW_SPEED_PID_KD  };
	const static float M6020_pitch_speed[3] = {M6020_PITCH_SPEED_PID_KP,M6020_PITCH_SPEED_PID_KI,M6020_PITCH_SPEED_PID_KD};
	const static float M6020_yaw_angle[3]   = {M6020_YAW_ANGLE_PID_KP,  M6020_YAW_ANGLE_PID_KI,  M6020_YAW_ANGLE_PID_KD  };
	const static float M6020_pitch_angle[3] = {M6020_PITCH_ANGLE_PID_KP,M6020_PITCH_ANGLE_PID_KI,M6020_PITCH_ANGLE_PID_KD};
	const static float IMU_yaw_angle[3]     = {IMU_YAW_ANGLE_PID_KP,    IMU_YAW_ANGLE_PID_KI,    IMU_YAW_ANGLE_PID_KD    };
	const static float IMU_yaw_speed[3]     = {IMU_YAW_SPEED_PID_KP,    IMU_YAW_SPEED_PID_KI,    IMU_YAW_SPEED_PID_KD    };
	//6020yaw轴角度环 速度环 PID初始化（由编码器反馈角度）
	PID_Init(&M6020_yaw_speed_pid          ,M6020_yaw_speed            ,
	          M6020_YAW_SPEED_VOLTAFE_MAX  ,M6020_YAW_SPEED_VOLTAFE_MIN,
	          M6020_YAW_SPEED_MAXOUTPUT    ,M6020_YAW_SPEED_MINOUTPUT   );
	PID_Init(&M6020_yaw_angle_pid          ,M6020_yaw_angle            ,
	          M6020_YAW_ANGLE_VOLTAFE_MAX  ,M6020_YAW_ANGLE_VOLTAFE_MIN,
	          M6020_YAW_ANGLE_MAXOUTPUT    ,M6020_YAW_ANGLE_MINOUTPUT   );
		//6020yaw轴角度环 速度环 PID初始化（由IMU反馈角度）
	PID_Init(&IMU_yaw_speed_pid          ,IMU_yaw_speed            ,
	          IMU_YAW_SPEED_VOLTAFE_MAX  ,IMU_YAW_SPEED_VOLTAFE_MIN,
	          IMU_YAW_SPEED_MAXOUTPUT    ,IMU_YAW_SPEED_MINOUTPUT   );
	PID_Init(&IMU_yaw_angle_pid          ,IMU_yaw_angle            ,
	          IMU_YAW_ANGLE_VOLTAFE_MAX  ,IMU_YAW_ANGLE_VOLTAFE_MIN,
	          IMU_YAW_ANGLE_MAXOUTPUT    ,IMU_YAW_ANGLE_MINOUTPUT   );
	//6020pitch轴角度环 速度环 PID初始化
	PID_Init(&M6020_pitch_speed_pid        ,M6020_pitch_speed            ,
	          M6020_PITCH_SPEED_VOLTAFE_MAX,M6020_PITCH_SPEED_VOLTAFE_MIN,
	          M6020_PITCH_SPEED_MAXOUTPUT  ,M6020_PITCH_SPEED_MINOUTPUT   );
	PID_Init(&M6020_pitch_angle_pid        ,M6020_pitch_angle            ,
	          M6020_PITCH_ANGLE_VOLTAFE_MAX,M6020_PITCH_ANGLE_VOLTAFE_MIN,
	          M6020_PITCH_ANGLE_MAXOUTPUT  ,M6020_PITCH_ANGLE_MINOUTPUT   );
	
	/***** 2006相关参数初始化 *****/
	const static float M2006_speed[3] = {M2006_SPEED_PID_KP, M2006_SPEED_PID_KI, M2006_SPEED_PID_KD};
	const static float M2006_angle[3] = {M2006_ANGLE_PID_KP, M2006_ANGLE_PID_KI, M2006_ANGLE_PID_KD};
	PID_Init(&M2006_speed_pid          ,M2006_speed            ,
	          M2006_SPEED_VOLTAFE_MAX  ,M2006_SPEED_VOLTAFE_MIN,
	          M2006_SPEED_MAXOUTPUT    ,M2006_SPEED_MINOUTPUT   );
	PID_Init(&M2006_angle_pid          ,M2006_angle            ,
	          M2006_ANGLE_VOLTAFE_MAX  ,M2006_ANGLE_VOLTAFE_MIN,
	          M2006_ANGLE_MAXOUTPUT    ,M2006_ANGLE_MINOUTPUT   );
}

void gimbal_pid_calc(volatile float *yaw_angle_set,volatile float *pitch_angle_set,volatile float *pluck_angle_set,
	                   volatile float *yaw_angle    ,volatile float *pitch_angle    ,volatile float *pluck_angle     ,
										 const uint8_t yaw_feedback_set)
{
	/***** 6020电机双闭环控制 *****/
	yaw_gimbal_motor .angular_speed   = M6020_encoder_yaw .speed_rpm *(2.0f*3.14f/60.0f);
	pitch_gimbal_motor .angular_speed = M6020_encoder_pitch .speed_rpm *(2.0f*3.14f/60.0f);
	pluck_motor .angular_speed        = encoder_pluck .speed_rpm *2.0f*3.14f/60.0f;
	if(yaw_feedback_set == ENCODER_FEEDBACK )
	{
		//yaw轴串级PID计算
	PID_Calc(&M6020_yaw_angle_pid, *yaw_angle, *yaw_angle_set );
	PID_Calc(&M6020_yaw_speed_pid,yaw_gimbal_motor .angular_speed,M6020_yaw_angle_pid .pidout);
	can_send .yaw = M6020_yaw_speed_pid .pidout ;	
	}
	else if(yaw_feedback_set == IMU_FEEDBACK )
	{
	PID_Calc(&IMU_yaw_angle_pid, *yaw_angle, *yaw_angle_set );
	PID_Calc(&IMU_yaw_speed_pid,yaw_gimbal_motor .angular_speed,IMU_yaw_angle_pid .pidout);
	can_send .yaw = IMU_yaw_speed_pid .pidout ;
	}
	//pitch轴串级PID计算
	PID_Calc(&M6020_pitch_angle_pid, *pitch_angle , *pitch_angle_set );
	PID_Calc(&M6020_pitch_speed_pid,pitch_gimbal_motor .angular_speed,M6020_pitch_angle_pid .pidout);
	can_send .pitch = M6020_pitch_speed_pid .pidout ;
	//pluck串级PID计算
	PID_Calc(&M2006_angle_pid, *pluck_angle , *pluck_angle_set );
	PID_Calc(&M2006_speed_pid, pluck_motor .angular_speed,M2006_angle_pid .pidout);
	can_send .pluck = M2006_speed_pid .pidout ;
	
}



//校准声明
#define GIMBAL_CALI_CALCULATE(angle_last, cmd_time, angle, angle_set, step)\
{                                                                          \
	if ((fabs(angle-angle_last)) < GIMBAL_CALI_GYRO_LIMIT){                  \
		(cmd_time)++;                                                          \
		if ((cmd_time) > GIMBAL_CALI_STEP_TIME){                               \
				(cmd_time) = 0;                                                    \
				(angle_set) = (angle);                                             \
				(step)++;}}                                                        \
}	
/***** 云台校准模式（执行一次）得到云台限位角度 *****/
extern uint8_t chassis_mode , gimbal_mode;
extern uint8_t Gimbal_Cali_Complete;
void gimbal_cali(volatile Angle_t *pitch,volatile Angle_t *yaw,Encoder_t *encoder_yaw,Encoder_t *encoder_pitch)
{
	static uint16_t cali_time = 0;
	static uint16_t gimbal_cali_step =1 ;
	if(gimbal_cali_step == GIMBAL_CALI_YAW_MAX_STEP)
		{
			yaw ->angle_limit.last_angle  = encoder_yaw ->ecd_angle;
			can_send .yaw_cali    = GIMBAL_CALI_MOTOR_SET ;
			can_send .pitch_cali  = 0 ;
			can_send .pluck_cali  = 0 ;
			can_send_mode = GIMBAL_CALI ;
			GIMBAL_CALI_CALCULATE(yaw ->angle_limit.last_angle , cali_time ,
			encoder_yaw ->ecd_angle , yaw ->angle_limit.max  , gimbal_cali_step);
		}
		else if(gimbal_cali_step == GIMBAL_CALI_YAW_MIN_STEP)
			{
				yaw ->angle_limit. last_angle = encoder_yaw ->ecd_angle;
			  can_send .yaw_cali    = -GIMBAL_CALI_MOTOR_SET ;
			  can_send .pitch_cali  = 0 ;
			  can_send .pluck_cali  = 0 ;
				can_send_mode = GIMBAL_CALI ;
				GIMBAL_CALI_CALCULATE(yaw ->angle_limit. last_angle , cali_time ,
				encoder_yaw ->ecd_angle , yaw ->angle_limit. min  , gimbal_cali_step);
			}
			else if(gimbal_cali_step == GIMBAL_CALI_PITCH_MAX_STEP)
				{
					pitch ->angle_limit. last_angle = encoder_pitch ->ecd_angle;
					can_send .yaw_cali    = 0 ;
					can_send .pitch_cali  = GIMBAL_CALI_MOTOR_SET ;
					can_send .pluck_cali  = 0 ;
					can_send_mode = GIMBAL_CALI ;
					GIMBAL_CALI_CALCULATE(pitch ->angle_limit. last_angle , cali_time ,
					encoder_pitch ->ecd_angle , pitch ->angle_limit. max , gimbal_cali_step);
				}
				else if(gimbal_cali_step == GIMBAL_CALI_PITCH_MIN_STEP)
					{
						pitch ->angle_limit. last_angle = encoder_pitch ->ecd_angle;
						can_send .yaw_cali    = 0 ;
						can_send .pitch_cali  = -GIMBAL_CALI_MOTOR_SET ;
						can_send .pluck_cali  = 0 ;
						can_send_mode = GIMBAL_CALI ;
						GIMBAL_CALI_CALCULATE(pitch ->angle_limit. last_angle , cali_time ,
						encoder_pitch ->ecd_angle , pitch ->angle_limit. min  , gimbal_cali_step);
					}
					else if(gimbal_cali_step == GIMBAL_CALI_END_STEP)
						{
							pitch ->angle_limit. middle = (pitch ->angle_limit. max + pitch ->angle_limit. min)/2;
							yaw ->angle_limit. middle   = (yaw ->angle_limit. max   + yaw ->angle_limit. min)  /2;
							pitch ->angle_set = pitch ->angle_limit .middle;
							yaw  ->angle_set  = yaw ->angle_limit .middle  ;
							can_send_mode = GIMBAL_NORMAL ;
							gimbal_mode = GIMBAL_ENCODER ; 
							Gimbal_Cali_Complete ++ ;
						}
}
	
void GIMBAL_CAN_SEND()
{
	switch( can_send_mode )
	{
		case GIMBAL_CALI :
	  Set_Gimbal_Current(&hcan1, can_send .yaw_cali , can_send .pitch_cali , can_send .pluck_cali );break;
		case GIMBAL_NORMAL :
		Set_Gimbal_Current(&hcan1,(int16_t)can_send .yaw ,(int16_t)can_send .pitch ,(int16_t)can_send .pluck);break;
		default :
	  Set_Gimbal_Current(&hcan1, 0 ,0 , 0 );break;
	}
}
