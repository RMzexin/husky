#include "control_task.h"
#include "keymouse_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "can_task.h"
#include "imu_task.h"
#include "mains.h"
#include <stdbool.h>
#include <stdlib.h>

extern IMU_t IMU;

extern Encoder_t M6623_encoder_yaw,M6623_encoder_pitch,
	               M6020_encoder_yaw,M6020_encoder_pitch,encoder_pluck;

//volatile Angle_t M6623_pitch_angle ,M6623_yaw_angle;
volatile Angle_t M6020_pitch_angle ,M6020_yaw_angle,M2006_angle,chassis_angle;

chassis_speed_t chassis_speed;

uint8_t chassis_mode , gimbal_mode;
volatile const uint8_t Gimbal_Cali_Complete = 0 ;

//�Ƕ�����������
#define ANGLE_LIMIT(angle,angle_max,angle_min)        \
{                                                     \
	if(angle > angle_max){                              \
		angle = angle_max;                                \
	}else if(angle<angle_min){                          \
	  angle = angle_min;}                               \
}

//�ٶ�����������
#define SPEED_LIMIT(speed,speed_max,speed_min)        \
{                                                     \
	if(speed > speed_max){                              \
		return speed_max;                                 \
	}else if(speed<angle_min){                          \
	  return speed_min;                                 \
	}else {return speed;}                               \
}

//���ڴ�ѭ����
void gimbal_change(void)
{
	if(Gimbal_Cali_Complete)
		{
			switch ( CHOICE_MODE() )
			{
				//ģʽһ �������˶�
				case AUTONOMY :
				{
					M6020_yaw_angle .add_angle = 0.0f;
					//����������������������ԽǶ�,ȷ��yaw��̶��Ƕ�
					M6020_yaw_angle.relative_angle = M6020_encoder_yaw.ecd_angle - IMU.C_yaw ;
					M6020_yaw_angle.yaw_fix_set    = IMU.C_yaw;
					//pitch������ �Ƕ����ƣ��ɱ���������У׼������
					M6020_pitch_angle .angle_set += Pitch_Rotate_Data() * PITCH_ROTATE_INC_FACT;
					ANGLE_LIMIT(M6020_pitch_angle .angle_set,M6020_pitch_angle .angle_limit .max ,M6020_pitch_angle .angle_limit .min ) ;
					//yaw��Ƕ����� ���ƣ��ɱ���������У׼������
					M6020_yaw_angle .angle_set   -= Yaw_Rotate_Data() * YAW_ROTATE_INC_FACT;
					ANGLE_LIMIT(M6020_yaw_angle .angle_set,M6020_yaw_angle .angle_limit .max ,M6020_yaw_angle .angle_limit .min ) ;
					//��̨ ����ģʽ��־λ��ִ����̨����̷����˶�
					chassis_mode = CHASSIS_AUTONOMY;
					gimbal_mode  = GIMBAL_ENCODER;
					chassis_behavior();break;
				}
				//ģʽ���������˶�
				case FOLLOWING :
				{
					//pitch������ �Ƕ����ƣ��ɱ���������У׼������
					M6020_pitch_angle .angle_set += Pitch_Rotate_Data() * PITCH_ROTATE_INC_FACT;
					ANGLE_LIMIT(M6020_pitch_angle .angle_set,M6020_pitch_angle .angle_limit .max ,M6020_pitch_angle .angle_limit .min ) ;
					//������ƫ���趨�Ƕȣ���̨�Ƕ�����
					M6020_yaw_angle .add_angle   -= Yaw_Rotate_Data() * YAW_ROTATE_INC_FACT;
					M6020_yaw_angle .angle_set = Correct_Angle_Feedback();
					ANGLE_LIMIT(M6020_yaw_angle .angle_set,M6020_yaw_angle .angle_limit .max ,M6020_yaw_angle .angle_limit .min ) ;
					//��̨ ����ģʽ��־λ��ִ����̨����̸����˶�
					chassis_mode = CHASSIS_FOLLOWING;
					gimbal_mode  = GIMBAL_IMU ;
					chassis_behavior();break;
				}
				//ģʽ����Ť���˶�
				case TWISTING :
				{
					//pitch������ �Ƕ����ƣ��ɱ���������У׼������
					M6020_pitch_angle .angle_set += Pitch_Rotate_Data() * PITCH_ROTATE_INC_FACT;
					ANGLE_LIMIT(M6020_pitch_angle .angle_set,M6020_pitch_angle .angle_limit .max ,M6020_pitch_angle .angle_limit .min);
					//��������ƫ���趨�Ƕȣ���̨�����Ƕ�
					M6020_yaw_angle .add_angle   -= Yaw_Rotate_Data() * YAW_ROTATE_INC_FACT;
					M6020_yaw_angle .angle_set = Correct_Angle_Feedback();
					ANGLE_LIMIT(M6020_yaw_angle .angle_set,M6020_yaw_angle .angle_limit .max ,M6020_yaw_angle .angle_limit .min);
					//��̨ ����ģʽ��־λ��ִ����̨�̶� ����Ť���˶�
					chassis_mode = CHASSIS_TWISTING;
					gimbal_mode  = GIMBAL_IMU ;
					chassis_behavior();break;
				}
			}
		}
	else 
	{
		//��̨У׼ �Ƕȳ�ʼ��
		gimbal_cali(&M6020_pitch_angle ,&M6020_yaw_angle,&M2006_angle,&M6020_encoder_yaw ,&M6020_encoder_pitch,&encoder_pluck);
	}	
}
//PC�Ƕ�����
void gimbal_PC_correct(int32_t *PC_yaw_add,int32_t*PC_pitch_add)
{
	M6020_pitch_angle .angle_set += *PC_pitch_add;
	ANGLE_LIMIT(M6020_pitch_angle .angle_set,M6020_pitch_angle .angle_limit .max ,M6020_pitch_angle .angle_limit .min);
	M6020_yaw_angle .angle_set   += *PC_yaw_add  ;
	ANGLE_LIMIT(M6020_yaw_angle .angle_set,M6020_yaw_angle .angle_limit .max ,M6020_yaw_angle .angle_limit .min);
}
//IMU�Ƕ�����������IMU�����
float Correct_Angle_Feedback()
{
	return  M6020_yaw_angle.yaw_fix_set+M6020_yaw_angle.relative_angle-(IMU.C_yaw - M6020_yaw_angle.yaw_fix_set)+M6020_yaw_angle .add_angle;
}

//���ڴ�ѭ����
bool static chassis_max_get = GET_INITIAL ;
bool static chassis_min_get = GET_INITIAL ;
static uint8_t chassis_mode_calc_set;
void chassis_behavior()
{
	switch( chassis_mode )
	{
		case CHASSIS_AUTONOMY :
		{
			//ң���Ҳ��˿���ǰ������ƽ�ƣ��󲦸˿�����̨
			chassis_speed .vx = Go_Forward_Data()* GO_FORWARD_INC_FACT ;
		  chassis_speed .vy = 0.0f ;
		  chassis_speed .wz = Left_Right_Data()* LEFT_RIGHT_INC_FACT;
			chassis_mode_calc_set = AUTONOMY ;break;
		}
		case CHASSIS_FOLLOWING :
		{
			chassis_speed .vx = Go_Forward_Data()* GO_FORWARD_INC_FACT ;
		  chassis_speed .vy = Left_Right_Data()* LEFT_RIGHT_INC_FACT ;
		  chassis_speed .wz = Follow_Speed_Calc(&M6020_encoder_yaw.ecd_angle,&M6020_yaw_angle .angle_limit .middle);
			chassis_mode_calc_set = AUTONOMY ;break;
		}
		/***** �����Զ�Ť���˶� *****/
		case CHASSIS_TWISTING :
		{
			M6020_yaw_angle.correct_angle = M6020_encoder_yaw.ecd_angle - M6020_yaw_angle .angle_limit .middle;
			//�����ж�����жϵ��̴ﵽָ���Ƕȼ���ת
			if((M6020_encoder_yaw.ecd_angle - M6020_yaw_angle .angle_limit .middle) < chassis_angle .angle_set)
			{
				//��ȡ���ת���Ƕȣ��жϱ�־λ����ת��ִ��һ�Σ�
				if(chassis_min_get == true )
				{
					chassis_angle .angle_set = (M6020_yaw_angle .angle_limit .middle  - M6020_yaw_angle .angle_limit .min ) *((rand()%100000)*0.00001f)*TWISTING_INC_FACT;
				  chassis_min_get = GET_COMPLETE  ;
					chassis_max_get = GET_INITIAL   ;
				}
				//PID���������ת�ٶȣ��Ƕȷ�����
				chassis_speed .wz = -Twist_Speed_Calc(&chassis_angle .angle_set,&M6020_yaw_angle.correct_angle);
			}
			//ͬ��
			else if((M6020_encoder_yaw.ecd_angle - M6020_yaw_angle .angle_limit .middle) > chassis_angle .angle_set)
			{
				if(chassis_max_get == true )
				{
					chassis_angle .angle_set = (M6020_yaw_angle .angle_limit .middle  - M6020_yaw_angle .angle_limit .max ) *((rand()%100000)*0.00001f)*TWISTING_INC_FACT;
				  chassis_max_get = GET_COMPLETE ;
					chassis_min_get = GET_INITIAL   ;
				}
				chassis_speed .wz = -Twist_Speed_Calc(&chassis_angle .angle_set,&M6020_yaw_angle.correct_angle);
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

//���ڶ�ʱ���������̨PID���㣩
static uint8_t yaw_angle_feedback_set;
uint8_t gimbal_set()
{
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
