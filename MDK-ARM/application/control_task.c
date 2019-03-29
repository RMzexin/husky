#include "control_task.h"
#include "remote_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "can_task.h"
#include "imu_task.h"

extern IMU_t IMU;

extern Encoder_t M6623_encoder_yaw,M6623_encoder_pitch,
	               M6020_encoder_yaw,M6020_encoder_pitch,encoder_pluck;

//volatile Angle_t M6623_pitch_angle ,M6623_yaw_angle;
volatile Angle_t M6020_pitch_angle ,M6020_yaw_angle,M2006_angle;

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
					//����������������������ԽǶ�
					M6020_yaw_angle.relative_angle = IMU.yaw - M6020_encoder_yaw.ecd_angle ;
					//pitch��Ƕ����ƣ��ɱ���������У׼������
					M6020_pitch_angle .angle_set += Pitch_Rotate_Data() * PITCH_ROTATE_INC_FACT;
					ANGLE_LIMIT(M6020_pitch_angle .angle_set,M6020_pitch_angle .angle_limit .max ,M6020_pitch_angle .angle_limit .min ) ;
					//yaw��Ƕ����ƣ��ɱ���������У׼������
					M6020_yaw_angle .angle_set += Yaw_Rotate_Data() * YAW_ROTATE_INC_FACT;
					ANGLE_LIMIT(M6020_yaw_angle .angle_set,M6020_yaw_angle .angle_limit .max ,M6020_yaw_angle .angle_limit .min ) ;
					//��̨ ����ģʽ��־λ��ִ����̨����̷����˶�
					chassis_mode = CHASSIS_AUTONOMY;
					gimbal_mode = GIMBAL_ENCODER;
					chassis_behavior();break;
				}
				//ģʽ���������˶�
				case FOLLOWING :
				{
					//pitch��Ƕ����ƣ��ɱ���������У׼������
					M6020_pitch_angle .angle_set += Pitch_Rotate_Data() * PITCH_ROTATE_INC_FACT;
					ANGLE_LIMIT(M6020_pitch_angle .angle_set,M6020_pitch_angle .angle_limit .max ,M6020_pitch_angle .angle_limit .min ) ;
					//yaw��Ƕ����ƣ��ɱ���������У׼������
					M6020_yaw_angle .angle_set += Yaw_Rotate_Data() * YAW_ROTATE_INC_FACT;
					ANGLE_LIMIT(M6020_yaw_angle .angle_set,M6020_yaw_angle .angle_limit .max ,M6020_yaw_angle .angle_limit .min ) ;
					chassis_mode = CHASSIS_AUTONOMY;
					gimbal_mode = GIMBAL_IMU ;
					chassis_behavior();break;
				}
			}
		}
	else 
	{
		gimbal_cali(&M6020_pitch_angle ,&M6020_yaw_angle,&M6020_encoder_yaw ,&M6020_encoder_pitch);
	}	
}

//���ڴ�ѭ����
void chassis_behavior()
{
	switch( chassis_mode )
	{
		case CHASSIS_AUTONOMY :
			chassis_speed .vx = Go_Forward_Data()* GO_FORWARD_INC_FACT ;
		  chassis_speed .wz = Left_Right_Data()* LEFT_RIGHT_INC_FACT ;break;
		default :
			chassis_speed .vx = 0.0f ;
		  chassis_speed .vy = 0.0f ;
	}
	chassis_pid_calc(chassis_speed .vx ,chassis_speed .vy ,chassis_speed .wz);
}

//���ڶ�ʱ����
uint8_t yaw_angle_feedback_set;
uint8_t gimbal_set()
{
	//���л�ģʽ��õ�����ԽǶȼ��������� yaw ���Ư����
//	IMU .yaw_dri = (IMU.yaw - M6020_encoder_yaw.ecd_angle) - M6020_yaw_angle.relative_angle;
	switch ( gimbal_mode )
	{
		case GIMBAL_ENCODER :
			M6020_yaw_angle   .actual_angle = M6020_encoder_yaw   .ecd_angle;
		  M6020_pitch_angle .actual_angle = M6020_encoder_pitch .ecd_angle;
		  M2006_angle .actual_angle       = encoder_pluck .ecd_angle      ;
		  yaw_angle_feedback_set = ENCODER_FEEDBACK;break;
		case GIMBAL_IMU     :
			M6020_yaw_angle   .actual_angle = (IMU .yaw -M6020_yaw_angle.relative_angle) - IMU .yaw_dri;
		  M6020_pitch_angle .actual_angle = M6020_encoder_pitch .ecd_angle;
		  M2006_angle .actual_angle       = encoder_pluck .ecd_angle      ;
		  yaw_angle_feedback_set = IMU_FEEDBACK;break;
		default :
			M6020_yaw_angle   .actual_angle = M6020_encoder_yaw   .ecd_angle;
		  M6020_pitch_angle .actual_angle = M6020_encoder_pitch .ecd_angle;
		  M2006_angle .actual_angle       = encoder_pluck .ecd_angle      ;
		  return 0 ;
	}
	gimbal_pid_calc(&M6020_yaw_angle.angle_set     ,&M6020_pitch_angle .angle_set   ,&M2006_angle .angle_set   ,
	                &M6020_yaw_angle .actual_angle ,&M6020_pitch_angle .actual_angle,&M2006_angle .actual_angle,
	                yaw_angle_feedback_set);
	return 1 ;
}
