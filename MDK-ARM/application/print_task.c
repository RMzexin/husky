#include "print_task.h"
#include "usart.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "control_task.h"
#include "imu_task.h"
#include "can_task.h"
#include "pid.h"
//����3�����ó�DMA��������ʹ�ò������������޸�
extern Encoder_t M6020_encoder_yaw,M6020_encoder_pitch,
                 encoder_pluck;
extern Angle_t M6020_pitch_angle ,M6020_yaw_angle,M2006_angle,chassis_angle;
extern Gimbal_Motor_t yaw_gimbal_motor,pitch_gimbal_motor,pluck_motor;
extern pid_t M6020_yaw_speed_pid,M6020_pitch_speed_pid,M6020_yaw_angle_pid,M6020_pitch_angle_pid,
	     M2006_speed_pid          , M2006_angle_pid     ,IMU_yaw_speed_pid  , IMU_yaw_angle_pid   ;
extern IMU_t IMU_chassis , IMU_gimbal;
extern pid_t M3508_motor_speed_pid[4],M3508_twisting_pid[4],chassis_twisting_correct_pid,chassis_following_correct_pid;

float a,b,c;
//��.c�ļ���Ҫ�����ɽ��������ַ��Ͳ���
unsigned char  wave_form_data[24] = {0};
void send_data(uint8_t date)
{
	HAL_UART_Transmit(&huart3,&date,1,10);
	//while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);  
	
}
void shanwai_send_wave_form(void)
{
	uint8_t i;
	
	send_data(0x03);
	send_data(0xfc);
	for(i = 0;i<24;i++)
	{
	  send_data((wave_form_data[i]&0xff));
	}
	send_data(0xfc);
	send_data(0x03);
}	
void shanwai_sprintf()
{
	a = M6020_yaw_angle.angle_set-M6020_encoder_yaw.ecd_angle;
	b = M6020_encoder_yaw.ecd_angle+IMU_chassis.C_yaw;
	Float_to_Byte(&IMU_gimbal.C_yaw,wave_form_data,0);
	Float_to_Byte(&IMU_chassis.C_yaw,wave_form_data,4);
	Float_to_Byte(&a,wave_form_data,8);
	Float_to_Byte(&b,wave_form_data,12);
	Float_to_Byte(&IMU_yaw_angle_pid .pidout,wave_form_data,16);
	Float_to_Byte(&M6020_yaw_angle .angle_limit .middle,wave_form_data,20);
	shanwai_send_wave_form();   //�����ݴ��䵽������λ�������Կ���ʵʱ����
}

//float ת�� 4���ֽ�
void Float_to_Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}
