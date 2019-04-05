#ifndef __CONTROL_TASK__
#define __CONTROL_TASK__
#include "stm32f4xx_hal.h"

/***** ң����ģʽ������� *****/
//��ģʽ�л�
#define AUTONOMY   1
#define FOLLOWING  3
#define TWISTING   2
//����ģʽ�л�
#define CHASSIS_AUTONOMY   1
#define CHASSIS_FOLLOWING  2
#define CHASSIS_TWISTING   3
//��̨ģʽ�л�
#define GIMBAL_ENCODER     1
#define GIMBAL_IMU         2
//С������ת�ǶȻ��
#define GET_COMPLETE  false
#define GET_INITIAL   true
#define TWISTING_INC_FACT   0.7428f  //С�����޷�ϵ��

//�����ٶȵ���
#define GO_FORWARD_INC_FACT        0.22f //ң����ǰ��ͨ��ϵ��
#define LEFT_RIGHT_INC_FACT        0.22f //ң����ƽ��ͨ��ϵ��
//��̨�����ȵ���
#define YAW_ROTATE_INC_FACT    0.00028f //ң������̨yawͨ��ϵ��
#define PITCH_ROTATE_INC_FACT  0.00028f //ң������̨pitchͨ��ϵ��

//��̨�ǶȽṹ�嶨��
typedef struct 
{
	float last_angle;
	float max;
	float min;
	float middle;
}Angle_Limit_t;

typedef struct 
{
	Angle_Limit_t angle_limit; //�Ƕ�����
	float angle_set;           //�����Ƕ�
	float actual_angle;        //ʵ�ʽǶ�
	
	float relative_angle;      //��ԽǶ�
	float yaw_fix_set;         //yaw��̶��Ƕ�
	float correct_angle;       //�����Ƕ�
	float add_angle;           //����ʱ���ӵĽǶ�
	
	
}Angle_t;
 
//�����ٶȽṹ�嶨��
typedef struct
{
	float vx;
	float vy;
	float wz;
}chassis_speed_t;

void gimbal_change(void);
void chassis_behavior(void);
uint8_t  gimbal_set(void);
float Correct_Angle_Feedback(void);
void gimbal_PC_correct(int32_t*PC_yaw_add,int32_t*PC_pitch_add);
#endif
