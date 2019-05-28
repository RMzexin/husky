#ifndef __CAN_TASK__
#define __CAN_TASK__

#include "stm32f4xx_hal.h"

typedef enum{
	CAN_3508_M1_ID = 0x201,
	CAN_3508_M2_ID = 0x202,
	CAN_3508_M3_ID = 0x203,
	CAN_3508_M4_ID = 0x204,
	
	CAN_YAW_MOTOR_ID = 0x205,
	CAN_PITCH_MOTOR_ID = 0x206,
	CAN_PLUCK_MOTOR_ID = 0x207,
	
}can_msg_id_e;

typedef struct{
  int32_t ecd_bias;				//初始编码器值
	uint16_t ecd;           //机械角度（编码器不经处理原始值）
  int16_t speed_rpm;      //转速
  int16_t given_current;  //实际转矩电流
  uint8_t temperate;      //电机温度
  int16_t last_ecd;       //上一次机械角度（上一次的编码器原始值）
	int32_t diff;			      //两次编码器之间的差值
	int32_t round_cnt;			//圈数
	int32_t temp_count;     //计数用
	int32_t ecd_value;      //经过处理后连续的编码器值
	float   ecd_angle_bias; //初始角度
	float   ecd_angle;      //角度
}Encoder_t;

extern uint8_t yaw_can_receive;
extern uint8_t pitch_can_receive;
extern uint32_t can_count_reset;
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);
void GetEncoderBias  (volatile Encoder_t *v, CAN_HandleTypeDef * msg);
void Get_3508_Encoder(volatile Encoder_t *v, CAN_HandleTypeDef * msg);
void Get_6020_Encoder(volatile Encoder_t *v, CAN_HandleTypeDef * msg);
void Get_2006_Encoder(volatile Encoder_t *v, CAN_HandleTypeDef * msg);
void Get_6623_Encoder(volatile Encoder_t *v, CAN_HandleTypeDef * msg);
void Set_CM_Speed(CAN_HandleTypeDef* hcan, int16_t cm1_iq,
                 	int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void Set_Gimbal_Current(CAN_HandleTypeDef* hcan, int16_t gimbal_yaw_iq,
                      	int16_t gimbal_pitch_iq, int16_t gimbal_pluck_iq);

#endif


