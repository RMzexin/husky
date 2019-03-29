#include "can.h"
#include "can_task.h"
#include "cmsis_os.h"

static uint32_t can_count = 0;
volatile Encoder_t M6623_encoder_yaw={0}, M6623_encoder_pitch={0},M6020_encoder_yaw={0}, M6020_encoder_pitch={0},
                   encoder_pluck={0}, encoder_chassis[4]={0}; 

extern osTimerId CanTimerSendHandle;

/*******************************************************************************************
  * @Func		my_can_filter_init
  * @Brief    CAN1和CAN2滤波器配置
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2015/11/30
 *******************************************************************************************/
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
	//can1 &can2 use same filter config
	CAN_FilterConfTypeDef		CAN_FilterConfigStructure;
	static CanTxMsgTypeDef		Tx1Message;
	static CanRxMsgTypeDef 		Rx1Message;


	CAN_FilterConfigStructure.FilterNumber = 0;
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN_FilterConfigStructure.BankNumber = 14;//can1(0-13)和can2(14-27)分别得到一半的filter
	CAN_FilterConfigStructure.FilterActivation = ENABLE;

	if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
	{
		//err_deadloop(); //show error!
	}


	if(_hcan == &hcan1){
		_hcan->pTxMsg = &Tx1Message;
		_hcan->pRxMsg = &Rx1Message;
	}
}

//保存初始编码器值
void GetEncoderBias(volatile Encoder_t *v, CAN_HandleTypeDef*_hcan)
{
	v->ecd_bias = (_hcan->pRxMsg->Data[0]<<8)|_hcan->pRxMsg->Data[1];  //保存初始编码器值作为偏差  
	v->ecd      = v->ecd_bias;
  v->last_ecd = v->ecd_bias;
  v->temp_count++;
	v->ecd_angle_bias = (float)(v->ecd_bias)*360/8192;
}

/*****        3508电机读取         *****/
/*****     速度环反馈 speed_rpm    *****/
void Get_3508_Encoder(volatile Encoder_t *v, CAN_HandleTypeDef*_hcan)
{
	v->last_ecd      = v->ecd;
	v->ecd           = (_hcan->pRxMsg->Data[0]<<8)|_hcan->pRxMsg->Data[1];   
	if(_hcan->pRxMsg->Data[2] & 128){		
		v->speed_rpm =(~(65535-((_hcan->pRxMsg->Data[2]<<8)|_hcan->pRxMsg->Data[3]))/19);
	}else{
		v->speed_rpm =(((_hcan->pRxMsg->Data[2]<<8)|_hcan->pRxMsg->Data[3])/19);}
  v->given_current = (_hcan->pRxMsg->Data[4]<<8)|_hcan->pRxMsg->Data[5]; 
  v->temperate     = (_hcan->pRxMsg->Data[6]);
}

/*****        6020电机读取         *****/
void Get_6020_Encoder(volatile Encoder_t *v, CAN_HandleTypeDef*_hcan)
{
	v->last_ecd      = v->ecd;
	v->ecd           = (_hcan->pRxMsg->Data[0]<<8)|_hcan->pRxMsg->Data[1];  
	if(_hcan->pRxMsg->Data[2] & 128){		
		v->speed_rpm =(~(65535-((_hcan->pRxMsg->Data[2]<<8)|_hcan->pRxMsg->Data[3])));
	}else{
		v->speed_rpm =(((_hcan->pRxMsg->Data[2]<<8)|_hcan->pRxMsg->Data[3]));}
  v->given_current = (_hcan->pRxMsg->Data[4]<<8)|_hcan->pRxMsg->Data[5]; 
  v->temperate     = (_hcan->pRxMsg->Data[6]);
	v->diff = v->ecd - v->last_ecd;
		if(v->diff < -7500)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
	}
	else if(v->diff>7500)
	{
		v->round_cnt--;
	}		

	//计算得到连续的编码器输出值
	v->ecd_value = v->ecd + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->ecd - v->ecd_bias)*360/8192 + v->round_cnt * 360;
}

/*****        2006电机读取         *****/
void Get_2006_Encoder(volatile Encoder_t *v, CAN_HandleTypeDef*_hcan)
{
	v->last_ecd      = v->ecd;
	v->ecd           = (_hcan->pRxMsg->Data[0]<<8)|_hcan->pRxMsg->Data[1];  
	if(_hcan->pRxMsg->Data[2] & 128){		
		v->speed_rpm =(~(65535-((_hcan->pRxMsg->Data[2]<<8)|_hcan->pRxMsg->Data[3]))/19);
	}else{
		v->speed_rpm =(((_hcan->pRxMsg->Data[2]<<8)|_hcan->pRxMsg->Data[3])/19);}
  v->given_current = (_hcan->pRxMsg->Data[4]<<8)|_hcan->pRxMsg->Data[5]; 
  v->temperate     = (_hcan->pRxMsg->Data[6]);
	v->diff = v->ecd - v->last_ecd;
		if(v->diff < -7500)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
	}
	else if(v->diff>7500)
	{
		v->round_cnt--;
	}		

	//计算得到连续的编码器输出值
	v->ecd_value = v->ecd + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->ecd - v->ecd_bias)*360/8192 + v->round_cnt * 360;
}

/*****        6623电机读取         *****/
void Get_6623_Encoder(volatile Encoder_t *v, CAN_HandleTypeDef*_hcan)
{
	v->last_ecd      = v->ecd;
	v->ecd           = (_hcan->pRxMsg->Data[0]<<8)|_hcan->pRxMsg->Data[1]; 
  v->given_current = (_hcan->pRxMsg->Data[2]<<8)|_hcan->pRxMsg->Data[3];
  v->diff = v->ecd - v->last_ecd;
		if(v->diff < -7500)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
	}
	else if(v->diff>7500)
	{
		v->round_cnt--;
	}	
	
	//计算得到连续的编码器输出值
	v->ecd_value = v->ecd + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->ecd - v->ecd_bias)*360/8192 + v->round_cnt * 360;	
}

uint32_t FlashTimer;
/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    HAL库中标准的CAN接收完成回调函数，需要在此处理通过CAN总线接收到的数据
  * @Param		
  * @Retval		None 
  * @Date     2015/11/24
 *******************************************************************************************/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
   can_count++;
	if(HAL_GetTick() - FlashTimer>500){
		FlashTimer = HAL_GetTick();	
	}

	switch(_hcan->pRxMsg->StdId)
		{
		case CAN_3508_M1_ID:
	  case CAN_3508_M2_ID:
		case CAN_3508_M3_ID:
		case CAN_3508_M4_ID:
			{
				static uint8_t i = 0;
        //处理电机ID号
        i = _hcan->pRxMsg->StdId - CAN_3508_M1_ID;
        //处理电机数据宏函数
				Get_3508_Encoder(&encoder_chassis[i] ,_hcan);   
			}break;
		case CAN_YAW_MOTOR_ID:
			{
			  (can_count<=50) ? GetEncoderBias(&M6623_encoder_yaw ,_hcan):Get_6020_Encoder(&M6020_encoder_yaw ,_hcan);				
			}break;
		case CAN_PITCH_MOTOR_ID:
			{
				(can_count<=50) ? GetEncoderBias(&M6623_encoder_pitch ,_hcan):Get_6020_Encoder(&M6020_encoder_pitch ,_hcan);
			}break;	
		case CAN_PLUCK_MOTOR_ID:
			{
				(can_count<=50) ? GetEncoderBias(&encoder_pluck ,_hcan):Get_2006_Encoder(&encoder_pluck ,_hcan);			
			}break;				
		}
/*#### add enable can it again to solve can receive only one ID problem!!!####**/
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
}


/********************************************************************************
   给底盘电调板发送指令，ID号为0x200８底盘返回ID为0x201-0x204
*********************************************************************************/
void Set_CM_Speed(CAN_HandleTypeDef* hcan, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq){

	hcan->pTxMsg->StdId = 0x200;
	hcan->pTxMsg->IDE = CAN_ID_STD;
	hcan->pTxMsg->RTR = CAN_RTR_DATA;
	hcan->pTxMsg->DLC = 0x08;
	hcan->pTxMsg->Data[0] = (cm1_iq >> 8);
	hcan->pTxMsg->Data[1] =  cm1_iq;
	hcan->pTxMsg->Data[2] = (cm2_iq >> 8);
	hcan->pTxMsg->Data[3] =  cm2_iq;
	hcan->pTxMsg->Data[4] = (cm3_iq >> 8);
	hcan->pTxMsg->Data[5] =  cm3_iq;
	hcan->pTxMsg->Data[6] = (cm4_iq >> 8);
	hcan->pTxMsg->Data[7] =  cm4_iq;
	
	HAL_CAN_Transmit(hcan, 100);
}
/********************************************************************************
   给电调板发送指令，ID号为0x1FF，用三个个电调板，数据回传ID为0x205和0x206和0x207
	 cyq:更改为发送三个电调的指令。
*********************************************************************************/
void Set_Gimbal_Current(CAN_HandleTypeDef* hcan, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t gimbal_pluck_iq){
	
	hcan->pTxMsg->StdId = 0x1FF;
	hcan->pTxMsg->IDE = CAN_ID_STD;
	hcan->pTxMsg->RTR = CAN_RTR_DATA;
	hcan->pTxMsg->DLC = 0x08;
	hcan->pTxMsg->Data[0] = (gimbal_yaw_iq >> 8);
	hcan->pTxMsg->Data[1] =  gimbal_yaw_iq;
	hcan->pTxMsg->Data[2] = (gimbal_pitch_iq >> 8);
	hcan->pTxMsg->Data[3] =  gimbal_pitch_iq;
	hcan->pTxMsg->Data[4] = (gimbal_pluck_iq >> 8);
	hcan->pTxMsg->Data[5] =  gimbal_pluck_iq;
	
	HAL_CAN_Transmit(hcan, 100);
}
