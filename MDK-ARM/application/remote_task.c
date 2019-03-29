#include "remote_task.h"

RC_Type   RC_Ctl;
uint32_t  Latest_Remote_Control_Pack_Time = 0;
uint32_t  LED_Flash_Timer_remote_control = 0;
/*******************************************************************************************
  * @Func		void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
  * @Brief  DR16接收机协议解码程序
  * @Param		RC_Type* rc　存储遥控器数据的结构体　　uint8_t* buff　用于解码的缓存
  * @Retval		None
  * @Date    
 *******************************************************************************************/
void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
{
	rc->ch0 = (buff[0] | buff[1]<<8) & 0x07FF;
	rc->ch1 = (buff[1]>>3 | buff[2]<<5 ) & 0x07FF;
	rc->ch2 = (buff[2]>>6 | buff[3]<<2 | buff[4]<<10) & 0x07FF;
	rc->ch3 = (buff[4]>>1 | buff[5]<<7) & 0x07FF;		
	
	rc->switch_left = ( (buff[5] >> 4)& 0x000C ) >> 2;
	rc->switch_right =( (buff[5] >> 4)& 0x0003 ) ;
	
	rc->mouse.x = buff[6] | (buff[7] << 8);	//x axis
	rc->mouse.y = buff[8] | (buff[9] << 8);
	rc->mouse.z = buff[10]| (buff[11] << 8);
	
	rc->mouse.press_left 	= buff[12];	// is pressed
	rc->mouse.press_right = buff[13];
	
	rc->keyBoard.key_code = buff[14] | buff[15] << 8; //key borad code
	
	Latest_Remote_Control_Pack_Time = HAL_GetTick();
	
	if(Latest_Remote_Control_Pack_Time - LED_Flash_Timer_remote_control>500){
			LED_Flash_Timer_remote_control = Latest_Remote_Control_Pack_Time;		
	}
}

void RC_Ctl_t_Init(void)	//对遥控器按键进行初始化
{
	RC_Ctl.ch0 = 1024;		//ch0和ch1为右摇杆
	RC_Ctl.ch1 = 1024;	
	RC_Ctl.ch2 = 1024;		//ch2和ch3为左摇杆
	RC_Ctl.ch3 = 1024;
	RC_Ctl.switch_left = 0;			//左上推杆
	RC_Ctl.switch_right = 0;		//右上推杆		
	RC_Ctl.mouse.press_left = 0;		//鼠标左键
	RC_Ctl.mouse.press_right = 0;		//鼠标右键
	RC_Ctl.mouse.x = 0;	//鼠标在X轴的移动速度
	RC_Ctl.mouse.y = 0;	//鼠标在Y轴的移动速度
	RC_Ctl.mouse.z = 0;	//鼠标在Z轴的移动速度
	RC_Ctl.keyBoard.key_code = 0;
}
//遥控器拨杆
uint8_t CHOICE_MODE(void){
	return RC_Ctl.switch_right;}
uint8_t FRICTION_WHEEL_MODE(void){
	return RC_Ctl.switch_left;}
//遥控器摇杆
int16_t Go_Forward_Data(void){
	return RC_Ctl .ch1 - 0x0400;}

int16_t Left_Right_Data(void){
	return RC_Ctl .ch0 - 0x0400;}

int16_t Yaw_Rotate_Data(void){
	return RC_Ctl .ch2 - 0x0400;}

int16_t Pitch_Rotate_Data(void){
	return RC_Ctl .ch3 - 0x0400;}

	


