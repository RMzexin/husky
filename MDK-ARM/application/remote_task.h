#ifndef __REMOTE_TASK__
#define __REMOTE_TASK__
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#define RC_Frame_Lentgh		18

typedef struct {
	int16_t ch0;	//each ch value from -364 -- +364
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	
	uint8_t switch_left;	//3 value
	uint8_t switch_right;
	
	struct {
		int16_t x;
		int16_t y;
		int16_t z;
	
		uint8_t press_left;
		uint8_t press_right;
	}mouse;
	
	struct {
		uint16_t key_code;
/**********************************************************************************
   * ¼üÅÌÍ¨µÀ:15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
   *          V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
************************************************************************************/

	}keyBoard;
	

}RC_Type;

enum{
	Switch_Up = 1,
	Switch_Middle = 3,
	Switch_Down = 2,
};

extern RC_Type RC_Ctl;
extern uint32_t  Latest_Remote_Control_Pack_Time ;
short int Get_Mode_Data(void);
void RC_Ctl_t_Init(void);
void Callback_RC_Handle(RC_Type* rc, uint8_t* buff);

uint8_t CHOICE_MODE(void);
uint8_t FRICTION_WHEEL_MODE(void);
int16_t Go_Forward_Data(void);
int16_t Left_Right_Data(void);
int16_t Yaw_Rotate_Data(void);
int16_t Pitch_Rotate_Data(void);

#endif



