#include "keymouse_task.h"
#include "control_task.h"
#include "remote_task.h"
#include "ramp.h"

ramp_t Km_forward_ramp    = RAMP_GEN_DAFAULT;
ramp_t Km_backoff_ramp    = RAMP_GEN_DAFAULT;
ramp_t Km_leftshift_ramp  = RAMP_GEN_DAFAULT;
ramp_t Km_rightshift_ramp = RAMP_GEN_DAFAULT;

uint8_t control_mode_selection()
{
	if(RC_Ctl.switch_right == 3&&RC_Ctl.switch_left == 3)
	{
		return KEY_MOUSE_MODE;
	}
	else 
		return REMOTE_CONTROL_MODE;
}


uint8_t CHOICE_MODE(void)
{
	static uint8_t mode_sign = AUTONOMY;
	if(control_mode_selection() == REMOTE_CONTROL_MODE)
	{
		//Ò£¿ØÆ÷²¦¸Ë
		return RC_Ctl.switch_right;
	}
	else if(control_mode_selection() == KEY_MOUSE_MODE)
	{
		if(judge_key_press(KEY_F)&&judge_key_unpress(KEY_G))
		{
			mode_sign = FOLLOWING;
		}
		else if(judge_key_press(KEY_G)&&judge_key_unpress(KEY_F))
		{
			mode_sign = AUTONOMY;
		}
		else if(judge_key_unpress(KEY_G)&&judge_key_unpress(KEY_F)&&judge_key_press(KEY_CTRL))
		{
			return TWISTING;
		}
		return mode_sign;
	}
	return 0 ;
}
uint8_t FRICTION_WHEEL_MODE(void)
{
	if(control_mode_selection() == REMOTE_CONTROL_MODE)
	{
		return RC_Ctl.switch_left;
	}
	else if(control_mode_selection() == KEY_MOUSE_MODE)
	{
		return 0;
	}
	return 0 ;
}
//Ò£¿ØÆ÷Ò¡¸Ë
int16_t Go_Forward_Data(void)
{
	if(control_mode_selection() == REMOTE_CONTROL_MODE)
	{
		return RC_Ctl .ch1 - 0x0400;
	}
	else if(control_mode_selection() == KEY_MOUSE_MODE)
	{
		if(judge_key_press(KEY_W)&&judge_key_unpress(KEY_SHIFT))
		{
			ramp_init(&Km_backoff_ramp, TRANSLATION_RAMP_TIME);
			return FORWARD_PARAMETERS* ramp_calc(&Km_forward_ramp);
		}
		else if(judge_key_press(KEY_S)&&judge_key_unpress(KEY_SHIFT))
		{
			ramp_init(&Km_forward_ramp, TRANSLATION_RAMP_TIME);
			return BACKOFF_PARAMETERS* ramp_calc(&Km_backoff_ramp);
		}
		else if(judge_key_press(KEY_W)&&judge_key_press(KEY_SHIFT))
		{
			ramp_init(&Km_backoff_ramp, TRANSLATION_RAMP_TIME);
			return FORWARD_PARAMETERS*SHIFT_INC_FACT* ramp_calc(&Km_forward_ramp);
		}
		else if(judge_key_press(KEY_S)&&judge_key_press(KEY_SHIFT))
		{
			ramp_init(&Km_forward_ramp, TRANSLATION_RAMP_TIME);
			return BACKOFF_PARAMETERS*SHIFT_INC_FACT* ramp_calc(&Km_backoff_ramp);
		}
		else 
		{
			ramp_init(&Km_forward_ramp, TRANSLATION_RAMP_TIME),
			ramp_init(&Km_backoff_ramp, TRANSLATION_RAMP_TIME);
			return 0;
		}
	}
	return 0 ;
}

int16_t Left_Right_Data(void)
{
	if(control_mode_selection() == REMOTE_CONTROL_MODE)
	{
		return RC_Ctl .ch0 - 0x0400;
	}
	else if(control_mode_selection() == KEY_MOUSE_MODE)
	{
		if(judge_key_press(KEY_A)&&judge_key_unpress(KEY_SHIFT))
		{
			ramp_init(&Km_rightshift_ramp, TRANSLATION_RAMP_TIME);
			return LEFTSHIFT_PARAMETERS* ramp_calc(&Km_leftshift_ramp);
		}
		else if(judge_key_press(KEY_D)&&judge_key_unpress(KEY_SHIFT))
		{
			ramp_init(&Km_leftshift_ramp, TRANSLATION_RAMP_TIME);
			return RIGHTSHIFT_PARAMETERS* ramp_calc(&Km_rightshift_ramp);
		}
		else if(judge_key_press(KEY_A)&&judge_key_press(KEY_SHIFT))
		{
			ramp_init(&Km_rightshift_ramp, TRANSLATION_RAMP_TIME);
			return LEFTSHIFT_PARAMETERS*SHIFT_INC_FACT* ramp_calc(&Km_leftshift_ramp);
		}
		else if(judge_key_press(KEY_D)&&judge_key_press(KEY_SHIFT))
		{
			ramp_init(&Km_leftshift_ramp, TRANSLATION_RAMP_TIME);
			return RIGHTSHIFT_PARAMETERS*SHIFT_INC_FACT* ramp_calc(&Km_rightshift_ramp);
		}
		else 
		{
			ramp_init(&Km_leftshift_ramp , TRANSLATION_RAMP_TIME),
			ramp_init(&Km_rightshift_ramp, TRANSLATION_RAMP_TIME);
			return 0 ;
		}
	}
	return 0 ;
}

int16_t Yaw_Rotate_Data(void)
{
	if(control_mode_selection() == REMOTE_CONTROL_MODE)
	{
		return RC_Ctl .ch2 - 0x0400;
	}
	else if(control_mode_selection() == KEY_MOUSE_MODE)
	{
		return RC_Ctl .mouse .x *MOUSE_YAW_INC_FACT ;
	}
	return 0 ;
}

int16_t Pitch_Rotate_Data(void)
{
	if(control_mode_selection() == REMOTE_CONTROL_MODE)
	{
		return RC_Ctl .ch3 - 0x0400;
	}
	else if(control_mode_selection() == KEY_MOUSE_MODE)
	{
		return RC_Ctl .mouse .y *MOUSE_PITCH_INC_FACT ;
	}
	return 0 ;
}

//ÅÐ¶Ï°´¼üÒÑ°´ÏÂ
uint8_t judge_key_press(uint16_t key)
{
	if(RC_Ctl .keyBoard .key_code & key )
		return 1 ;
	else return 0 ;
}

//ÅÐ¶Ï°´¼üÎ´°´ÏÂ
uint8_t judge_key_unpress(uint16_t key)
{
	if(RC_Ctl .keyBoard .key_code & key )
		return 0 ;
	else return 1 ;
}
