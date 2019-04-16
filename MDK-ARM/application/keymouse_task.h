#ifndef __KEYMOUSE_TASK__
#define __KEYMOUSE_TASK__

#include "stm32f4xx_hal.h"

#define REMOTE_CONTROL_MODE   1
#define KEY_MOUSE_MODE        2

#define KEY_V		  0x4000
#define KEY_C		  0x2000
#define KEY_X		  0x1000
#define KEY_Z		  0x0800
#define KEY_G		  0x0400
#define KEY_F		  0x0200
#define KEY_R		  0x0100
#define KEY_E		  0x0080
#define KEY_Q		  0x0040
#define KEY_CTRL	0x0020
#define KEY_SHIFT	0x0010
#define KEY_D		  0x0008
#define KEY_A		  0x0004
#define KEY_S		  0x0002
#define KEY_W		  0x0001

#define KEY_JITTER_TIME         100

//加速斜坡时间
#define TRANSLATION_RAMP_TIME         1500
//W S A D 原始值
#define FORWARD_PARAMETERS             540
#define BACKOFF_PARAMETERS            -540
#define LEFTSHIFT_PARAMETERS          -440
#define RIGHTSHIFT_PARAMETERS          440
//加速
#define SHIFT_INC_FACT                1.55
//鼠标移动灵敏度
#define MOUSE_YAW_INC_FACT            10
#define MOUSE_PITCH_INC_FACT         -10
//射速射频调节间隔
#define FIRING_TATE_TIME              2000//2000MS


uint8_t control_mode_selection(void);
uint8_t CHOICE_MODE(void);
uint8_t FRICTION_WHEEL_MODE(void);
int16_t Go_Forward_Data(void);
int16_t Left_Right_Data(void);
int16_t Yaw_Rotate_Data(void);
int16_t Pitch_Rotate_Data(void);
uint8_t pc_aim(void);
uint8_t launch(void);
uint8_t firing_tate(void);
uint8_t judge_key_press(uint16_t key);
uint8_t judge_key_unpress(uint16_t key);

#endif
