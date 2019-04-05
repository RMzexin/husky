#ifndef _SHOOTTASK_H_
#define _SHOOTTASK_H_

#include "stm32f4xx_hal.h"

/***** snailĦ����ģʽ��ز������� *****/

#define WHEEL_STOP_MODE   1
#define WHEEL_SLOW_MODE   3
#define WHEEL_FAST_MODE   2

#define PWM_UPDATE_INTERVAL  200 //PWM����ʱ�� ms
#define WHEEL_RAMP_TIME       10 //б��ִ�д���

#define WHEEL_STOP_VALUE 1000.0f
#define WHEEL_SLOW_VALUE 1150.0f
#define WHEEL_FAST_VALUE 1200.0f
#define WHEEL_INIT_VALUE 2000.0f

/***** 2006������ģʽ��ز������� *****/
#define PLUCK_RATIO                36.0f      //�����ּ��ٱ�
#define LATTICE_NUMBER              9.0f      //�������̸���
#define CONTINUOUS_FIRING_INTERVAL  100      //�����������ʱ�� ms

#define SINGLE_SHOT_MODE     1
#define RUNING_FIRE_MODE     2



void friction_wheel_mode(void);
float pluck_angle_add(void);
void shoot_mode(void);
void shoot_init(void);

#endif

