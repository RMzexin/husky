#ifndef _SHOOTTASK_H_
#define _SHOOTTASK_H_

#include "stm32f4xx_hal.h"

/***** snail摩擦轮模式相关参数设置 *****/
#define WHEEL_STOP_MODE   1
#define WHEEL_SLOW_MODE   3
#define WHEEL_FAST_MODE   2

#define WHEEL_RAMP_TIME 800

#define WHEEL_STOP_VALUE 1000.0f
#define WHEEL_SLOW_VALUE 1150.0f
#define WHEEL_FAST_VALUE 1200.0f
#define WHEEL_INIT_VALUE 2000.0f

/***** 2006拨弹轮模式相关参数配置 *****/
#define SINGLE_SHOT_MODE  1
#define RUNING_FIRE_MODE  2



void friction_wheel_mode(void);
void shoot_mode(void);
void shoot_init(void);

#endif

