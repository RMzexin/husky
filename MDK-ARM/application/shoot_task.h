#ifndef _SHOOTTASK_H_
#define _SHOOTTASK_H_

#include "stm32f4xx_hal.h"

/***** snail摩擦轮模式相关参数设置 *****/

#define WHEEL_STOP_MODE   1
#define WHEEL_SLOW_MODE   2
#define WHEEL_FAST_MODE   3

#define PWM_UPDATE_INTERVAL  200 //PWM更新时间 ms
#define WHEEL_RAMP_TIME       10 //斜坡执行次数

#define WHEEL_STOP_VALUE 1000.0f
#define WHEEL_SLOW_VALUE 1120.0f
#define WHEEL_FAST_VALUE 1230.0f
#define WHEEL_INIT_VALUE 2000.0f

/***** 2006拨弹轮模式相关参数配置 *****/
#define PLUCK_RATIO                36.0f      //拨弹轮减速比
#define LATTICE_NUMBER              7.0f      //拨弹轮盘格数
#define LAUNCH_STOP              0
#define SLOW_FREQUENCY           1
#define HIGH_FREQUENCY           2
#define SLOW_FREQUENCY_INTERVAL  300      //低频连续发弹间隔时间 ms
#define HIGH_FREQUENCY_INTERVAL  150      //高频发弹间隔时间 ms

#define SINGLE_SHOT_MODE     1
#define RUNING_FIRE_MODE     2



void friction_wheel_mode(void);
float pluck_angle_add(void);
void shoot_mode(void);
void shoot_init(void);

#endif

