#include "shoot_task.h"
#include "keymouse_task.h"
#include "remote_task.h"
#include "Driver_Judge.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ramp.h"
#include "tim.h"
#include <stdbool.h>

ramp_t Fr_stop_ramp = RAMP_GEN_DAFAULT;
ramp_t Fr_slow_ramp = RAMP_GEN_DAFAULT;
ramp_t Fr_fast_ramp = RAMP_GEN_DAFAULT;

float   wheel_value = WHEEL_INIT_VALUE;
uint8_t launch_frequency;

void shoot_init()
{
	ramp_init(&Fr_stop_ramp, WHEEL_RAMP_TIME);
	ramp_init(&Fr_slow_ramp, WHEEL_RAMP_TIME);
	ramp_init(&Fr_fast_ramp, WHEEL_RAMP_TIME);
	//snail电机比较傲娇，初始化稍有偏差就工作不正常，你们要用得用示波器去调，配合电调调试器
	motor_pwm_setvalue1(wheel_value);
	HAL_Delay(3000);
	for(int i = 0;i < 10;i++)
	{
		wheel_value -=100.0f;
		motor_pwm_setvalue1(wheel_value);
		HAL_Delay(100);
	}
}

uint8_t firing_indicator_light = 0;
//这里是调摩擦轮转速，这么多句就是因为垃圾snail
void friction_wheel_mode()
{
	static uint32_t system_runtime;
  static uint32_t last_system_runtime;
  system_runtime = xTaskGetTickCount();
	switch (firing_tate())
	{
		case WHEEL_STOP_MODE :
			if(system_runtime - last_system_runtime >= PWM_UPDATE_INTERVAL )
				{
					last_system_runtime = system_runtime ;
					wheel_value += (WHEEL_STOP_VALUE - wheel_value) * ramp_calc(&Fr_stop_ramp);
					if( wheel_value != WHEEL_STOP_VALUE)
						{
							motor_pwm_setvalue1(wheel_value);
						}
				}
				firing_indicator_light = 1;
				launch_frequency = LAUNCH_STOP;
				ramp_init(&Fr_slow_ramp, WHEEL_RAMP_TIME),ramp_init(&Fr_fast_ramp, WHEEL_RAMP_TIME);break;
		case WHEEL_SLOW_MODE :
			if(system_runtime - last_system_runtime >= PWM_UPDATE_INTERVAL )
				{
					last_system_runtime = system_runtime ;
					wheel_value += (WHEEL_SLOW_VALUE - wheel_value) * ramp_calc(&Fr_slow_ramp);
					if( wheel_value != WHEEL_SLOW_VALUE)
						{
							motor_pwm_setvalue1(wheel_value);
						}
				}
				firing_indicator_light = 3;
				launch_frequency = HIGH_FREQUENCY;
				ramp_init(&Fr_stop_ramp, WHEEL_RAMP_TIME),ramp_init(&Fr_fast_ramp, WHEEL_RAMP_TIME);break;
		case WHEEL_FAST_MODE :
			if(system_runtime - last_system_runtime >= PWM_UPDATE_INTERVAL )
				{
					last_system_runtime = system_runtime ;
					wheel_value += (WHEEL_FAST_VALUE - wheel_value) * ramp_calc(&Fr_fast_ramp);
					if( wheel_value != WHEEL_FAST_VALUE)
						{
							motor_pwm_setvalue1(wheel_value);
						}			
				}
				firing_indicator_light = 7;
				launch_frequency = SLOW_FREQUENCY;
				ramp_init(&Fr_stop_ramp, WHEEL_RAMP_TIME),ramp_init(&Fr_slow_ramp, WHEEL_RAMP_TIME);break;	
			}
}

bool static single_launch   = true;
bool static continue_launch = false;
bool static shooter_heat0_Unlimit = true;
uint8_t pc_aim_launch = 0;
uint8_t Excess_heat_indicator_light = 0;
//这里是控制拨弹轮的，射频用系统时间控制
float pluck_angle_add()
{
	static uint32_t system_runtime;
  static uint32_t last_system_runtime;
  system_runtime = xTaskGetTickCount();
	if(receive_103_data.shooter_heat0_cooling_limit >200)
	{
		if((receive_103_data .shooter_heat0_cooling_limit - receive_103_data .shooter_heat0) >80)
		{
			shooter_heat0_Unlimit = true;
			Excess_heat_indicator_light = 8;
		}
		else 
		{
			shooter_heat0_Unlimit = false;
			Excess_heat_indicator_light = 0;
		}
	}
	if((launch()|| pc_aim_launch) && shooter_heat0_Unlimit)
	{
		if(launch_frequency == LAUNCH_STOP)
		{
			pc_aim_launch = 0;
			return 0 ;
		}
		else if(launch_frequency == HIGH_FREQUENCY)
		{
			if(single_launch)
				{
					single_launch = false ;
					pc_aim_launch = 0;
					last_system_runtime = system_runtime ;
				  return 360.0f*PLUCK_RATIO/LATTICE_NUMBER;
				}
			if(system_runtime - last_system_runtime >= SIMGLE_FREQUENCY_INTERVAL)
				{
					continue_launch = true;
				}				
			if(system_runtime - last_system_runtime >= HIGH_FREQUENCY_INTERVAL && continue_launch)
				{
					pc_aim_launch = 0;
					last_system_runtime = system_runtime ;
					return 360.0f*PLUCK_RATIO/LATTICE_NUMBER;
				}
			else
				{
					return 0 ;
				}
		}
		else if(launch_frequency == SLOW_FREQUENCY)
		{
			if(single_launch)
				{
					single_launch = false ;
					pc_aim_launch = 0;
					last_system_runtime = system_runtime ;
				  return 360.0f*PLUCK_RATIO/LATTICE_NUMBER;
				}
			if(system_runtime - last_system_runtime >= SIMGLE_FREQUENCY_INTERVAL)
				{
					continue_launch = true;
				}				
			if(system_runtime - last_system_runtime >= SLOW_FREQUENCY_INTERVAL && continue_launch)
				{
					pc_aim_launch = 0;
					last_system_runtime = system_runtime ;
					return 360.0f*PLUCK_RATIO/LATTICE_NUMBER;
				}
			else
				{
					return 0 ;
				}
		}
	}
	else 
	{
		single_launch   = true;
		continue_launch = false;
		return 0 ;
	}
	return 0 ;
}

void shoot_mode()
{
	friction_wheel_mode();
}
