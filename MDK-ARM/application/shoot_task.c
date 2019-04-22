#include "shoot_task.h"
#include "keymouse_task.h"
#include "remote_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ramp.h"
#include "tim.h"

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
	motor_pwm_setvalue1(wheel_value);
	HAL_Delay(3000);
	for(int i = 0;i < 10;i++)
	{
		wheel_value -=100.0f;
		motor_pwm_setvalue1(wheel_value);
		HAL_Delay(100);
	}
}

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
				launch_frequency = SLOW_FREQUENCY;
				ramp_init(&Fr_stop_ramp, WHEEL_RAMP_TIME),ramp_init(&Fr_slow_ramp, WHEEL_RAMP_TIME);break;	
			}
}


float pluck_angle_add()
{
	static uint32_t system_runtime;
  static uint32_t last_system_runtime;
  system_runtime = xTaskGetTickCount();
	if(launch())
	{
		if(launch_frequency == LAUNCH_STOP)
		{
			return 0 ;
		}
		if(launch_frequency == HIGH_FREQUENCY)
		{
		if(system_runtime - last_system_runtime >= HIGH_FREQUENCY_INTERVAL)
			{
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
		if(system_runtime - last_system_runtime >= SLOW_FREQUENCY_INTERVAL)
			{
				last_system_runtime = system_runtime ;
				return 360.0f*PLUCK_RATIO/LATTICE_NUMBER;
			}
			else
			{
				return 0 ;
			}
		}
	}
	return 0 ;
}

void shoot_mode()
{
	friction_wheel_mode();
}
