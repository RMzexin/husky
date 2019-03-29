#include "shoot_task.h"
#include "remote_task.h"
#include "ramp.h"
#include "tim.h"

ramp_t Fr_stop_ramp = RAMP_GEN_DAFAULT;
ramp_t Fr_slow_ramp = RAMP_GEN_DAFAULT;
ramp_t Fr_fast_ramp = RAMP_GEN_DAFAULT;

float wheel_value = 2000.0;
static uint8_t count;

void shoot_init()
{
	motor_pwm_setvalue1(wheel_value);
	ramp_init(&Fr_stop_ramp, WHEEL_RAMP_TIME);
	ramp_init(&Fr_slow_ramp, WHEEL_RAMP_TIME);
	ramp_init(&Fr_fast_ramp, WHEEL_RAMP_TIME);
}

void friction_wheel_mode()
{
	count++;
	switch ( FRICTION_WHEEL_MODE() )
	{
		case WHEEL_STOP_MODE :
			wheel_value += (WHEEL_STOP_VALUE - wheel_value) * ramp_calc(&Fr_stop_ramp);
		  if( wheel_value != WHEEL_STOP_VALUE){
			motor_pwm_setvalue1(wheel_value);}
	    ramp_init(&Fr_slow_ramp, WHEEL_RAMP_TIME),ramp_init(&Fr_fast_ramp, WHEEL_RAMP_TIME);break;
		case WHEEL_SLOW_MODE :
			wheel_value += (WHEEL_SLOW_VALUE - wheel_value) * ramp_calc(&Fr_slow_ramp);
		  if( wheel_value != WHEEL_SLOW_VALUE){
			motor_pwm_setvalue1(wheel_value);}
	    ramp_init(&Fr_stop_ramp, WHEEL_RAMP_TIME),ramp_init(&Fr_fast_ramp, WHEEL_RAMP_TIME);break;
		case WHEEL_FAST_MODE :
			wheel_value += (WHEEL_FAST_VALUE - wheel_value) * ramp_calc(&Fr_fast_ramp);
		  if( wheel_value != WHEEL_FAST_VALUE){
			motor_pwm_setvalue1(wheel_value);}
	    ramp_init(&Fr_stop_ramp, WHEEL_RAMP_TIME),ramp_init(&Fr_slow_ramp, WHEEL_RAMP_TIME);break;	
	}
}

void shoot_mode()
{
	friction_wheel_mode();
	
}
