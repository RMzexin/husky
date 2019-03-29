#include "Init_Task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "remote_task.h"
#include "shoot_task.h"
#include "inv_mpu.h"

void bsp_init (void)
{
	RC_Ctl_t_Init();
	gimbal_init();
	chassis_init();
	shoot_init();
}

