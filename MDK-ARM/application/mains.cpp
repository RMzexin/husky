#include <ros.h>
#include "mains.h"
#include "control_task.h"
#include <skyguard_msgs/gimbal.h>
int ros_connect_tim = 90000;

#if imca_msg

void fire_callback(const imca_msgs::fire& rxbuff);
void gimbal_callback(const imca_msgs::gimbal& rxbuff);

ros::NodeHandle nh;

imca_msgs::gimbal gimbalctr_rec;
imca_msgs::fire firectr_rec;

ros::Subscriber<imca_msgs::fire> firectr("firectr", fire_callback);
ros::Subscriber<imca_msgs::gimbal> gimbalctr("gimbalctr", gimbal_callback);

void gimbal_callback( const imca_msgs::gimbal& rxbuff)
{
    gimbalctr_rec = rxbuff;
		set_ag1 = gimbalctr_rec.yaw;
	  set_ag2 = gimbalctr_rec.pitch;
}

void fire_callback( const imca_msgs::fire& rxbuff)
{
		firectr_rec = rxbuff;
}

#endif

#if skyguard_msg

void gimbal_callback(const skyguard_msgs::gimbal& rxbuff);

ros::NodeHandle nh;

skyguard_msgs::gimbal gimbalctr_rec;
//skyguard_msgs::gimbal gimbal_pos;

ros::Subscriber<skyguard_msgs::gimbal> gimbalctr("gimbalctr", gimbal_callback);
//ros::Publisher gimbal_position("position", &gimbal_pos);

extern uint8_t pc_aim_launch;
void gimbal_callback( const skyguard_msgs::gimbal& rxbuff)
{		
	static int movement = 1;
	if(pc_aim())
	{
    gimbalctr_rec = rxbuff;
		int yaw = gimbalctr_rec.yaw;
		int pitch = gimbalctr_rec.pitch;
		bool yaw_direction = yaw > 0;
		bool pitch_direction = pitch > 0;
		int yaw_movement = yaw_direction ? movement : -movement; 
		int pitch_movement = pitch_direction ? movement : -movement;
		
		while(!(yaw == 0 &&pitch == 0))
		{
			if(yaw == 0)
			{
				yaw_movement = 0;
			}
			if(pitch == 0)
			{
				pitch_movement = 0;
			}
			yaw -= yaw_movement;
			pitch -= pitch_movement;
			gimbal_PC_correct(&yaw_movement,&pitch_movement);
			vTaskDelay(10);
		}
      pc_aim_launch = 1;
	}
}

#endif

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void SETUP(void)
{
  nh.initNode();
//	nh.advertise(gimbal_position);
	nh.subscribe(gimbalctr);
	
	while (!nh.connected()){
		if(HAL_GetTick()>ros_connect_tim)
		{
			break;
		}
		nh.spinOnce();
	}   
	
	nh.loginfo("Skyguard Connected!");
	
}

void loop(void)
{  

	nh.spinOnce();
}

