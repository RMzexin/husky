#include "imu_task.h"
#include "inv_mpu.h"
#include "mpu9250.h"
#include "control_task.h"
#include "IOI2C.h"
#include "REG.h"
#include "main.h"
#include "cmsis_os.h"

IMU_data_t  IMU_chassis_data , IMU_gimbal_data;
IMU_t IMU_chassis , IMU_gimbal;
unsigned char chassis_chrTemp[30];
unsigned char gimbal_chrTemp[30];

void Get_IMU_Data()
{	
	IICreadBytes(0x51,AX,24,&gimbal_chrTemp[0]);
	IICreadBytes(0x50,AX,24,&chassis_chrTemp[0]);
	if(gimbal_chrTemp[22]!= 0x00)
		{
			IMU_gimbal .last_yaw = IMU_gimbal .ecd_yaw;
			IMU_gimbal .ecd_yaw  = (float)CharToShort(&gimbal_chrTemp[22])/32768*180.0f;
		  if(IMU_gimbal .ecd_yaw - IMU_gimbal .last_yaw < -300)
				IMU_gimbal .round_cnt ++;
			else if(IMU_gimbal .ecd_yaw - IMU_gimbal .last_yaw > 300)
				IMU_gimbal .round_cnt --;
			IMU_gimbal .C_yaw =-(IMU_gimbal .ecd_yaw + IMU_gimbal .round_cnt*360.0f);
		}
	if(chassis_chrTemp[22]!= 0x00)
		{
			IMU_chassis .last_yaw = IMU_chassis .ecd_yaw;
			IMU_chassis .ecd_yaw  = (float)CharToShort(&chassis_chrTemp[22])/32768*180.0f;
		  if(IMU_chassis .ecd_yaw - IMU_chassis .last_yaw < -300)
				IMU_chassis .round_cnt ++;
			else if(IMU_chassis .ecd_yaw - IMU_chassis .last_yaw > 300)
				IMU_chassis .round_cnt --;
			IMU_chassis .C_yaw =(IMU_chassis .ecd_yaw + IMU_chassis .round_cnt*360.0f);
		}
		Correct_Angle_Feedback();
}

/**
  * @brief  获取mpu9250角速度
  * @param  X轴角速度存储地址
  * @param  Y轴角速度存储地址
  * @param  Z轴角速度存储地址
  * @note   默认量程±2000°
  * @retval 0读取成功       1读取失败
  */
uint8_t MPU9250_GetGyro(float *X, float *Y, float *Z)
{
//    unsigned long timestamp;
    
//    if(mpu_get_gyro_reg(IMU_data.gyro, &timestamp))
//    {
//        return 1;
//    }
//    else
//    {
//        *X = 0.0610352F * IMU_data.gyro[0];
//        *Y = 0.0610352F * IMU_data.gyro[1];
//        *Z = 0.0610352F * IMU_data.gyro[2];
//        
//        return 0;
//    }
}

/**
  * @brief  获取mpu9250重力加速度
  * @param  X轴重力加速度存储地址
  * @param  Y轴重力加速度存储地址
  * @param  Z轴重力加速度存储地址
  * @note   默认量程±2g°
  * @retval 0读取成功       1读取失败
  */
uint8_t MPU9250_GetAccel(float *X, float *Y, float *Z)
{
//    unsigned long timestamp;
//    
//    if(mpu_get_accel_reg(IMU_data.acc, &timestamp))
//    {
//        return 1;
//    }
//    else
//    {
//        *X = 2 * Acceleration_Of_Gravity * IMU_data.acc[0] / 32768;
//        *Y = 2 * Acceleration_Of_Gravity * IMU_data.acc[1] / 32768;
//        *Z = 2 * Acceleration_Of_Gravity * IMU_data.acc[2] / 32768;
//        
//        return 0;
//    }
}

