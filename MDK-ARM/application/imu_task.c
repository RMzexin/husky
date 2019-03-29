#include "imu_task.h"
#include "inv_mpu.h"
#include "mpu9250.h"
#include "IOI2C.h"
#include "REG.h"
#include "main.h"
#include "cmsis_os.h"

IMU_data_t  IMU_data;
IMU_t IMU;
unsigned char chrTemp[30];

uint8_t Get_IMU_Data(void)
{	
	IICreadBytes(0x50, AX, 24,&chrTemp[0]);
		if(chrTemp[22]!= 0x00)
		{
		IMU.yaw = (float)CharToShort(&chrTemp[22])/32768*180 + 180.0f;
			return 1 ;
		}
		return 0 ;
//	uint8_t result;  
//	result = mpu_dmp_get_data(&IMU_data.pitch, &IMU_data.roll, &IMU_data.yaw);
//	if(result)return 1;
//	IMU.pitch = IMU_data .pitch + 180.0f;
//	IMU.yaw   = IMU_data .yaw   + 180.0f;
//	IMU.roll  = IMU_data .roll  + 180.0f;
//	return 0;
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
    unsigned long timestamp;
    
    if(mpu_get_gyro_reg(IMU_data.gyro, &timestamp))
    {
        return 1;
    }
    else
    {
        *X = 0.0610352F * IMU_data.gyro[0];
        *Y = 0.0610352F * IMU_data.gyro[1];
        *Z = 0.0610352F * IMU_data.gyro[2];
        
        return 0;
    }
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
    unsigned long timestamp;
    
    if(mpu_get_accel_reg(IMU_data.acc, &timestamp))
    {
        return 1;
    }
    else
    {
        *X = 2 * Acceleration_Of_Gravity * IMU_data.acc[0] / 32768;
        *Y = 2 * Acceleration_Of_Gravity * IMU_data.acc[1] / 32768;
        *Z = 2 * Acceleration_Of_Gravity * IMU_data.acc[2] / 32768;
        
        return 0;
    }
}

