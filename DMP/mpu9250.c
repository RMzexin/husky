#include "mpu9250.h"
#include "stm32f4xx_hal_i2c.h"
#include "i2c.h"


//初始化MPU9250 可不用（部分有误）
//返回值:0,成功
//    其他,错误代码
unsigned char MPU9250_Init(void)
{
	unsigned char pdata;
	//检查设备是否准备好  地址   检查 次数 超时时间  
  HAL_I2C_IsDeviceReady(&hi2c1,MPU9250_ADDR, 10, HAL_MAX_DELAY);
  unsigned char res=0;
	pdata=0x80; //复位MPU
	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_PWR_MGMT1_REG, 1, &pdata, 1, HAL_MAX_DELAY); //写0x80复位
//   MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//复位MPU9250
    HAL_Delay(500);  //延时100ms
	//唤醒MPU
	pdata=0x01; // 7位 1 重启  6位睡眠模式1 睡眠 2 唤醒   3位 为u你懂传感器0 开启    0-2位 时钟选择  01 PLL使用XZ轴陀螺作为参数
	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_PWR_MGMT1_REG, 1, &pdata, 1, HAL_MAX_DELAY); //写0x80复位
//    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//唤醒MPU9250
	//设陀螺仪 寄存器  2000  3
	pdata=3<<3; //设置3 为量程2000  右移3位 到对应的 3 4 位寄存器中
	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_GYRO_CFG_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
//    MPU_Set_Gyro_Fsr(3);					        	//陀螺仪传感器,±2000dps
	//设置加速度传感器量程 2g
	pdata=0; 
	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_ACCEL_CFG_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
//	  MPU_Set_Accel_Fsr(0);					       	 	//加速度传感器,±2g
//  pdata=50; //1000/50-1  这个还需要查资料查看  原因 和计算方法
//	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_SAMPLE_RATE_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
    MPU_Set_Rate(50);						       	 	//设置采样率50Hz
  pdata=0;
	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_INT_EN_REG, 1, &pdata, 1, HAL_MAX_DELAY);
//    MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //关闭所有中断
  pdata=0;
	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_USER_CTRL_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
//	  MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C主模式关闭
  pdata=0;
	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_FIFO_EN_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
//	  MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	//关闭FIFO
  pdata=0x82;
	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_INTBP_CFG_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
//	  MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//INT引脚低电平有效，开启bypass模式，可以直接读取磁力计
  pdata=MPU_DEVICE_ID_REG;
  res=HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDR, MPU_DEVICE_ID_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
//    res=MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //读取MPU6500的ID
    if(res==MPU6500_ID) //器件ID正确
    {
			pdata=0x01;
			HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR,MPU_PWR_MGMT1_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
//        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//设置CLKSEL,PLL X轴为参考
			pdata=0x00;
			HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR,MPU_PWR_MGMT2_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
//        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//加速度与陀螺仪都工作
//			pdata=50; //1000/50-1  这个还需要查资料查看  原因 和计算方法
//	    HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_SAMPLE_RATE_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
		    MPU_Set_Rate(50);						       	//设置采样率为50Hz   
    }else return 1;
  pdata=MAG_WIA;
  res=HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDR,MAG_WIA, 1, &pdata, 1, HAL_MAX_DELAY); 
//    res=MPU_Read_Byte(AK8963_ADDR,MAG_WIA);    			//读取AK8963 ID   
    if(res==AK8963_ID)
    {
			pdata=0x11;
			HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR,MAG_CNTL1, 1, &pdata, 1, HAL_MAX_DELAY); 
//        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);		//设置AK8963为单次测量模式
    }else return 1;

    return 0;
}
//设置MPU9250的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
unsigned char MPU_Set_LPF(u16 lpf)
{
	unsigned char data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
  return HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR, MPU_CFG_REG, 1, &data, 1, HAL_MAX_DELAY); 	//设置数字低通滤波器  
}

//设置MPU9250的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
unsigned char MPU_Set_Rate(u16 rate)
{
	unsigned char data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR,MPU_SAMPLE_RATE_REG, 1, &data, 1, HAL_MAX_DELAY);//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}
//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
	unsigned char buf[2]; 
	float temp;
	short raw;
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, MPU_TEMP_OUTH_REG, 1, buf, 2, HAL_MAX_DELAY); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87;  
    return temp*100;;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
unsigned char MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
	HAL_StatusTypeDef status;
    unsigned char buf[6]; 
	status=HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDR,MPU_GYRO_XOUTH_REG, 1,buf, 6, HAL_MAX_DELAY); 
	if(status==HAL_OK)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return status;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
unsigned char MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
	HAL_StatusTypeDef status;
    unsigned char buf[6]={0}; 
	status=HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, MPU_ACCEL_XOUTH_REG, 1,buf, 6, HAL_MAX_DELAY);
	if(status==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return status;
}

//得到磁力计值(原始值)
//mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
unsigned char MPU_Get_Magnetometer(short *mx,short *my,short *mz)
{
	HAL_StatusTypeDef status;
	unsigned char pdata=0X11;
    unsigned char buf[6];
	status=HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR,MAG_XOUT_L, 1,buf, 6, HAL_MAX_DELAY);
	if(status==0)
	{
		*mx=((u16)buf[1]<<8)|buf[0];  
		*my=((u16)buf[3]<<8)|buf[2];  
		*mz=((u16)buf[5]<<8)|buf[4];
	} 	
		HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDR,MAG_CNTL1, 1, &pdata, 1, HAL_MAX_DELAY); 
    return status;
}

//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
unsigned char MPU_Write_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
{
	HAL_I2C_Mem_Write(&hi2c1, ((addr<<1)|0), reg, 1, (unsigned char *)buf, len, HAL_MAX_DELAY);
	return 0;
} 

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
unsigned char MPU_Read_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
{ 
	HAL_I2C_Mem_Read(&hi2c1, ((addr<<1)|1), reg, 1, (unsigned char *)buf, len, HAL_MAX_DELAY);
	return 0;
	
}


