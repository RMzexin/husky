#ifndef _PID_H_
#define _PID_H_
#include "stm32f4xx_hal.h"

//PID结构体整合
enum{
    LLAST	= 2,
    LAST 	= 1,
    NOW 	= 0,
    
    POSITION_PID,
    DELTA_PID,
};

typedef struct 
	{
		float p;
		float i;
		float d;
    
		float set[3];				//目标值,包含NOW， LAST， LLAST上上次
		float get[3];				//测量值
    float err[3];				//误差
	
	  float Voltage;     //积分量
	  float Voltage_max;     //最大积分量
	  float Voltage_min;     //最小积分量
	
    float pout;							//p输出
    float iout;							//i输出
    float dout;							//d输出
	  float pidout;
		float MaxOutput;				//输出限幅
		float MinOutput;	
		
		
}pid_t;
	
void PID_Init(pid_t* pid,const float PID[3],float Voltage_max,float Voltage_min,float MaxOutput,float MinOutput);
float PID_Calc(pid_t* pid, float get, float set);

#endif
