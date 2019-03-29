#include "pid.h"
#include <arm_math.h>

#define VAL_LIMIT(val, min, max)\
if(val<=min){                   \
	val = min;                    \
}else if(val>=max){             \
	val = max;}

float PID_Calc(pid_t* pid, float get, float set)
{
	static unsigned short index = 0;
	pid->err[LAST]= pid->err[NOW];
	pid->get[NOW] = get;
  pid->set[NOW] = set;
  pid->err[NOW] = set - get;	
	
	if(pid->Voltage > pid->Voltage_max)
       {
         index = 1;
				 if(pid->err[NOW]<0)
					pid->Voltage+=pid->err[NOW]; 
       }
			 else if(pid->Voltage < pid->Voltage_min)
       {
         index = 1;
				 if(pid->err[NOW]>0)
					pid->Voltage+=pid->err[NOW];
       }
//			 else if(fabs(pid->Voltage) < pid->Voltage_max-(pid->Voltage_max/2))
//		   {
//         index=1;
//				 pid->Voltage+=pid->err[NOW];
//			 }
//			 else
//			{
//				index=(pid->Voltage_max-fabs(pid->Voltage))/(pid->Voltage_max/2);
//				pid->Voltage+=pid->err[NOW];
//			}
			 else
			{
				index = 1;
				pid->Voltage+=pid->err[NOW];
			}

			pid->pout=pid->p*pid->err[NOW];
			pid->iout=index*pid->i*pid->Voltage;
			pid->dout=pid->d*(pid->err[NOW]-pid->err[LAST]);
			pid->pidout=pid->pout + pid->iout + pid->dout;
			VAL_LIMIT(pid->pidout,pid->MinOutput,pid->MaxOutput);
			return pid->pidout;	
}


void PID_Init(pid_t* pid,const float PID[3],float Voltage_max,float Voltage_min,float MaxOutput,float MinOutput)
{
	pid->p           = PID[0];
	pid->i           = PID[1];
	pid->d           = PID[2];
	pid->Voltage_max = Voltage_max;
	pid->Voltage_min = Voltage_min;
	pid->MaxOutput   = MaxOutput;
	pid->MinOutput   = MinOutput;
	pid->set[0] = pid->set[1] = pid->set[2] = 0.0f;
	pid->get[0] = pid->get[1] = pid->get[2] = 0.0f;
	pid->err[0] = pid->err[1] = pid->err[2] = 0.0f;
}





