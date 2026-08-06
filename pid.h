#ifndef __PID_H__
#define __PID_H__

#include "ti_msp_dl_config.h"
typedef struct
{
	float target_val;   //目标值
	float Error;          /*第 k 次偏差 */
	float LastError;     /* Error[-1],第 k-1 次偏差 */
	float PrevError;    /* Error[-2],第 k-2 次偏差 */
	float Kp,Ki,Kd;     //比例、积分、微分系数
	float integral;     //积分值
	float output_val;   //输出值
}PID;
PID PosionPID;
PID cicriPID;
PID PosionPIDR;
PID cicriPIDR;
PID addPID;
PID grayPID;
PID gyroPID;
void PID_param_init(void);
float addPID_realize(PID *pid, float actual_val);
float PosionPID_realize(PID *pid, float actual_val);
float PID_realize_limited(PID *pid, float actual_val, float output_limit, float integral_limit);
float GYRO_Control(float now,float target);
#endif
