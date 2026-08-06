#include "pid.h"

#define cicir  260
#include "wit.h"

void PID_param_init()
{
	PosionPID.target_val=0 ;	//70			
	PosionPID.output_val=0.0;
	PosionPID.Error=0.0;
	PosionPID.LastError=0.0;
	PosionPID.integral=0.0;
	PosionPID.Kp = 0.16;//0.2
	PosionPID.Ki = 0.16;//0，12
	PosionPID.Kd = 0.05;

	addPID.target_val=0;				
	addPID.output_val=0.0;
	addPID.Error=0.0;
	addPID.LastError=0.0;
	addPID.integral=0.0;
	addPID.Kp = 0.18;//Task2 straight speed balance
	addPID.Ki = 0.02;
	addPID.Kd = 0.00;

	
	cicriPID.target_val=0;				
	cicriPID.output_val=0.0;
	cicriPID.Error=0.0;
	cicriPID.LastError=0.0;
	cicriPID.integral=0.0;
	cicriPID.Kp = 0.3;//0.5转一圈最好
	cicriPID.Ki = 0.00;//0.005
	cicriPID.Kd = 0;

	PosionPIDR.target_val=0;				
	PosionPIDR.output_val=0.0;
	PosionPIDR.Error=0.0;
	PosionPIDR.LastError=0.0;
	PosionPIDR.integral=0.0;
	PosionPIDR.Kp = 0.16;//0.15
	PosionPIDR.Ki = 0.165;
	PosionPIDR.Kd = 0.05;
	



	cicriPIDR.target_val=0;				
	cicriPIDR.output_val=0.0;
	cicriPIDR.Error=0.0;
	cicriPIDR.LastError=0.0;
	cicriPIDR.integral=0.0;
	cicriPIDR.Kp = 0.3;//0.5转一圈最好
	cicriPIDR.Ki = 0.00;//0.005
	cicriPIDR.Kd = 0;

	grayPID.target_val=0;
	grayPID.output_val=0.0;
	grayPID.Error=0.0;
	grayPID.LastError=0.0;
	grayPID.PrevError=0.0;
	grayPID.integral=0.0;
	grayPID.Kp = 0.47;
	grayPID.Ki = 0.0;
	grayPID.Kd = 0.0;

	gyroPID.target_val=0;
	gyroPID.output_val=0.0;
	gyroPID.Error=0.0;
	gyroPID.LastError=0.0;
	gyroPID.PrevError=0.0;
	gyroPID.integral=0.0;
	gyroPID.Kp = 0.33;//陀螺仪
	gyroPID.Ki = 0.01;
	gyroPID.Kd = 0.13;
}
float PosionPID_realize(PID *pid, float actual_val)
{
	/*计算目标值与实际值的误差*/
	pid->Error = pid->target_val - actual_val;
	/*积分项*/
	pid->integral += pid->Error;
	/*PID算法实现*/
	pid->output_val = pid->Kp * pid->Error +
	                  pid->Ki * pid->integral +
	                  pid->Kd *(pid->Error -pid->LastError);
	/*误差传递*/
	pid-> LastError = pid->Error;
	/*返回当前实际值*/
	return pid->output_val;
}
float addPID_realize(PID *pid, float actual_val)
{
	/*计算目标值与实际值的误差*/
	pid->Error = pid->target_val - actual_val;
	/*PID算法实现，照搬公式*/
	pid->output_val += pid->Kp * (pid->Error - pid-> LastError) +
	                  pid->Ki * pid->Error +
	                  pid->Kd *(pid->Error -2*pid->LastError+pid->PrevError);
	/*误差传递*/
	pid-> PrevError = pid->LastError;
	pid-> LastError = pid->Error;
	/*返回当前实际值*/
	return pid->output_val;
}


// 角度环 PID
float PID_realize_limited(PID *pid, float actual_val, float output_limit, float integral_limit)
{
	pid->Error = pid->target_val - actual_val;
	pid->integral += pid->Error;

	if(pid->integral > integral_limit) {
		pid->integral = integral_limit;
	} else if(pid->integral < -integral_limit) {
		pid->integral = -integral_limit;
	}

	pid->output_val = pid->Kp * pid->Error +
	                  pid->Ki * pid->integral +
	                  pid->Kd * (pid->Error - pid->LastError);

	if(pid->output_val > output_limit) {
		pid->output_val = output_limit;
	} else if(pid->output_val < -output_limit) {
		pid->output_val = -output_limit;
	}

	pid->PrevError = pid->LastError;
	pid->LastError = pid->Error;
	return pid->output_val;
}

#define   Kp3       1
#define   Ki3       0
#define   Kd3  	    0

// extern wit_data.yaw;
float GYRO_Control(float now,float target)
{
	static float Bias, Last_bias, Last2_bias, Pwm;
	Bias = target-now;
	Pwm += Kp3 * (Bias - Last_bias) + Ki3 * Bias + Kd3 * (Bias - 2 * Last_bias + Last2_bias);

	Last_bias = Bias;
	Last2_bias = Last_bias;
	return Pwm;
}
