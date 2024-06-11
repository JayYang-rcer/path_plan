#include "r2_path/pid.h"


PID::PID()
{
}

PID::~PID()
{
}


//增量式PID(速度环)
float PID::PID_Incremental_PID_Calculation_by_error(float error)	
{
	pid_data.Error = error;
	pid_data.D_Error = error - pid_data.Last_Error;
	pid_data.Sum_Error = error;
	
	//计算输出值
	pid_data.Output += pid_data.K_P * (pid_data.D_Error) + abs_limit(pid_data.K_I*pid_data.Error,pid_data.I_Limit) + pid_data.K_D * (pid_data.Error +  pid_data.Earlier_Error - 2*pid_data.Last_Error);
		
	//输出限制
	if(pid_data.Output > pid_data.Out_MAX )  pid_data.Output = pid_data.Out_MAX;
	if(pid_data.Output < - pid_data.Out_MAX )  pid_data.Output = -pid_data.Out_MAX;
	
	pid_data.Earlier_Error = pid_data.Last_Error;
	pid_data.Last_Error = pid_data.Error;

	//死区
	if(abs(pid_data.Error) < pid_data.Dead_Size)
	{
		pid_data.Output = 0;
	}

	return pid_data.Output;
}


//位置环PID，有死区
float PID::PID_Position_Calculate_by_error(float error)
{
	if(pid_data.first_flag == 1)
	{
		pid_data.Last_Error = error;
		pid_data.Earlier_Error = error;
		pid_data.first_flag = 0;
	}

	pid_data.Error = error;
	pid_data.Sum_Error += pid_data.Error;
	pid_data.D_Error = pid_data.Error - pid_data.Last_Error;

	pid_data.Output =  pid_data.K_P * pid_data.Error + abs_limit(pid_data.K_I * pid_data.Sum_Error, pid_data.K_I ) + pid_data.K_D * pid_data.D_Error;

	if (pid_data.Output > pid_data.Out_MAX) pid_data.Output = pid_data.Out_MAX;
	if (pid_data.Output < pid_data.Out_MIN) pid_data.Output = pid_data.Out_MIN;

	pid_data.Last_Error = pid_data.Error;
	if(abs(pid_data.Error) < pid_data.Dead_Size) pid_data.Output = 0;
	
	return pid_data.Output;
}