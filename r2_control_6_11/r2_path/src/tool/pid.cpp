#include "r2_path/tool/pid.h"


PID::PID()
{
	last_time = ros::Time::now();
	now_time = ros::Time::now();
	dt = 0;
}

PID::~PID()
{
}


float PID::Adjust(void)
{
    //get real time failed
	last_time = now_time;
	now_time = ros::Time::now();
	dt = (now_time - last_time).toSec();
    // ROS_INFO("dt: %f", dt);
	float c1,c2,c3,gama=0.1;
	static float control_dk;
    static float control_dk_last;

    //calculate error and deadzone
    error = target - current;
    if(abs(error) < DeadZone)
    {
        Out=0;
        return Out;
    }
    
    //lowpass filter, change the trust value to adjust the filter
    error = LowPass_error.f(error);

    if(Imcreatement_of_Out)     //output increment mode
        P_Term = Kp * error - Kp * pre_error;
    else                        //output position mode
          P_Term = Kp * error;

    //calculate integral term, if use integral term
    if(Ki!=0)
    {
        if(Imcreatement_of_Out)
            integral_e = error;
        else
            integral_e  += error*dt;
        integral_e = abs_limit(integral_e, I_Term_Max);
    }
    else
    {
        integral_e = 0;
    }

    //integral separate
    if(abs(error) < I_SeparThresh)
    {
        I_Term = Ki * integral_e;
        I_Term = abs_limit(I_Term, I_Term_Max);
    }
    else
    {
        I_Term = 0;
    }

    float d_err = 0;
    if(D_of_Current)
    {
        if(Imcreatement_of_Out)
            d_err = (current + eriler_Current - 2*pre_Current) / dt;
        else
		{
			// c1 = (gama*Kd)/(gama*Kd+Kp);
			// c2 = (Kd+Kp)/(gama*Kd+Kp);
			// c3 = Kd/(gama*Kd+Kp);
			// control_dk = c1*control_dk_last+c2*control_dk+c3*control_dk_last;
			d_err = (current - pre_Current) / dt;
			// d_err = control_dk/Kd;
		}
            
    }
    else
    {
        if(Imcreatement_of_Out)
            d_err = (error + eriler_error - 2*pre_error) / dt;
        else
            d_err = (error - pre_error) / dt;
    }

    d_err = LowPass_d_err.f(d_err);     //进行不完全微分
    D_Term = Kd * d_err;

    eriler_error = pre_error;
    pre_error = error;
    eriler_Current = pre_Current;
    pre_Current = current;
    if(Imcreatement_of_Out)
        Out = P_Term + I_Term + D_Term + last_out;
    else
        Out = P_Term + I_Term + D_Term;
		
    last_out = Out;

    Out = abs_limit(Out, Out_Max);

	// ROS_INFO("target: %f, current: %f, error: %f, Out: %f", target, current, error, Out);
    return Out;
}
