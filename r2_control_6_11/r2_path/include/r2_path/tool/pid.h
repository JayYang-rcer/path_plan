#ifndef __PID_H
#define __PID_H

#include "stdint.h"
#include "r2_path/tool/math.h"
#include "r2_path/tool/filter.h"
#include "ros/ros.h"


class PID : public math_ns::math 
{
public:
    PID();
	~PID();
    float Adjust();
    void PID_Param_Init(float _Kp, float _Ki, float _Kd, float _I_Term_Max, float _Out_Max)
    {
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;
        I_Term_Max = _I_Term_Max;
        Out_Max = _Out_Max;
		ROS_INFO("Kp: %f, Ki: %f, Kd: %f, I_Term_Max: %f, Out_Max: %f", Kp, Ki, Kd, I_Term_Max, Out_Max);
    }

    float current=0,target=0,Out=0;
    float DeadZone = 0; 		        /*!< 死区，需为整数，fabs(error)小于DeadZone时，输出为0。 */
    float I_SeparThresh = 400;          /*!< 积分分离阈值，需为正数。fabs(error)大于该阈值取消积分作用。*/
    bool D_of_Current = false;          /*!< 启用微分先行，文献中Current多译作Process Variable(PV)。 */
    bool Imcreatement_of_Out = false;   /*!< 输出增量模式。 */

    LowPassFilter LowPass_error = LowPassFilter(1);
    LowPassFilter LowPass_d_err = LowPassFilter(1); /*!< 不完全微分。 */

private:
	ros::Time last_time,now_time;
	float dt;

    const uint8_t ID = 0;
    float error=0; 
    float Kp = 0, Ki = 0, Kd = 0;
    float I_Term_Max = 0;               /*<! I项限幅 */
    float Out_Max = 0;                  /*<! 输出限幅 */

    float pre_error = 0;        /*!< 上一次的error。 */
    float eriler_error = 0;     /*!< 上上次的error。 */
    float pre_Current = 0;      /*!< 上一次的Current。 */
    float eriler_Current = 0;   /*!< 上上次的Current。 */
    float integral_e = 0;       /*!< 积分器输入 */
    float I_Term = 0;			/* 积分器输出 */
    float P_Term = 0;			/* 比例器输出 */
    float D_Term = 0;			/* 微分器输出 */
    float last_out = 0;         /*!< 上一次的输出 */
};



#endif 
