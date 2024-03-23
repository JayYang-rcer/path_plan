#ifndef __PID_H
#define __PID_H

#include "stdint.h"
#include  "r2_path/math.h"

// #define  ABS(x)      ((x)>0? (x):(-(x)))

typedef struct PID_Data
	{
		float Error ;
		float Last_Error ;
		float Earlier_Error ;
		float Error_Max;
		float Sum_Error;
		float D_Error;
		float Dead_Size;
		float K_P ;
		float K_I ;
		float K_D ;
		float I_Separate ;
		float I_Limit ;
		float Out_MAX ;
		float Out_MIN ; 
		float Output ;
		uint8_t first_flag;
	}PID_Data;


class PID : public math_ns::math
{
private:

public:
	PID_Data pid_data;

	~PID();
	PID();

	virtual float PID_Incremental_PID_Calculation_by_error(float error);
	virtual float PID_Position_Calculate_by_error(float error);
};


#endif 
