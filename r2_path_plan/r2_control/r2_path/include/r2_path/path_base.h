#ifndef __PATH_H
#define __PATH_H

#include "stdint.h"
#include "ros/ros.h"
#include <iostream>
#include "nav_msgs/Odometry.h"
#include "r2_msgs/pid_info.h"
#include "r2_path/pid.h"
#include "r2_msgs/path_cmd.h"
#include "r2_msgs/action_cmd.h"
#include "r2_path/math.h"
#include "geometry_msgs/Point.h"

#define PI 3.1415926

enum PATH_ENUM	//the enum of the path plan mode
{
	Hand=0,
	LockupPoint_e=1,
	PATH1=2,
	PATH2=3,
	auto_take_put,
	Take=5,
	ToSilo_one=6,
	ToSilo_two=7,
	ToSilo_three=8,
	ToSilo_four=9,
	ToSilo_five=10,
};
	
	

namespace path_base_ns{
	
	class path_base : public PID
	{
	private:
		double COS,SIN;	//cos and sin of Yaw,use to transform the robot velocity to world velocity
		float error_X, error_Y, error_Yaw;	//PD controller error
		
		int first_time_flag; //the flag to judge if it is the first time to run the function
		float threeB_Hz; int k; float threeb_t;	//the control frequency, the segment number, the used time of the segment
		float last_X; float last_Y; float last_Yaw;	 //the last target point of the robot in path plan
		float threeb_f1s; float threeb_f2s; float threeb_f3s; float threeb_f4s; //the four parts(function) of the position spline function

	public:
		path_base(/* args */);
		geometry_msgs::Point now_path_point;	//the target point of the robot in path plan
		geometry_msgs::Twist now_path_vel;     //the target velocity of the robot in path plan
		geometry_msgs::Point robot_pos;	//now position of the robot 
		geometry_msgs::Twist robot_target_vel; //the target velocity command of the robot
		PID path_pid_x_,path_pid_y_,path_pid_yaw_;  //create PID class
		float pre_feedx,pre_feedy;
		void Angle_Limit(float *angle);
		void Yaw_Adjust(float target_angle);
		virtual void LockupPoint(float POS_X, float POS_Y, float POS_YAW, float Vxy_max, float Vw_Max);
		virtual int ThreeB_PathPlan(float t_real, float t_target, int num, float *X , float *Y, float *Yaw);
		virtual PATH_ENUM stringToPATH_ENUM(const std::string& path_str);
		geometry_msgs::Twist RobotMove_To_WorldMove(geometry_msgs::Twist target_robot_vel);

	protected:
		virtual void PD_Controller(geometry_msgs::Point target_point, geometry_msgs::Point robot_position);
		virtual int TrapezoidPlaning_set(float S,float POS_X,float POS_Y,float POS_YAW,float V_max,float R_ac,float R_de);
		virtual int Bezier_PathPlan(float t_real, float t_target, int num, float *X, float *Y, float *Yaw);
	};
}


#endif 
