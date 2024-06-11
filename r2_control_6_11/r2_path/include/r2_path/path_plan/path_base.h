#ifndef __PATH_H
#define __PATH_H

#include "stdint.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "r2_path/tool/math.h"
#include "r2_msgs/path_cmd.h"
#include "nav_msgs/Path.h"
#include <tf/tf.h> 
#include "r2_msgs/pid_followerAction.h"
#include <actionlib/client/simple_action_client.h>

#define PI 3.1415926

enum PATH_ENUM	//the enum of the path plan mode
{
	Hand=0,
	LockupPoint_e=1,
	PATH1=2,
	PATH2=3,
	auto_take_put=4,
	Stop=5,
};

enum AUTO_TAKE_PUT
{
	Take=1,
	Put=2,
};

enum SILO_ENUM
{
	ToSilo_one=1,
	ToSilo_two=2,
	ToSilo_three=3,
	ToSilo_four=4,
	ToSilo_five=5,
};

enum PATH_STATUS
{
	SUCCCEED = 12,
	ACTIVE = 13,
	CANCEL = 14,
};

namespace path_base_ns{
	
	class path_base : public math_ns::math
	{
	private:
		double COS,SIN;	//cos and sin of Yaw,use to transform the robot velocity to world velocity
		geometry_msgs::PoseStamped pose;
		int k; float threeb_t;	//the control frequency, the segment number, the used time of the segment
		float threeb_f1s; float threeb_f2s; float threeb_f3s; float threeb_f4s; //the four parts(function) of the position spline function

	public:
		path_base(/* args */)
		{
		}
		

		virtual geometry_msgs::Twist RobotMove_To_WorldMove(geometry_msgs::Twist target_robot_vel, float now_yaw)
		{
			COS = cos (now_yaw*PI/180);
			SIN = sin (now_yaw*PI/180);

			geometry_msgs::Twist world_vel;
			world_vel.linear.x  = (target_robot_vel.linear.x * COS + target_robot_vel.linear.y * SIN);
			world_vel.linear.y  = -(target_robot_vel.linear.x * SIN - target_robot_vel.linear.y * COS);
			world_vel.angular.z = target_robot_vel.angular.z;
			
			return world_vel;
		}
		nav_msgs::Path path_msg;
		protected:
			
			virtual int ThreeB_Curve(int num, std::vector<float>& X, std::vector<float>& Y, std::vector<float>& YAW, int total_points)
			{
				double path_Yaw;
				geometry_msgs::Quaternion goal_quat;
				path_msg.poses.clear();
				for (int i = 0; i < total_points; ++i)
				{
					int k = (int)(i*(num + 1)/total_points); //第k段
					float threeb_t = i - k*total_points/(num + 1);
					threeb_t = threeb_t * (num + 1)/total_points; //第k段时间

					//位置样条函数
					threeb_f1s = (1 - threeb_t) * (1 - threeb_t) * (1 - threeb_t) / 6;
					threeb_f2s = (3 * threeb_t * threeb_t * threeb_t - 6 * threeb_t * threeb_t + 4) / 6;
					threeb_f3s = (-3 * threeb_t * threeb_t * threeb_t + 3 * threeb_t * threeb_t + 3 * threeb_t + 1) / 6;
					threeb_f4s = (threeb_t * threeb_t * threeb_t) / 6;

					// 计算目标跟踪点
					pose.pose.position.x = (double)(X[k]*threeb_f1s + X[k+1]*threeb_f2s + X[k+2]*threeb_f3s + X[k+3]*threeb_f4s);
					pose.pose.position.y = (double)(Y[k]*threeb_f1s + Y[k+1]*threeb_f2s + Y[k+2]*threeb_f3s + Y[k+3]*threeb_f4s);
					path_Yaw = YAW[k]*threeb_f1s + YAW[k+1]*threeb_f2s + YAW[k+2]*threeb_f3s + YAW[k+3]*threeb_f4s;
					path_Yaw = path_Yaw/180.00*PI;

					goal_quat = tf::createQuaternionMsgFromYaw(path_Yaw);
					pose.pose.orientation = goal_quat;
					pose.header.stamp = ros::Time::now();
					pose.header.frame_id = "map";
					path_msg.poses.push_back(pose);

				}
        		path_msg.header.frame_id = "map";
				path_msg.header.stamp = ros::Time::now();

				return 0;
			}
	};


	
}

class path_plan : public path_base_ns::path_base
{
private:
	ros::NodeHandle nh;
	ros::Publisher path_pub;
	ros::Subscriber path_cmd_sub;
	ros::Subscriber robot_pose_sub;
	std::string odom_topic;
	r2_msgs::pid_followerGoal goal;

	std::vector<float> X0,Y0,Yaw0;
	std::vector<float> X1,Y1,Yaw1;
	std::vector<float> BALL_X,BALL_Y,BALL_Yaw;
	std::string field_direction;
	float path1_control_num,path1_total_points;
    float path2_control_num,path2_total_points;
	float takeBall_speed=0.0f, control_frequence;
	float ball_distance_error=0;
	
	PATH_STATUS path_status;
	geometry_msgs::Point robot_pose;
	geometry_msgs::PoseStamped start_pose_,end_pose_;
	geometry_msgs::PoseStamped lockup_pose;

	float takeBall_calculate(geometry_msgs::PoseStamped start_pos, geometry_msgs::PoseStamped end_pos);

public:
	actionlib::SimpleActionClient<r2_msgs::pid_followerAction> client;
	path_plan(/* args */);
	int take_ball_planner(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped middle_pose, geometry_msgs::PoseStamped end_pose);
	void planner_fsm();
	void lockup_point(geometry_msgs::Point target);
	PATH_ENUM path_mode;
};

#endif 
