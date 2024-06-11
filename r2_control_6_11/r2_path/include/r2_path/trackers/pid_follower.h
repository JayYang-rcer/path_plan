#pragma once
#include "ros/ros.h"
#include "r2_path/tool/pid.h"
#include "r2_msgs/global_path.h"
#include "math.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "r2_msgs/pid_followerAction.h"
#include <actionlib/server/simple_action_server.h>

#define PI 3.1415926

typedef actionlib::SimpleActionServer<r2_msgs::pid_followerAction> Server;

namespace tracker
{
class pid_follower
{
private:
    ros::NodeHandle nh;
    ros::ServiceClient path_client;
    ros::Subscriber robot_pose_sub;
    ros::Timer control_timer;
    ros::Publisher vel_pub;
    Server as_;
    std::string pid_follower_action;
    std::string odom_topic;

    nav_msgs::Path path;
    geometry_msgs::Point robot_pose;
    geometry_msgs::Twist target_vel;
    geometry_msgs::Twist forecast_vel;
    PID pid_x, pid_y, pid_yaw;
    float pid_x_PreFeed, pid_y_PreFeed, pid_yaw_PreFeed;
    float LockupPoint_xMax, LockupPoint_yMax, LockupPoint_YawMax;
    float xy_deadzone=0,yaw_deadzone=0;
    int path_flag = 0, path_done = 0;
    float control_frequence=0;

    void robot_pose_callback(const geometry_msgs::Point::ConstPtr &msg);
    void PD_control(geometry_msgs::Point target, geometry_msgs::Point current);
    void control_timer_callback(const ros::TimerEvent&);
    void pid_init(void);

public:
    pid_follower(std::string name);
    ~pid_follower();
    void control(void);
    void executeCallback(const r2_msgs::pid_followerGoalConstPtr &goal);
    void Yaw_Adjust(float target_angle,float now_angle);
    void Lockup_Point(geometry_msgs::Point target, geometry_msgs::Point current);

protected:
    void Angle_Limit(float *angle)
	{
		static uint8_t recursiveTimes = 0;
		recursiveTimes++;

		if(recursiveTimes<100)
		{
			if(*angle > 180.0f)
			{
				*angle -= 360.0f;
				Angle_Limit(angle);
			}
			else if(*angle < -180.0f)
			{
				*angle += 180.0f;
				Angle_Limit(angle);
			}
			else{}
		}

		recursiveTimes--;
	}

    geometry_msgs::Twist RobotMove_To_WorldMove(geometry_msgs::Twist target_robot_vel, float robot_yaw);
};
}