#ifndef HAND_CONTROL_H 
#define HAND_CONTROL_H

#include <ros/ros.h>
#include "rc_msgs/IbusData.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "r2_msgs/pid_info.h"
#include "r2_msgs/controller_cmd.h"
#include "math.h"
#include "r2_msgs/path_cmd.h"

enum PATH_ENUM
{
    Hand=0,
	LockupPoint_e=1,
	PATH1=2,
	PATH2=3,
	TAKE_BALL
};

typedef enum
{
    FW_INVERTED=2,  //摩擦轮反转
    FW_TAKE_BALL,   //摩擦轮取球
    FW_CONTROLLER_OFF,
    BP_SHOOT_BALL,  //滚筒出球
    BP_ABANDON_BALL,//滚筒筛球
	BP_INVERTED,    //滚筒反转
    BP_CONTROLLER_OFF,  //机构关闭

    LINEAR_ACTUATOR_GO,
    LINEAR_ACTUATOR_BACK,
    LINEAR_ACTUATOR_OFF
}CONTROLLER_STATE;

namespace hand_control_ns
{
class HandControl
{
public:
    HandControl();
    ~HandControl();
    void handCallback(const rc_msgs::IbusData::ConstPtr& msg);
    void robot_coord_callback(const geometry_msgs::Point::ConstPtr& msg);
    void timer_callback(const ros::TimerEvent& event);
    void run();

private:
    ros::NodeHandle nh_;
    ros::Subscriber hand_sub_;
    ros::Publisher twist_pub_;
    ros::Subscriber robot_coord_sub;
    ros::Publisher path_;
    ros::Publisher r2_cmd_pub_;
    ros::Timer timer;
    ros::Time last_time;
    geometry_msgs::Twist robot; //use to remember the robot's yaw
    geometry_msgs::Twist twist_;
    r2_msgs::path_cmd path_msg_;
    r2_msgs::controller_cmd controller;
};
}

#endif