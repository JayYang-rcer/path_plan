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
#include "r2_app/take_ball.h"

// enum PATH_ENUM
// {
//     Hand=0,
// 	LockupPoint_e=1,
// 	PATH1=2,
// 	PATH2=3,
// 	TAKE_BALLS
// };

// typedef enum
// {
//     TAKE_BALL =2,   //取球
//     FILTER_BALL,    //筛球
//     SHOOT_BALL,     //出球
//     CONTROLLER_OFF, //关闭
//     CONTROLLER_ERROR    //错误状态
// }CONTROLLER_STATE;

// typedef enum
// {
//     NORMAL_TAKE_BALL = 0,   //正常吸球
//     LEFT_TAKE_BALL,         //左吸球
//     RIGHT_TAKE_BALL         //右吸球
// }TAKE_BALL_STATE;

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
    ros::Publisher path_pub;
    ros::Publisher r2_cmd_pub_;
    ros::Timer timer;
    ros::Time last_time;
    geometry_msgs::Twist robot; //use to remember the robot's yaw
    geometry_msgs::Twist twist_;
    r2_msgs::path_cmd path_msg_;
    r2_msgs::controller_cmd controller;
    r2_msgs::controller_cmd auto_controller;

    take_ball_ns::take_ball take_ball;
};
}

#endif