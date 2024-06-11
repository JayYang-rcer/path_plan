#ifndef A3E5C406_AAE2_4EFC_847C_9AE5663AD480
#define A3E5C406_AAE2_4EFC_847C_9AE5663AD480
#include "geometry_msgs/Twist.h"
#include "r2_msgs/controller_cmd.h"
#include "ros/ros.h"

class uart_stm32
{
private:
    /* data */
    r2_msgs::controller_cmd controller;
    ros::Time last_msg_time;
    int if_use_gazebo;
    ros::NodeHandle nh;
    ros::Subscriber chassis_sub;
    ros::Subscriber upper_sub;
    ros::Publisher cmd_pub;
    ros::Timer timer;

public:
    uart_stm32(/* args */);
    ~uart_stm32();
    void run();
    void do_vel_Msg(const geometry_msgs::Twist::ConstPtr &msg_p);
    void control_callback(const r2_msgs::controller_cmd::ConstPtr &msg_p);
    void timer_callback(const ros::TimerEvent&);
};


#endif /* A3E5C406_AAE2_4EFC_847C_9AE5663AD480 */
