#include <ros/ros.h>
#include "rc_msgs/ChassisCmd.h"

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "publisher_node");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建发布者
    ros::Publisher publisher = nh.advertise<rc_msgs::ChassisCmd>
    ("/controllers/chassis_controller/cmd_chassis", 50);

    // 设置发布频率
    ros::Rate rate(50);

    while (ros::ok())
    {
        // 创建并填充消息
        rc_msgs::ChassisCmd msg;

        // 在msg中设置所需的数据
        msg.accel.angular.z = 4;
        msg.accel.linear.x = 3;
        msg.accel.linear.y = 3;
        // 发布消息
        publisher.publish(msg);

        // 处理回调函数
        ros::spinOnce();

        // 以指定频率进行循环
        rate.sleep();
    }

    return 0;
}
