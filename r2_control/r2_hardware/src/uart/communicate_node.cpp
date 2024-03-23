#include "ros/ros.h"
#include "r2_hardware/communica.h"
#include "thread"

r2_msgs::controller_cmd msgs;
control_base::CONTROL_BASE base;
r2_msgs::stm32 stm32;
ros::Subscriber sub;
ros::Publisher stm32_pub;


void doMsg(r2_msgs::controller_cmd::ConstPtr Msg_p)
{
    msgs.cmd_vel.linear.x = Msg_p->cmd_vel.linear.x;
    msgs.cmd_vel.linear.y = Msg_p->cmd_vel.linear.y;
    msgs.cmd_vel.angular.z = -Msg_p->cmd_vel.angular.z;
    msgs.chassis_ctrl_flag = Msg_p->chassis_ctrl_flag;
    msgs.next_bp_state = Msg_p->next_bp_state;
    msgs.next_fw_state = Msg_p->next_fw_state;
    
    // ROS_INFO("Send!!!");
}


void node1Function()
{
    ros::Rate rate(50);
    ros::NodeHandle nh;
    while(ros::ok())
    {
        base.ROS_WRITE_TO_STM32(msgs.cmd_vel.linear.x,msgs.cmd_vel.linear.y,
                                msgs.cmd_vel.angular.z,msgs.chassis_ctrl_flag,
                                msgs.next_bp_state,msgs.next_fw_state,msgs.is_fw_open_flag);
        rate.sleep();
    }
   
}

void node2Function()
{
    ros::Rate rate(50);
    while (ros::ok())
    {
        base.ROS_READ_FROM_STM32(stm32.current_bp_state,stm32.current_fw_state,
                        stm32.is_ball_in_car,stm32.color_flag1,
                        stm32.color_flag2,stm32.color_flag3);

        stm32_pub.publish(stm32);
    }
    
}

int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc,argv,"test");
    ros::NodeHandle nh;
    
    std::string stm32_serial_port_;
    sub = nh.subscribe<r2_msgs::controller_cmd>("/r2_cmd",1,doMsg); //use to acieve all cmd from ros
    stm32_pub = nh.advertise<r2_msgs::stm32>("/stm32",1);
    nh.param<std::string>("stm32_serial_port", stm32_serial_port_, "/dev/ttyUSB0");
    base.SerialInit(stm32_serial_port_.data());

    std::thread node1Thread(node1Function);
    std::thread node2Thread(node2Function);

    ROS_INFO("open uart with stm32 successfully!\n");

    ros::spin();
    return 0;
}

// int main(int argc, char *argv[])
// {
//     /* code */
//     ros::init(argc,argv,"test");
//     ros::NodeHandle nh;
//     control_base::CONTROL_BASE base;
    
//     std::string stm32_serial_port_;
//     nh.param<std::string>("stm32_serial_port", stm32_serial_port_, "/dev/ttyUSB0");
//     base.SerialInit(stm32_serial_port_.data());

//     uint8_t if_use_auto=0;
//     float if_use_path=0,RealAngle_1,RealAngle_3;
//     plumbing_ros_32::stm32 stm32;
//     ros::Subscriber sub = nh.subscribe<plumbing_ros_32::r2_controller_msg>("/r2_cmd",1,doMsg); //use to acieve all cmd from ros

//     ros::Rate rate(100);
//     ROS_INFO("open uart with stm32 successfully!\n");

//     while(ros::ok())
//     {
//         base.ROS_WRITE_TO_STM32(msgs.cmd_vel.linear.x,msgs.cmd_vel.linear.y,
//                                 msgs.cmd_vel.angular.z,msgs.chassis_ctrl_flag,
//                                 msgs.next_bp_state,msgs.next_fw_state); 

//         // base.ROS_READ_FROM_STM32(RealAngle_1,RealAngle_3,if_use_path,if_use_auto);
        
//         ros::spinOnce();
//         rate.sleep();
//     }
//     return 0;
// }

