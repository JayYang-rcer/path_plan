#include "ros/ros.h"
#include "r2_hardware/communica.h"
#include "thread"

r2_msgs::controller_cmd msgs;
control_base::CONTROL_BASE base;
r2_msgs::stm32 upper;
ros::Subscriber sub;
ros::Publisher upper_pub;


void doMsg(r2_msgs::controller_cmd::ConstPtr Msg_p)
{
    msgs.cmd_vel.linear.x = Msg_p->cmd_vel.linear.x;
    msgs.cmd_vel.linear.y = Msg_p->cmd_vel.linear.y;
    msgs.cmd_vel.angular.z = Msg_p->cmd_vel.angular.z;
    msgs.chassis_ctrl_flag = Msg_p->chassis_ctrl_flag;
    msgs.next_controller_state = Msg_p->next_controller_state;
    msgs.take_ball_flag = Msg_p->take_ball_flag;
    // ROS_INFO("Send!!!");
}


void node1Function()
{
    ros::Rate rate(500);
    ros::NodeHandle nh;
    while(ros::ok())
    {
        ROS_INFO("Send!!!");
        base.UPPER_TO_STM32(msgs.next_controller_state,msgs.take_ball_flag);
        rate.sleep();
    }
   
}

void node2Function()
{
    ros::Rate rate(500);
    while (ros::ok())
    {
        base.ROS_READ_FROM_UPPER(upper.now_controller_state,upper.ball_state,
                                 upper.ball_color,upper.Huidu_flag,upper.color_cnt);
        upper_pub.publish(upper);
    }
    
}

void node3Function()
{
    ros::Rate rate(500);
    ros::NodeHandle nh;
    while(ros::ok())
    {
        base.CHASSIS_TO_STM32(msgs.cmd_vel.linear.x,msgs.cmd_vel.linear.y,
                                msgs.cmd_vel.angular.z,msgs.chassis_ctrl_flag,1);
        rate.sleep();
    }
   
}

int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc,argv,"test");
    ros::NodeHandle nh;
    
    std::string upper_serial_port_;
    std::string chassis_serial_port_;
    sub = nh.subscribe<r2_msgs::controller_cmd>("/r2_cmd",1,doMsg); //use to acieve all cmd from ros
    upper_pub = nh.advertise<r2_msgs::stm32>("/stm32",1);
    nh.param<std::string>("stm32_serial_port", upper_serial_port_, "/dev/stm32");
    nh.param<std::string>("chassis_serial_port", chassis_serial_port_, "/dev/chassis");

    base.ChassisSerialInit(chassis_serial_port_.data());
    base.SerialInit(upper_serial_port_.data());

    std::thread node1Thread(node1Function);         //upper
    std::thread node2Thread(node2Function);         //read
    std::thread node3Thread(node3Function);         //chassis
    ROS_INFO("open uart with stm32 successfully!\n");

    ros::spin();
    return 0;
}

