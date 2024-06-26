#include "hand_control/hand_control.h"
#include "r2_msgs/action_cmd.h"

#define PI 3.1415926

namespace hand_control_ns
{   
    HandControl::HandControl()
    {
        ros::NodeHandle nh_;
        //手柄话题订阅
        hand_sub_ = nh_.subscribe("ibus_data", 1, &HandControl::handCallback, this);
        //底盘速度指令发布
        twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        //偏航角消息订阅
        robot_coord_sub = nh_.subscribe("robot_coordinate", 1, &HandControl::robot_coord_callback, this);
        //发布路径点控制消息
        path_pub = nh_.advertise<r2_msgs::path_cmd>("path_cmd", 1);
        //控制器指令发布
        r2_cmd_pub_ = nh_.advertise<r2_msgs::controller_cmd>("r2_control", 1);   
    }

    HandControl::~HandControl()
    {
    }

    void HandControl::handCallback(const rc_msgs::IbusData::ConstPtr& msg)
    {
        last_time = ros::Time::now();

        if(msg->sw_a!=2)        //swa on top  close the status
        {
            twist_.linear.x = 0;
            twist_.linear.y = 0;
            twist_.angular.z = 0;
            twist_pub_.publish(twist_);

            path_msg_.PathMode = Hand;
            controller.chassis_ctrl_flag = 0;
            controller.next_controller_state = CONTROLLER_OFF;
        }
        else            //open the chassis
        {
            controller.chassis_ctrl_flag = 1;
        }

        if(msg->sw_a==2 && msg->sw_b==3)        //swa below and swb middle：hand control
        {
            path_msg_.PathMode = Hand;
            geometry_msgs::Twist hand_cmd;
            hand_cmd.linear.y = -msg->ch_l_x*2; //x方向的速度
            hand_cmd.linear.x = msg->ch_l_y*2;  //y方向的速度
            hand_cmd.angular.z = -msg->ch_r_x*2;
            
            double COS,SIN;
            COS = cos(robot.angular.z/180 *PI);
            SIN = sin(robot.angular.z/180 *PI);
            // ROS_INFO("%f",hand_cmd.angular.z);

            //世界坐标系转换
            //转化为转速
            twist_.linear.x  = (hand_cmd.linear.x * COS - hand_cmd.linear.y * SIN);
            twist_.linear.y  = -(hand_cmd.linear.x * SIN + hand_cmd.linear.y * COS);
            twist_.angular.z = hand_cmd.angular.z;
            twist_pub_.publish(twist_);

            if(msg->sw_d==2)        //swd below
            {
            //     if(msg->sw_c==1)    //filter ball
            //     {
            //         controller.next_controller_state = FILTER_BALL;
            //     }
            //     else if(msg->sw_c==3)   //take ball
            //     {
            //         controller.next_controller_state = TAKE_BALL;
            //         controller.take_ball_flag = NORMAL_TAKE_BALL;
            //     }
            //     else if(msg->sw_c==2)   //shoot ball 
            //     {
            //         controller.next_controller_state = SHOOT_BALL;
            //     }
            //     else    //unexpected situation
            //     {
            //         controller.next_controller_state = CONTROLLER_OFF;
            //     }
            // }
            // else    //swd on the top
            // {
            //     controller.next_controller_state = CONTROLLER_OFF;
            }
            
        }

        //atuo
        if(msg->sw_a==2&&msg->sw_b==2)
        {
            if(msg->sw_d==2)
            {
                switch (msg->sw_c)
                {
                    case 1:
                        path_msg_.PathMode = PATH1;
                        break;
                    
                    case 3:
                        path_msg_.PathMode = PATH2;
                        break;
                    
                    case 2:
                        take_ball.robot_decision(path_msg_,controller);
                        break;

                    default:
                        break;
                }
            }
        }
    }

    void HandControl::run()
    {
        ros::Rate loop_rate(100);
        while (ros::ok())
        {
            // ROS_INFO("ok");
            path_pub.publish(path_msg_);
            r2_cmd_pub_.publish(controller);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void HandControl::robot_coord_callback(const geometry_msgs::Point::ConstPtr& msg)
    {
        robot.angular.z = msg->z;
    }
}