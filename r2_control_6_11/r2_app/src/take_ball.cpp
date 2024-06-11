#include "r2_app/take_ball.h"

namespace take_ball_ns
{
take_ball::take_ball()
{
    cameraData_sub = nh.subscribe("/ball_position", 1, &take_ball::cameraDataCallback, this);
    robotPos_sub = nh.subscribe("/robot_coordinate", 1, &take_ball::robotPosCallback, this);
    path_status_sub = nh.subscribe("/path_status", 1, &take_ball::pathStatusCallback, this);
    controller_status_sub = nh.subscribe("/stm32", 1, &take_ball::controllerStatusCallback, this);

    if(!nh.param<bool>("/take_ball/if_use_decision", if_use_decision, false))
        ROS_ERROR("GET if_use_decision FAILED");
    if(!nh.param<std::string>("/take_ball/decision_topic", decision_topic, "decision_topic"))
        ROS_ERROR("GET decision_topic FAILED");
    if(!nh.param<std::string>("/take_ball/zone_decision_topic", zone_decision_topic, "zone_decision_topic"))
        ROS_ERROR("GET zone_decision_topic FAILED");
    if(!nh.param<float>("take_ball/y_max", y_max, 12)||
       !nh.param<float>("take_ball/y_min", y_min, 8)||
       !nh.param<float>("take_ball/x_abs_max", x_abs_max, 6))
        ROS_ERROR("GET y_max,y_min,x_abs_max FAILED");
    if(!nh.param<std::string>("field_direction", field_direction, "right"))
        ROS_ERROR("GET field_direction FAILED");

    if(!nh.param<bool>("if_use_debug", if_use_debug, true))
        ROS_ERROR("GET if_use_debug FAILED");

    if(field_direction=="left")
    {
        mirror_flag = -1;
    }
    else if(field_direction=="right")
    {
        mirror_flag = 1;
    }
    else{}

    decision_sub = nh.subscribe(decision_topic, 1, &take_ball::decisionCallback, this);
    // zone_decision_sub = nh.subscribe(zone_decision_topic, 1, &take_ball::decisionCallback, this);
}

void take_ball::robot_decision(r2_msgs::path_cmd &path_cmd,r2_msgs::controller_cmd &controller_cmd)
{
    float ball_distance = sqrt(pow(ball_position.x-robot_position.x,2)+pow(ball_position.y-robot_position.y,2));
    if(if_use_debug)
    {
        if(decision.competition_status==0)
        {
            path_cmd.PathMode = TAKE_BALLS;
            take_ball_planner(path_cmd, controller_cmd);
            put_ball_planner(path_cmd, controller_cmd);
        }
        else
        {
            path_cmd.PathMode = Hand;
            controller_cmd.next_controller_state = CONTROLLER_OFF;
            controller_cmd.chassis_ctrl_flag = 0;
        }

        if(path_cmd.take_or_put==Take&&controller_status.ball_state==BALL_OUTSIDE)
        {
            if(ball_distance<1)
            {
                path_cmd.PathMode = LockupPoint_e;
                path_cmd.ball_position.z = 0;
                camera_data_first = 1;
            }
        }
    }
    else
    {
        if(controller_status.Huidu_flag == 1)
        {
            
        }
    }

    if(abs(robot_position.x)>6 || robot_position.y>12 || robot_position.y<0)
    {
        path_cmd.PathMode = STOP;
    }
}

//机器人跟踪球的方案
/*
1、使用一个定时器，每隔一段时间发布球的位置，更新机器人的路径(没有局部路径规划，只有全局路径规划，PASS)
2、当机器人距离球的距离小于一定值时，机器人进入PD的点对点跟踪状态(退而求其次，盘他)
*/
void take_ball::take_ball_planner(r2_msgs::path_cmd &path_cmd,r2_msgs::controller_cmd &controller_cmd)
{
    if(!if_use_decision)
    {
        ball_position.x = -5.5;
        ball_position.y = 10;
        ball_position.z = 90;
        path_cmd.PathMode = TAKE_BALLS;
    }

    if(abs(ball_position.x)<6 && (ball_position.y>8 && ball_position.y<12))
    {
        //取球策略有待修改，样条路径终点的设定。到某个点后，再进行纯跟踪取球
        //在取球的过程中，如果发现球的视野消失，如何处理
        //取球过程中，球滚到边缘怎么办
        if(controller_status.ball_state == BALL_OUTSIDE && camera_data_first==1)
        {
            camera_data_first = 0;
            ROS_INFO("get ball position");
            //球的初始位置在场地边缘
            if(ball_position.y < y_min || ball_position.y > y_max)
            {
                //留出变量接口，以便后续修改。放到ymal中
                if(ball_position.y>y_max)
                {
                    path_cmd.mid_position.x = 2*mirror_flag;
                    path_cmd.mid_position.y = 11.5;
                    path_cmd.mid_position.z = 0;
                    controller_cmd.take_ball_flag = LEFT_TAKE_BALL;
                }
                
                if(ball_position.y < y_min)
                {
                    path_cmd.mid_position.x = 2*mirror_flag;
                    path_cmd.mid_position.y = 8.5;
                    path_cmd.mid_position.z = 0;
                    controller_cmd.take_ball_flag = RIGHT_TAKE_BALL;
                }

                path_cmd.ball_position.z = 0;
            }
            else    //球的初始位置在场地中间
            {
                //中间点不进行赋值，路径为直线
                path_cmd.mid_position.x = 0;
                path_cmd.mid_position.y = 0;
                path_cmd.mid_position.z = 0;
                path_cmd.ball_position.z = atan2(ball_position.y-robot_position.y, ball_position.x-robot_position.x);
                controller_cmd.take_ball_flag = NORMAL_TAKE_BALL;
            }

            controller_cmd.next_controller_state = TAKE_BALL;
            path_cmd.ball_position.x = ball_position.x;
            path_cmd.ball_position.y = ball_position.y;
            path_cmd.take_or_put = Take;
            // ROS_INFO("take ball status");
        }
    }
}


void take_ball::put_ball_planner(r2_msgs::path_cmd &path_cmd,r2_msgs::controller_cmd &controller_cmd)
{
    camera_data_first = 1;
    if(controller_status.ball_state == BALL_HAVE_TAKEN||controller_status.ball_state == BALL_IN_CLAW)
    {
        path_cmd.take_or_put = Put;
        if(if_use_decision)
        {
            path_cmd.silo_num = decision.silo_num;
        }
        else
        {
            path_cmd.PathMode = TAKE_BALLS;
            path_cmd.silo_num = 3;
        }
        // ROS_INFO("put ball status");
    }

    if(now_path_status.now_takeput_mode == Put&&now_path_status.destination_arrived==SUCCCEED)
    {
        if(controller_status.ball_state == BALL_IN_CLAW)
            controller_cmd.next_controller_state = SHOOT_BALL;
        // ROS_INFO("put ball status");
    }
}

void take_ball::decisionCallback(const r2_msgs::decision::ConstPtr& msg)
{
    decision = *msg;
}

void take_ball::cameraDataCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    ball_position = *msg;
    ROS_INFO("x:%f, y:%f", msg->x, msg->y);
}

void take_ball::robotPosCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    robot_position = *msg;
}

void take_ball::pathStatusCallback(const r2_msgs::path_status::ConstPtr& msg)
{
    now_path_status = *msg;
}

void take_ball::controllerStatusCallback(const r2_msgs::stm32::ConstPtr& msg)
{
    controller_status = *msg;
}

} // namespace name
