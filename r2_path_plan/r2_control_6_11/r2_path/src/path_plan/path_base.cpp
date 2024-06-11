#include "r2_path/path_plan/path_base.h"

path_plan::path_plan(/* args */) : client(nh, "pid_follower", true)
{
    path_pub = nh.advertise<nav_msgs::Path>("/trajectory",1,true);
    X0.reserve(20); Y0.reserve(20); Yaw0.reserve(20);
	X1.reserve(20); Y1.reserve(20); Yaw1.reserve(20);
    BALL_X.reserve(20); BALL_Y.reserve(20); BALL_Yaw.reserve(20);

    if(!nh.param<float>("/path_plan/takeBall_ChangeDistance",ball_distance_error,0))
        ROS_ERROR("GET BALL DISTANCE ERROR FAILED");
    if(!nh.param<float>("/path_plan/take_ball_move_speed",takeBall_speed,0.0f))
        ROS_ERROR("GET TAKE BALL SPEED FAILED");
    if(!nh.param<std::string>("/field_direction",field_direction,"left"))
        ROS_ERROR("GET FIELD DIRECTION ERROR!!!");
    if(!nh.param<std::string>("/path_plan/odom_topic",odom_topic,"/odom"))
        ROS_ERROR("GET ODOM TOPIC ERROR!!!");
    if(!nh.param<float>("control_frequence",control_frequence,0))
        ROS_ERROR("GET CONTROL FREQUENCE FAILED");

	if(!nh.param<float>("/path_plan/path1/path_total_points",path1_total_points,0)||
	   !nh.param<std::vector<float>>("/path_plan/path1/path_pointX",X0,{0})||
	   !nh.param<std::vector<float>>("/path_plan/path1/path_pointY",Y0,{0})||
	   !nh.param<std::vector<float>>("/path_plan/path1/path_YAW",Yaw0,{0})||
	   !nh.param<float>("/path_plan/path2/path_total_points",path2_total_points,0)||
	   !nh.param<std::vector<float>>("/path_plan/path2/path_pointX",X1,{0})||
	   !nh.param<std::vector<float>>("/path_plan/path2/path_pointY",Y1,{0})||
	   !nh.param<std::vector<float>>("/path_plan/path2/path_YAW",Yaw1,{0}))
	   	ROS_ERROR("GET PATH1 POINT ERROR!!!");
    else
    {
        path1_control_num = X0.size() - 4;
        path2_control_num = X1.size() - 4;
        if(field_direction == "left")
        {
            for(int i=0;i<X0.size();i++)
                X0[i] = -X0[i];
            for(int i=0;i<X1.size();i++)
                X1[i] = -X1[i];
        }
    }
}

int path_plan::take_ball_planner(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped middle_pose, geometry_msgs::PoseStamped end_pose)
{
    tf::Quaternion quaternion_start,quaternion_end;
    geometry_msgs::Quaternion buffer_start,buffer_end;
    double Yaw_start,Yaw_end,roll,pitch;
    BALL_X.clear(); BALL_Y.clear(); BALL_Yaw.clear();
    
    start_pose_ = start_pose; end_pose_ = end_pose;
    buffer_start = start_pose.pose.orientation;
    buffer_end = end_pose.pose.orientation;
    tf::quaternionMsgToTF(buffer_start,quaternion_start);
    tf::quaternionMsgToTF(buffer_end,quaternion_end);
    tf::Matrix3x3(quaternion_start).getRPY(roll,pitch,Yaw_start);
    tf::Matrix3x3(quaternion_end).getRPY(roll,pitch,Yaw_end);
    Yaw_end = Yaw_end/PI*180.00;
    Yaw_start = Yaw_start/PI*180.00;

    float distance = distance_of_two_point(start_pose.pose.position.x,start_pose.pose.position.y,
                                           end_pose.pose.position.x,end_pose.pose.position.y);
                            
    float COS_PATH = abs(start_pose.pose.position.x-end_pose.pose.position.x)/distance;
    float SIN_PATH = abs(start_pose.pose.position.y-end_pose.pose.position.y)/distance;
    float p=0.2;
    // float COS_PATH = 0; float SIN_PATH = 0;

    //让每个方向上的加减量是均匀的，要乘以一个系数，系数应该是起点与终点的偏角的cos值
    if(start_pose.pose.position.x > end_pose.pose.position.x)
    {
        BALL_X[0] = (start_pose.pose.position.x + p*COS_PATH);
        BALL_X[1] = start_pose.pose.position.x;
        BALL_X[2] = (start_pose.pose.position.x - p*COS_PATH);

        // BALL_X[3] = (start_pose.pose.position.x + end_pose.pose.position.x)/2;
        BALL_X[3] = middle_pose.pose.position.x;

        BALL_X[4] = (end_pose.pose.position.x + p*COS_PATH);
        BALL_X[5] = end_pose.pose.position.x;
        BALL_X[6] = (end_pose.pose.position.x - p*COS_PATH);
    }
    else
    {
        BALL_X[0] = (start_pose.pose.position.x - p*COS_PATH);
        BALL_X[1] = start_pose.pose.position.x;
        BALL_X[2] = (start_pose.pose.position.x + p*COS_PATH);

        // BALL_X[3] = (start_pose.pose.position.x + end_pose.pose.position.x)/2;
        BALL_X[3] = middle_pose.pose.position.x;

        BALL_X[4] = (end_pose.pose.position.x - p*COS_PATH);
        BALL_X[5] = end_pose.pose.position.x;
        BALL_X[6] = (end_pose.pose.position.x + p*COS_PATH);
    }

    if(start_pose.pose.position.y > end_pose.pose.position.y)
    {
        BALL_Y[0] = (start_pose.pose.position.y + p*SIN_PATH);
        BALL_Y[1] = start_pose.pose.position.y;
        BALL_Y[2] = (start_pose.pose.position.y - p*SIN_PATH);

        // BALL_Y[3] = (start_pose.pose.position.y + end_pose.pose.position.y)/2;
        BALL_Y[3] = middle_pose.pose.position.y;

        BALL_Y[4] = (end_pose.pose.position.y + p*SIN_PATH);
        BALL_Y[5] = end_pose.pose.position.y;
        BALL_Y[6] = (end_pose.pose.position.y - p*SIN_PATH);
    }
    else
    {
        BALL_Y[0] = (start_pose.pose.position.y - p*SIN_PATH);
        BALL_Y[1] = start_pose.pose.position.y;
        BALL_Y[2] = (start_pose.pose.position.y + p*SIN_PATH);

        // BALL_Y[3] = (start_pose.pose.position.y + end_pose.pose.position.y)/2;
        BALL_Y[3] = middle_pose.pose.position.y;
        BALL_Y[4] = (end_pose.pose.position.y - p*SIN_PATH);
        BALL_Y[5] = end_pose.pose.position.y;
        BALL_Y[6] = (end_pose.pose.position.y + p*SIN_PATH);
    }

    if(Yaw_start > Yaw_end)
    {
        BALL_Yaw.push_back(Yaw_start + 0.005);
        BALL_Yaw.push_back(Yaw_start);
        BALL_Yaw.push_back(Yaw_start - 0.005);

        // BALL_Yaw.push_back((Yaw_start + Yaw_end)/2);
        BALL_Yaw.push_back(middle_pose.pose.orientation.z*180.00/PI);

        BALL_Yaw.push_back(Yaw_end + 0.005);
        BALL_Yaw.push_back(Yaw_end);
        BALL_Yaw.push_back(Yaw_end - 0.005);
    }
    else
    {
        BALL_Yaw.push_back(Yaw_start - 0.005);
        BALL_Yaw.push_back(Yaw_start);
        BALL_Yaw.push_back(Yaw_start + 0.005);

        // BALL_Yaw.push_back((Yaw_start + Yaw_end)/2);
        BALL_Yaw.push_back(middle_pose.pose.orientation.z*180.00/PI);

        BALL_Yaw.push_back(Yaw_end - 0.005);
        BALL_Yaw.push_back(Yaw_end);
        BALL_Yaw.push_back(Yaw_end + 0.005);
    }

    return 0;
}

void path_plan::lockup_point(geometry_msgs::Point target)
{
    lockup_pose.header.frame_id = "map";
    lockup_pose.header.stamp = ros::Time::now();
    lockup_pose.pose.position.x = target.x;
    lockup_pose.pose.position.y = target.y;
    lockup_pose.pose.orientation = tf::createQuaternionMsgFromYaw(target.z*3.1415/180.0f);
}

void path_plan::planner_fsm()
{
    switch (path_mode)
    {
        case LockupPoint_e:
        {
            geometry_msgs::PoseStamped pose;
            path_msg.header.frame_id = "map";
            path_msg.header.stamp = ros::Time::now();
            path_msg.poses.clear();
            path_msg.poses.push_back(lockup_pose);
            break;
        }

        case PATH1:
        {
            ThreeB_Curve(path1_control_num,X0,Y0,Yaw0,path1_total_points);
            break;
        }

        case PATH2:
        {
            ThreeB_Curve(path2_control_num,X1,Y1,Yaw1,path2_total_points);
            break;
        }

        case auto_take_put:
        {
            int point_num = takeBall_calculate(start_pose_,end_pose_)*control_frequence;
            // ROS_INFO("point_num: %f",control_frequence);
            ThreeB_Curve(3,BALL_X,BALL_Y,BALL_Yaw,point_num);
            break;
        }
        
        default:
            break;
    }
    path_pub.publish(path_msg);
}


float path_plan::takeBall_calculate(geometry_msgs::PoseStamped start_pos, geometry_msgs::PoseStamped end_pos)
{
    float distance = distance_of_two_point(start_pos.pose.position.x,start_pos.pose.position.y,
                                           end_pos.pose.position.x,end_pos.pose.position.y);
    float use_time = distance/takeBall_speed;

    return use_time;
}
