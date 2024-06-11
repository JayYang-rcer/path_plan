#include "r2_path/trackers/pid_follower.h"

namespace tracker
{
    pid_follower::pid_follower(std::string name) : as_(nh, name, boost::bind(&pid_follower::executeCallback, this, _1), false), pid_follower_action(name)
    {
        if(!nh.param<std::string>("/path_pid/odom_topic",odom_topic,"/odom"))
            ROS_ERROR("GET PARAM ERROR!!!");
        if(!nh.param<float>("control_frequence",control_frequence,0))
            ROS_ERROR("GET CONTROL FREQUENCE FAILED");
        if(!nh.param<float>("path_pid/LockupPoint/x_max",LockupPoint_xMax,0)||
           !nh.param<float>("path_pid/LockupPoint/y_max",LockupPoint_yMax,0)||
           !nh.param<float>("path_pid/LockupPoint/yaw_max",LockupPoint_YawMax,0))
            ROS_ERROR("GET LOCKUP POINT ERROR!!!");
        as_.start();
        robot_pose_sub = nh.subscribe<geometry_msgs::Point>(odom_topic,1,&pid_follower::robot_pose_callback,this);
        control_timer = nh.createTimer(ros::Duration(0.05),&pid_follower::control_timer_callback,this);
        vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
        pid_init();
        path_flag=0;
    }

    pid_follower::~pid_follower()
    {
    }

    void pid_follower::robot_pose_callback(const geometry_msgs::Point::ConstPtr &msg)
    {
        robot_pose = *msg;
    }

    void pid_follower::pid_init(void)
    {
        if(!nh.param<float>("/path_pid/xy_deadzone",xy_deadzone,0)||!nh.param<float>("/path_pid/yaw_deadzone",yaw_deadzone,0))
            ROS_ERROR("PATH DEAD ZONE INIT ERROR!!!"); 
        float Kp_x, Ki_x, Kd_x, dead_zone_x, Out_Max_x;
        if(!nh.param<float>("/path_pid/path1_pointX_pid/p",Kp_x,0)||
            !nh.param<float>("/path_pid/path1_pointX_pid/i",Ki_x,0)||
            !nh.param<float>("/path_pid/path1_pointX_pid/d",Kd_x,0)||
            !nh.param<float>("/path_pid/path1_pointX_pid/out_max",Out_Max_x,0)||
            !nh.param<float>("/path_pid/path1_pointX_pid/dead_zone",dead_zone_x,-1)||
            !nh.param<float>("/path_pid/path1_pointX_pid/pre_feed",pid_x_PreFeed,0))
            ROS_ERROR("POINT_X PID INIT ERROR!!!");
        else
        {
            //进行PID初始化
            pid_x.PID_Param_Init(Kp_x,Ki_x,Kd_x,0,Out_Max_x);
            pid_x.DeadZone = dead_zone_x;
            pid_x.D_of_Current = true;
            pid_x.Imcreatement_of_Out = false;
            pid_x.I_SeparThresh = 400;
            pid_x.LowPass_error.Trust = 0.7;
            pid_x.LowPass_d_err.Trust = 0.3;
        }
                
        float Kp_y, Ki_y, Kd_y, dead_zone_y, Out_Max_y;
        if(!nh.param<float>("/path_pid/path1_pointY_pid/p",Kp_y,0)||
            !nh.param<float>("/path_pid/path1_pointY_pid/i",Ki_y,0)||
            !nh.param<float>("/path_pid/path1_pointY_pid/d",Kd_y,0)||
            !nh.param<float>("/path_pid/path1_pointY_pid/out_max",Out_Max_y,0)||
            !nh.param<float>("/path_pid/path1_pointY_pid/dead_zone",dead_zone_y,-1)||
            !nh.param<float>("/path_pid/path1_pointY_pid/pre_feed",pid_y_PreFeed,0))
            ROS_ERROR("POINT_Y PID INIT ERROR!!!");
        else
        {
            //进行PID初始化
            pid_y.PID_Param_Init(Kp_y,Ki_y,Kd_y,0,Out_Max_y);
            pid_y.DeadZone = dead_zone_y;
            pid_y.D_of_Current = true;
            pid_y.Imcreatement_of_Out = false;
            pid_y.I_SeparThresh = 400;
            pid_y.LowPass_error.Trust = 0.7;	
            pid_y.LowPass_d_err.Trust = 0.3;
        }

        float Kp_yaw, Ki_yaw, Kd_yaw, dead_zone_yaw, Out_Max_yaw;
        if(!nh.param<float>("/path_pid/path1_Yaw_pid/p",Kp_yaw,0)||
            !nh.param<float>("/path_pid/path1_Yaw_pid/i",Ki_yaw,0)||
            !nh.param<float>("/path_pid/path1_Yaw_pid/d",Kd_yaw,0)||
            !nh.param<float>("/path_pid/path1_Yaw_pid/out_max",Out_Max_yaw,0)||
            !nh.param<float>("/path_pid/path1_Yaw_pid/dead_zone",dead_zone_yaw,-1)||
            !nh.param<float>("/path_pid/path1_Yaw_pid/pre_feed",pid_yaw_PreFeed,0))
            ROS_ERROR("YAW PID INIT ERROR!!!");
        else
        {
            //进行PID初始化
            pid_yaw.PID_Param_Init(Kp_yaw,Ki_yaw,Kd_yaw,0,Out_Max_yaw);
            pid_yaw.DeadZone = dead_zone_yaw;
            pid_yaw.D_of_Current = true;
            pid_yaw.Imcreatement_of_Out = false;
            pid_yaw.I_SeparThresh = 400;
            pid_yaw.LowPass_error.Trust = 0.7;
            pid_yaw.LowPass_d_err.Trust = 0.3;
        }
    }
}