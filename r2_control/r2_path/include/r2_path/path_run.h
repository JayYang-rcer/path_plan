#ifndef __PLAN_RUN_H
#define __PLAN_RUN_H

#include "r2_path/path_base.h"

class path_run : public math_ns::math
{
private:
    ros::NodeHandle nh;
    ros::Subscriber robot_coord_sub;
    ros::Publisher vel_pub_;
    ros::Subscriber ball_pos_sub;
    ros::Publisher target_vel_pub;

    float control_cycle;
    int path_res;
    int recieve_ball_flag;
    float move_time_counter=0;
    float ball_pos_X[7],ball_pos_Y[7],ball_pos_Yaw[7];
    float take_ball_time,take_ball_move_speed;
    int robot_start_pos_flag;
    float ball_distance;
    float final_distance;
    PATH_ENUM mode,last_mode,take_put_mode;
    float path_cos, path_sin;
    int take_put_arrive_flag;

    geometry_msgs::Twist robot_vel;
    geometry_msgs::Point robot_pos;
    geometry_msgs::Point robot_start_pos,last_ball_pos,target_ball_pos;
    geometry_msgs::Point last_target_ball_pos;
    geometry_msgs::Point pos_silo_one,pos_silo_two,pos_silo_three,pos_silo_four,pos_silo_five;
    void robot_coord_callback(const geometry_msgs::Point::ConstPtr& pos);

    float Target_Lockup_X,Target_Lockup_Y,Target_Lockup_YAW;
    float path1_control_num,path1_t;
    float path2_control_num,path2_t;
    std::vector<float> X_TEST1,Y_TEST1,Yaw_TEST1;
    std::vector<float> X_TEST2,Y_TEST2,Yaw_TEST2;
    double COS,SIN;
    float X0[8],Y0[8],Yaw0[8];
	float X1[8],Y1[8],Yaw1[8];

public:
    path_run();
    void run();
    path_base_ns::path_base path;
    void path_cmd_callback(const r2_msgs::path_cmd::ConstPtr& path_cmd);
    void path_stauts(r2_msgs::path_cmd cmd);
    void fsm_path();
    void dynamic_curve();
};

#endif