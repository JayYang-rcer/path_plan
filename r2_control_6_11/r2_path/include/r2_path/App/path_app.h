#pragma once
#include "ros/ros.h"
#include "r2_path/path_plan/path_base.h"
#include "r2_msgs/pid_followerAction.h"
#include "r2_msgs/path_status.h"
#include <actionlib/client/simple_action_client.h>

namespace path_app_ns
{
class path_app : public math_ns::math
{
public:
    path_app();
    path_plan path;

private:
    ros::NodeHandle nh;
    ros::Subscriber path_cmd_sub,robot_pose_sub;
    ros::Publisher path_status_pub;
    actionlib::SimpleActionClient<r2_msgs::pid_followerAction> client;
    r2_msgs::pid_followerGoal goal;

    PATH_ENUM path_mode,last_mode;
    AUTO_TAKE_PUT takeput_mode,last_takeput_mode;
    PATH_STATUS path_status;
    r2_msgs::path_status path_status_msg;
    float ball_distance_error;
    float takeBall_ChangeDistance;

    geometry_msgs::Point robot_pose;
    geometry_msgs::PoseStamped start_pose,end_pose;
    geometry_msgs::Point silo_one,silo_two,silo_three,silo_four,silo_five;
    geometry_msgs::Point last_ball_pose;

    void path_cmd_cb(const r2_msgs::path_cmd::ConstPtr& msg);
    void robot_pose_callback(const geometry_msgs::Point::ConstPtr &msg);
    void active_cb();
	void feedback_cb(const r2_msgs::pid_followerFeedbackConstPtr &feedback);
	void done_cb(const actionlib::SimpleClientGoalState& state, const r2_msgs::pid_followerResultConstPtr &result);
};
}
