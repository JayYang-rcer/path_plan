#include "r2_path/App/path_app.h"
#include <r2_path/path_plan/path_base.h>

namespace path_app_ns
{
path_app::path_app() : client("pid_follower",true)
{
    client.waitForServer();
    path_cmd_sub = nh.subscribe<r2_msgs::path_cmd>("/path_cmd",1,&path_app::path_cmd_cb,this);
    robot_pose_sub = nh.subscribe<geometry_msgs::Point>("/robot_coordinate",1,&path_app::robot_pose_callback,this);
    path_status_pub = nh.advertise<r2_msgs::path_status>("/path_status",1);
    path_status = SUCCCEED; path.path_mode=Hand;

    if(!nh.param<float>("/path_plan/takeBall_ChangeDistance",ball_distance_error,0))
        ROS_ERROR("GET BALL DISTANCE ERROR FAILED");

    std::string field_direction; int mirror_flag;
    if(!nh.param<std::string>("field_direction",field_direction,"rigth"))
        ROS_ERROR("GET FIELD DIRECTION FAILED");
    if(field_direction == "left")
        mirror_flag = -1;
    else
        mirror_flag = 1;

    if(!nh.param<double>("path_plan/silo_coordinate/silo1/x",silo_one.x,0.5)||
       !nh.param<double>("path_plan/silo_coordinate/silo1/y",silo_one.y,8.5)||
       !nh.param<double>("path_plan/silo_coordinate/silo1/z",silo_one.z,0))
        ROS_ERROR("GET SILO1 COORDINATE FAILED");
    if(!nh.param<double>("path_plan/silo_coordinate/silo2/x",silo_two.x,0.5)||
         !nh.param<double>("path_plan/silo_coordinate/silo2/y",silo_two.y,9.25)||
         !nh.param<double>("path_plan/silo_coordinate/silo2/z",silo_two.z,0))
          ROS_ERROR("GET SILO2 COORDINATE FAILED");
    if(!nh.param<double>("path_plan/silo_coordinate/silo3/x",silo_three.x,0.5)||
        !nh.param<double>("path_plan/silo_coordinate/silo3/y",silo_three.y,10)||
        !nh.param<double>("path_plan/silo_coordinate/silo3/z",silo_three.z,0))
        ROS_ERROR("GET SILO3 COORDINATE FAILED");
    if(!nh.param<double>("path_plan/silo_coordinate/silo4/x",silo_four.x,0.5)||
        !nh.param<double>("path_plan/silo_coordinate/silo4/y",silo_four.y,10.75)||
        !nh.param<double>("path_plan/silo_coordinate/silo4/z",silo_four.z,0))
        ROS_ERROR("GET SILO4 COORDINATE FAILED");
    if(!nh.param<double>("path_plan/silo_coordinate/silo5/x",silo_five.x,0.5)||
        !nh.param<double>("path_plan/silo_coordinate/silo5/y",silo_five.y,11.5)||
        !nh.param<double>("path_plan/silo_coordinate/silo5/z",silo_five.z,0))
        ROS_ERROR("GET SILO5 COORDINATE FAILED");

    silo_one.x = mirror_flag*silo_one.x;
    silo_two.x = mirror_flag*silo_two.x;
    silo_three.x = mirror_flag*silo_three.x;
    silo_four.x = mirror_flag*silo_four.x;
    silo_five.x = mirror_flag*silo_five.x;
}


void path_app::path_cmd_cb(const r2_msgs::path_cmd::ConstPtr& msg)
{
    // ROS_INFO("path_cmd_cb");
    static SILO_ENUM last_num;
    static int flag = 0, cnt = 0;
    path_mode = (PATH_ENUM)msg->PathMode;
    takeput_mode = (AUTO_TAKE_PUT)msg->take_or_put;
    path.path_mode = path_mode;
    
    float ball_change_distance = distance_of_two_point(msg->ball_position.x,msg->ball_position.y,
                                                       last_ball_pose.x,last_ball_pose.y);     

    if(path_mode == Hand || path_mode == Stop)
    {
        if(path_status==ACTIVE)
            client.cancelGoal();
    }
    
    if(path_mode == LockupPoint_e)
    {
        float x_error = abs(abs(msg->ball_position.x)-abs(last_ball_pose.x));
        float y_error = abs(abs(msg->ball_position.y)-abs(last_ball_pose.y));
        float yaw_error = abs(abs(msg->ball_position.z)-abs(last_ball_pose.z));
        if(x_error>0.1||y_error>0.1||yaw_error>2)
            flag = 1;             
    }

    if(path_mode == auto_take_put)
    {
        if(msg->take_or_put==Take)
        {
            float x_error = abs(abs(msg->ball_position.x)-abs(last_ball_pose.x));
            float y_error = abs(abs(msg->ball_position.y)-abs(last_ball_pose.y));
            float yaw_error = abs(abs(msg->ball_position.z)-abs(last_ball_pose.z));
            if(((x_error>0.1||y_error>0.1||yaw_error>2)&&path_mode == auto_take_put)||
                (path_mode != last_mode && last_mode!=Hand)                         || 
                 takeput_mode != last_takeput_mode)
            {
                // ROS_INFO("error");
                if(path_status == ACTIVE)  
                {
                    if(ball_change_distance > ball_distance_error)
                    {
                        ROS_INFO("cancel goal");
                        client.cancelGoal();
                    }
                }
                flag=0;
                cnt=0;
            }
        }
        else if(msg->take_or_put == Put)
        {
            if(msg->silo_num != last_num&&path_mode == auto_take_put || (path_mode != last_mode && last_mode!=Hand))
            {
                if(path_status == ACTIVE)
                {
                    if(ball_change_distance > ball_distance_error)
                    {
                        ROS_INFO("cancel goal");
                        client.cancelGoal();
                    }
                }
                flag=0;
                cnt=0;
            }
        }
        else
        {}

        if(msg->take_or_put == Take)
        {
            if(flag == 0 && cnt<2)
            {
                cnt++;
                ROS_INFO("change point");
                ROS_INFO("%f,%f",msg->ball_position.x,msg->ball_position.y);
                start_pose.pose.position.x = robot_pose.x;
                start_pose.pose.position.y = robot_pose.y;
                start_pose.pose.orientation = tf::createQuaternionMsgFromYaw((float)(robot_pose.z*PI/180));
                end_pose.pose.position.x = msg->ball_position.x;
                end_pose.pose.position.y = msg->ball_position.y;
                end_pose.pose.orientation = tf::createQuaternionMsgFromYaw((float)(msg->ball_position.z*PI/180));
                geometry_msgs::PoseStamped middle_pose;

                if(msg->mid_position.x==0&&msg->mid_position.y==0)
                {
                    middle_pose.pose.position.x = (start_pose.pose.position.x+end_pose.pose.position.x)/2;            
                    middle_pose.pose.position.y = (start_pose.pose.position.y+end_pose.pose.position.y)/2;
                    middle_pose.pose.orientation = tf::createQuaternionMsgFromYaw((tf::getYaw(start_pose.pose.orientation)+tf::getYaw(end_pose.pose.orientation))/2);
                }
                else
                {
                    middle_pose.pose.position.x = msg->mid_position.x;
                    middle_pose.pose.position.y = msg->mid_position.y;
                    middle_pose.pose.orientation = tf::createQuaternionMsgFromYaw((float)(msg->mid_position.z*PI/180));
                }
                path.take_ball_planner(start_pose,middle_pose,end_pose);
                flag = 1;
            }
        }
        else if(msg->take_or_put == Put)
        {
            if(flag == 0 && cnt<2)
            {
                cnt++;
                ROS_INFO("change point");
                ROS_INFO("%f,%f",msg->ball_position.x,msg->ball_position.y);
                start_pose.pose.position.x = robot_pose.x;
                start_pose.pose.position.y = robot_pose.y;
                start_pose.pose.orientation = tf::createQuaternionMsgFromYaw((robot_pose.z*PI/180));
                switch (msg->silo_num)
                {
                    case 1:
                        end_pose.pose.position.x = silo_one.x;
                        end_pose.pose.position.y = silo_one.y;
                        end_pose.pose.orientation = tf::createQuaternionMsgFromYaw((silo_one.z*PI/180));
                        break;
                    case 2:
                        end_pose.pose.position.x = silo_two.x;
                        end_pose.pose.position.y = silo_two.y;
                        end_pose.pose.orientation = tf::createQuaternionMsgFromYaw((silo_two.z*PI/180));
                        break;
                    case 3:
                        end_pose.pose.position.x = silo_three.x;
                        end_pose.pose.position.y = silo_three.y;
                        end_pose.pose.orientation = tf::createQuaternionMsgFromYaw((silo_three.z*PI/180));
                        break;
                    case 4:
                        end_pose.pose.position.x = silo_four.x;
                        end_pose.pose.position.y = silo_four.y;
                        end_pose.pose.orientation = tf::createQuaternionMsgFromYaw((silo_four.z*PI/180));
                        break;
                    case 5:
                        end_pose.pose.position.x = silo_five.x;
                        end_pose.pose.position.y = silo_five.y;
                        end_pose.pose.orientation = tf::createQuaternionMsgFromYaw((silo_five.z*PI/180));
                        break;
                    default:
                        break;
                }
                geometry_msgs::PoseStamped middle_pose;
                middle_pose.pose.position.x = (start_pose.pose.position.x+end_pose.pose.position.x)/2;
                middle_pose.pose.position.y = (start_pose.pose.position.y+end_pose.pose.position.y)/2;
                middle_pose.pose.orientation = tf::createQuaternionMsgFromYaw((tf::getYaw(start_pose.pose.orientation)+tf::getYaw(end_pose.pose.orientation))/2);
                path.take_ball_planner(start_pose,middle_pose,end_pose);
                flag = 1;
            }
        }
    }
    path.lockup_point(msg->ball_position);
    path.planner_fsm();
    goal.path = path.path_msg;
    // ROS_INFO("path size: %ld",path.path_msg.poses.size());
    // ROS_INFO("path_mode: %d, last_mode: %d",path_mode,last_mode);
    if((path_mode != last_mode && path_mode!=Hand) || flag==1)
    {
        client.sendGoal(goal,boost::bind(&path_app::done_cb,this,_1,_2),boost::bind(&path_app::active_cb,this),boost::bind(&path_app::feedback_cb,this,_1));
        ROS_INFO("send goal");
        flag=2;
    }

    //更新当前的路径状态
    last_mode = path_mode; last_num = (SILO_ENUM)msg->silo_num;
    last_takeput_mode = takeput_mode;
    last_ball_pose = msg->ball_position;
    if(path.path_msg.poses.size() > 0)
    {
        path_status_msg.now_target_point.x = path.path_msg.poses[path.path_msg.poses.size()-1].pose.position.x;
        path_status_msg.now_target_point.y = path.path_msg.poses[path.path_msg.poses.size()-1].pose.position.y;
        path_status_msg.now_target_point.z = tf::getYaw(path.path_msg.poses[path.path_msg.poses.size()-1].pose.orientation)*180/PI;
    }
    path_status_msg.now_path_mode = path.path_mode;
    path_status_msg.now_takeput_mode = msg->take_or_put;
    path_status_msg.destination_arrived = path_status;
    path_status_pub.publish(path_status_msg);
}


void path_app::active_cb()
{
    path_status = ACTIVE;
    ROS_INFO("Goal just went active");
}


void path_app::feedback_cb(const r2_msgs::pid_followerFeedbackConstPtr &feedback)
{
    // ROS_INFO("Got Feedback of length %lf",feedback->progress_bar);
}


void path_app::done_cb(const actionlib::SimpleClientGoalState& state, const r2_msgs::pid_followerResultConstPtr &result)
{
    path_status = SUCCCEED;
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Answer: %d", result->result);
}


void path_app::robot_pose_callback(const geometry_msgs::Point::ConstPtr &msg)
{
    robot_pose = *msg;
}


} // namespace path_app_ns


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "path_app");
    path_app_ns::path_app path_app;
    ros::spin();

    return 0;
}
