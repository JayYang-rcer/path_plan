#include "r2_path/trackers/pid_follower.h"


namespace tracker
{
    void pid_follower::control_timer_callback(const ros::TimerEvent&)
    {
        
    }

    void pid_follower::executeCallback(const r2_msgs::pid_followerGoalConstPtr &goal) 
    {
        
        path_flag = 0;
        path_done = 0;
        ros::Rate rate(1000);
        ros::Time now = ros::Time::now();
        ros::Time last;
        path = goal->path;
        ROS_INFO("Path received!");
        while(ros::ok())
        {
            if(as_.isPreemptRequested())
            {
                ROS_INFO("Preempted");
                target_vel.linear.x = 0;
                target_vel.linear.y = 0;
                target_vel.angular.z = 0;
                as_.setPreempted();
                break;
            }

            now = ros::Time::now();
            if(now.toSec() - last.toSec() >= 1/control_frequence)
            {
                control();
                target_vel = RobotMove_To_WorldMove(target_vel,robot_pose.z);
                vel_pub.publish(target_vel);
                last = now;
            }
            if(path_done == 1)
            {
                break;
            }
            rate.sleep();
        }
    }
    

    void pid_follower::control(void)
    {
        static r2_msgs::pid_followerFeedback feedback;
        static int path_index = 0;
        static int path_size;
        static int cnt = 0;
        
        if(path_flag==0)
        {
            ROS_INFO("init");
            path_size = path.poses.size();
            path_flag = 1;
            path_index=0;
        }

        if(path_flag==1)
        {
            if(path_size==0)
            {
                ROS_INFO("Path size is 0!");
                path_flag = 0;
                return;
            }

            if(path_index<path_size)
            {
                geometry_msgs::Point target;
                static geometry_msgs::Point last_target;
                float yaw = tf::getYaw(path.poses[path_index].pose.orientation)*180/PI;
                target.x = path.poses[path_index].pose.position.x;
                target.y = path.poses[path_index].pose.position.y;
                forecast_vel.linear.x = (target.x - last_target.x)*pid_x_PreFeed;
                forecast_vel.linear.y = (target.y - last_target.y)*pid_y_PreFeed;
                target.z = yaw;

                PD_control(target,robot_pose);
                last_target = target;
                path_index++;
                // ROS_INFO("path_index: %d", path_index);
            }
            else
            {
                geometry_msgs::Point target;
                target.x = path.poses[path_size-1].pose.position.x;
                target.y = path.poses[path_size-1].pose.position.y;
                target.z = tf::getYaw(path.poses[path_size-1].pose.orientation)*180/PI;
                forecast_vel.linear.x = 0;
                forecast_vel.linear.y = 0;
                PD_control(target,robot_pose);
                if(target_vel.linear.x > LockupPoint_xMax)
                    target_vel.linear.x = LockupPoint_xMax;
                if(target_vel.linear.x < -LockupPoint_xMax)
                    target_vel.linear.x = -LockupPoint_xMax;
                if(target_vel.linear.y > LockupPoint_yMax)
                    target_vel.linear.y = LockupPoint_yMax;
                if(target_vel.linear.y < -LockupPoint_yMax)
                    target_vel.linear.y = -LockupPoint_yMax;
                if(target_vel.angular.z > LockupPoint_YawMax)
                    target_vel.angular.z = LockupPoint_YawMax;
                
                //加上一个限制，防止只有一瞬间经过终点
                if(sqrt(pow(target.x-robot_pose.x,2)+pow(target.y-robot_pose.y,2))<xy_deadzone&&abs(target.z-robot_pose.z)<yaw_deadzone)
                {
                    cnt++;
                    if(path_done == 0&&cnt>8&&cnt<=10)
                    {
                        target_vel.linear.x = 0;
                        target_vel.linear.y = 0;
                        target_vel.angular.z = 0;
                    }
                    if(path_done == 0&&cnt>10)
                    {
                        ROS_INFO("Path done!");
                        as_.setSucceeded();
                        path_done = 1;
                        cnt=0;
                    }
                }
                        
            }

            feedback.progress_bar = (float)path_index/(float)path_size;
            // feedback.current_pose = robot_pose;
            as_.publishFeedback(feedback);
        }
    }


    void pid_follower::PD_control(geometry_msgs::Point target, geometry_msgs::Point current)
    {
        Yaw_Adjust(target.z, current.z);
        pid_x.current = current.x;
        pid_x.target = target.x;
        // ROS_INFO("pid_x.current: %f, pid_x.target: %f", pid_x.current, pid_x.target);

        pid_y.current = current.y;
        pid_y.target = target.y;
        target_vel.linear.x = pid_x.Adjust() + forecast_vel.linear.x;
        target_vel.linear.y = pid_y.Adjust() + forecast_vel.linear.y;
        // ROS_INFO("target_vel.linear.x: %f, target_vel.linear.y: %f", target_vel.linear.x, target_vel.linear.y);
    }

    /**
	* @brief  YawAdjust偏航角控制
	* @note		将偏航角控制在目标角度
	* @param  Target_angle:要限制的值
	* @retval 
	*/
	void pid_follower::Yaw_Adjust(float target_angle, float now_angle)
	{
		//计算误差
		float yaw_error;
		if(now_angle * target_angle > 0)
		{
			yaw_error = target_angle - now_angle;
		}
		else
		{
			if(abs(now_angle) + abs(target_angle) <= 180)
				yaw_error = target_angle - now_angle;
			else
			{
				Angle_Limit(&yaw_error);
				target_angle = yaw_error + now_angle;
			}
		}

		//PID输出角速度
		pid_yaw.current = now_angle;
		pid_yaw.target = target_angle;
		target_vel.angular.z = pid_yaw.Adjust();
	}

    geometry_msgs::Twist pid_follower::RobotMove_To_WorldMove(geometry_msgs::Twist target_robot_vel, float robot_yaw)
	{
		double COS = cos (robot_yaw*PI/180);
		double SIN = sin (robot_yaw*PI/180);

		geometry_msgs::Twist world_vel;
		world_vel.linear.x  = (target_robot_vel.linear.x * COS + target_robot_vel.linear.y * SIN);
		world_vel.linear.y  = -(target_robot_vel.linear.x * SIN - target_robot_vel.linear.y * COS);
		world_vel.angular.z = target_robot_vel.angular.z;
	
		return world_vel;
	}
} // namespace tracker


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"pid_follower");
    tracker::pid_follower follower("pid_follower");
    ros::spin();
    
    return 0;
}
