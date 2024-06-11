#include "ros/ros.h"
#include "r2_path/path_run.h"


path_run::path_run()
{
    robot_coord_sub = nh.subscribe<geometry_msgs::Point>("/robot_coordinate",1,&path_run::robot_coord_callback,this);
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    ball_pos_sub = nh.subscribe<r2_msgs::path_cmd>("/path_cmd",1,&path_run::path_cmd_callback,this);
	target_vel_pub = nh.advertise<geometry_msgs::Point>("/target_vel",1);
	path_status_pub = nh.advertise<r2_msgs::path_status>("/path_status",1);
    mode = last_mode = Hand;
    path_plan_start=recieve_ball_flag=path_res=0;

	//init the silo position
	pos_silo_one.x = 500;   pos_silo_one.y = 8500;    pos_silo_one.z = 0;
	pos_silo_two.x = 500;   pos_silo_two.y = 9250;    pos_silo_two.z = 0;
	pos_silo_three.x = 500; pos_silo_three.y = 10000; pos_silo_three.z = 0;
	pos_silo_four.x = 500;  pos_silo_four.y = 10750;  pos_silo_four.z = 0;
	pos_silo_five.x = 500;  pos_silo_five.y = 11500;  pos_silo_five.z = 0;

	if(!nh.param<float>("/path_plan/take_ball_move_speed",take_ball_move_speed,0)||
	   !nh.param<float>("/path_plan/control_cycle",control_cycle,0))
		ROS_ERROR("TAKE BALL INFORMATON INIT ERROR!!!");
    if(!nh.param<float>("/path_plan/lockup/target_pointX",Target_Lockup_X,0)||
	   !nh.param<float>("/path_plan/lockup/target_pointY",Target_Lockup_Y,0)||
	   !nh.param<float>("/path_plan/lockup/target_YAW",Target_Lockup_YAW,0))
		ROS_ERROR("LOCKUP TARGET POINT INIT ERROR!!!");

	if(!nh.param<float>("/path_plan/path1_pointX_pid/p",path.path_pid_x_.pid_data.K_P,0)||
	   !nh.param<float>("/path_plan/path1_pointX_pid/i",path.path_pid_x_.pid_data.K_I,0)||
	   !nh.param<float>("/path_plan/path1_pointX_pid/d",path.path_pid_x_.pid_data.K_D,0)||
	   !nh.param<float>("/path_plan/path1_pointX_pid/out_max",path.path_pid_x_.pid_data.Out_MAX,0)||
	   !nh.param<float>("/path_plan/path1_pointX_pid/dead_zone",path.path_pid_x_.pid_data.Dead_Size,-1)||
	   !nh.param<float>("/path_plan/path1_pointX_pid/pre_feed",path.pre_feedx,0))
		ROS_ERROR("POINT_X PID INIT ERROR!!!");
			
	if(!nh.param<float>("/path_plan/path1_pointY_pid/p",path.path_pid_y_.pid_data.K_P,0)||
	   !nh.param<float>("/path_plan/path1_pointY_pid/i",path.path_pid_y_.pid_data.K_I,0)||
		!nh.param<float>("/path_plan/path1_pointY_pid/d",path.path_pid_y_.pid_data.K_D,0)||
	   !nh.param<float>("/path_plan/path1_pointY_pid/out_max",path.path_pid_y_.pid_data.Out_MAX,0)||
	   !nh.param<float>("/path_plan/path1_pointY_pid/dead_zone",path.path_pid_y_.pid_data.Dead_Size,-1)||
	   !nh.param<float>("/path_plan/path1_pointY_pid/pre_feed",path.pre_feedy,0))
		ROS_ERROR("POINT_Y PID INIT ERROR!!!");

	if(!nh.param<float>("/path_plan/path1_Yaw_pid/p",path.path_pid_yaw_.pid_data.K_P,0)||
	   !nh.param<float>("/path_plan/path1_Yaw_pid/i",path.path_pid_yaw_.pid_data.K_I,0)||
	   !nh.param<float>("/path_plan/path1_Yaw_pid/d",path.path_pid_yaw_.pid_data.K_D,0)||
	   !nh.param<float>("/path_plan/path1_Yaw_pid/out_max",path.path_pid_yaw_.pid_data.Out_MAX,0)||
	   !nh.param<float>("/path_plan/path1_Yaw_pid/dead_zone",path.path_pid_yaw_.pid_data.Dead_Size,-1))
		ROS_ERROR("YAW PID INIT ERROR!!!");

        path.path_pid_x_.pid_data.Out_MIN = -path.path_pid_x_.pid_data.Out_MAX;
		path.path_pid_y_.pid_data.Out_MIN = -path.path_pid_y_.pid_data.Out_MAX;
		path.path_pid_yaw_.pid_data.Out_MIN = -path.path_pid_yaw_.pid_data.Out_MAX;

		std::vector<float> X_TEST1,Y_TEST1,Yaw_TEST1;
		std::vector<float> X_TEST2,Y_TEST2,Yaw_TEST2;
	if(!nh.param<float>("/path_plan/path1/path_control_point_num",path1_control_num,0)||
	   !nh.param<float>("/path_plan/path1/path_use_time",path1_t,0)||
	   !nh.param<std::vector<float>>("/path_plan/path1/path_pointX",X_TEST1,{0})||
	   !nh.param<std::vector<float>>("/path_plan/path1/path_pointY",Y_TEST1,{0})||
	   !nh.param<std::vector<float>>("/path_plan/path1/path_YAW",Yaw_TEST1,{0})||
	   !nh.param<float>("/path_plan/path2/path_control_point_num",path2_control_num,0)||
	   !nh.param<float>("/path_plan/path2/path_use_time",path2_t,0)||
	   !nh.param<std::vector<float>>("/path_plan/path2/path_pointX",X_TEST2,{0})||
	   !nh.param<std::vector<float>>("/path_plan/path2/path_pointY",Y_TEST2,{0})||
	   !nh.param<std::vector<float>>("/path_plan/path2/path_YAW",Yaw_TEST2,{0}))
	   	ROS_ERROR("GET PATH1 POINT ERROR!!!");
	else
	{
		for(float num : X_TEST1)   {static int i=0; X0[i] = num;   i++;}
		for(float num : Y_TEST1)   {static int i=0; Y0[i] = num;   i++;}
		for(float num : Yaw_TEST1) {static int i=0; Yaw0[i] = num; i++;}
		for(float num : X_TEST2)   {static int i=0; X1[i] = num;   i++;}
		for(float num : Y_TEST2)   {static int i=0; Y1[i] = num;   i++;}
		for(float num : Yaw_TEST2) {static int i=0; Yaw1[i] = num; i++;}
	}
}

void path_run::run()
{
    //set the absolutely delay time
  	double absolute_delay = control_cycle; // 50ms
	ros::Rate rate(1000);
	ros::Time start = ros::Time::now();		//get the start time

    while (ros::ok())
    {
		// get the current time
		ros::Time current_time = ros::Time::now();
		// calculate the elapsed time		
		ros::Duration elapsed_time = current_time - start;

		path.robot_pos = robot_pos;	// input the robot position now

		// the control cycle is 50ms
		if(elapsed_time.toSec() >= absolute_delay)
		{
            move_time_counter+=control_cycle;	//calculate the elapsed time of the path plan
            fsm_path();	
            vel_pub_.publish(path.RobotMove_To_WorldMove(path.robot_target_vel));	//publish the velocity command
			target_vel_pub.publish(path.now_path_point);	//publish the target velocity
			start = ros::Time::now(); 	//update the start time
			// ROS_INFO("path_res:%d",path_res);	
        }

		path_status_.destination_arrived = take_put_arrive_flag;
		path_status_.now_path_mode = mode;
		path_status_.now_target_point = target_ball_pos;
		path_status_pub.publish(path_status_);

        rate.sleep();
        ros::spinOnce();
    }
}


void path_run::fsm_path()
{
    switch(mode)
    {
        case LockupPoint_e:
        {
            path.LockupPoint(target_ball_pos.x,target_ball_pos.y,target_ball_pos.z,1.5,1.5);
            break;
        }

        case PATH1:
        {
            if(path_res!=1)
                path_res = path.ThreeB_PathPlan(move_time_counter,path1_t,path1_control_num,X0,Y0,Yaw0);

            if(path_res==1)
                path.LockupPoint(1600,6500,0,1.5,1.5);
            break;
        }

        case PATH2:
        {
            if(path_res!=1)
                path_res = path.ThreeB_PathPlan(move_time_counter,path2_t,path2_control_num,X1,Y1,Yaw1);

            if(path_res==1)
				path.LockupPoint(4000,9500,0,1.5,1.5);
            break;
        }

        case auto_take_put:
		{
						
			if(recieve_ball_flag==1)
			{
				if(final_distance>1000)	
				{
					if(path_res!=1)
						path_res = path.ThreeB_PathPlan(move_time_counter,take_ball_time,3,ball_pos_X,ball_pos_Y,ball_pos_Yaw);

					if(path_res==1)
						path.LockupPoint(target_ball_pos.x,target_ball_pos.y,target_ball_pos.z,1.5,1.5);

				}
				else
				{
				    switch (take_put_mode)
					{
						case Take:
							path.robot_target_vel.linear.x = 1.5*(target_ball_pos.x-robot_pos.x)/1000;
							path.robot_target_vel.linear.y = 1.5*(target_ball_pos.y-robot_pos.y)/1000;
							path.Yaw_Adjust(target_ball_pos.z);
							break;
						
						case ToSilo_one:
							path.LockupPoint(target_ball_pos.x,target_ball_pos.y,0,1.5,1.5);
							break;
					}
				}
				// path.LockupPoint(target_ball_pos.x,target_ball_pos.y,target_ball_pos.z,1.5,1.5);
            }	

			// if the distance is smaller than 50mm, the robot is arrived, 
			// we should add the flag that we have taken the ball or put the ball in the future, it's very important
			if(ball_distance > 20)	
			    take_put_arrive_flag = 0;
			else
				take_put_arrive_flag = 1;

			break;
        }

        default:
            break;

    }
}


void path_run::path_cmd_callback(const r2_msgs::path_cmd::ConstPtr& path_cmd)
{
	recieve_ball_flag = 1;
	path_stauts(*path_cmd);
	
	ball_distance = distance_of_two_point(target_ball_pos.x,target_ball_pos.y,robot_pos.x,robot_pos.y);

	if(path_plan_start==1)
	{
		dynamic_curve();

		final_distance = distance_of_two_point(target_ball_pos.x,target_ball_pos.y,robot_start_pos.x,robot_start_pos.y);
		ROS_INFO("final_distance:%f",final_distance);
		take_ball_time = final_distance/(take_ball_move_speed*1000);
		last_target_ball_pos = target_ball_pos;
		path_plan_start = 0;	//reset the start flag
	}

	last_ball_pos = target_ball_pos;
	last_mode = mode;
}


void path_run::robot_coord_callback(const geometry_msgs::Point::ConstPtr& robot_coord)
{
    robot_pos = *robot_coord;
}


void path_run::dynamic_curve()
{
	if(take_put_arrive_flag)
	{
		move_time_counter=control_cycle; path_res=0;
		robot_start_pos = robot_pos;

		for(int i=0;i<3;i++)
		{
			ball_pos_X[i] = robot_start_pos.x;
			ball_pos_Y[i] = robot_start_pos.y;
			ball_pos_Yaw[i] = robot_start_pos.z;
		}

		ball_pos_X[3] = (target_ball_pos.x+robot_start_pos.x)/2;
		ball_pos_Y[3] = (target_ball_pos.y+robot_start_pos.y)/2;
		ball_pos_Yaw[3] = target_ball_pos.z;

		for(int i=4;i<7;i++)
		{
			ball_pos_X[i] = target_ball_pos.x;
			ball_pos_Y[i] = target_ball_pos.y;
			ball_pos_Yaw[i] = target_ball_pos.z;
		}
	}
	else
	{
		//增加一个可
		ROS_INFO("dynamic_curve");
		path_cos = abs(target_ball_pos.x-robot_start_pos.x)/ball_distance;
		path_sin = abs(target_ball_pos.y-robot_start_pos.y)/ball_distance;

		
		if(target_ball_pos.x-robot_start_pos.x>0)
			ball_pos_X[3] = (robot_pos.x + 0)*path_cos;
		else
			ball_pos_X[3] = (robot_pos.x - 0)*path_cos;

		if(target_ball_pos.y-robot_start_pos.y>0)
			ball_pos_Y[3] = (robot_pos.y + 0)*path_sin;
		else
			ball_pos_Y[3] = (robot_pos.y - 0)*path_sin;
		
		ball_pos_Yaw[3] = target_ball_pos.z;	

		for(int i=4;i<7;i++)
		{
			ball_pos_X[i] = target_ball_pos.x;
			ball_pos_Y[i] = target_ball_pos.y;
			ball_pos_Yaw[i] = target_ball_pos.z;	
		}

		//calculate the time of the robot after the target point changed
		take_ball_time = move_time_counter + (ball_distance*0.8)/(take_ball_move_speed*1000);
	}
}
 

void path_run::path_stauts(r2_msgs::path_cmd cmd)
{
	if(cmd.take_or_put == Take)
	{
		take_put_mode = Take;
	}
	else
	{
		take_put_mode = ToSilo_one;
	}

	//elec the path mode
	switch (cmd.PathMode)
	{
		case PATH1: {mode = PATH1; break;}			
		case PATH2: {mode = PATH2; break;}
		case auto_take_put: {mode = auto_take_put; break;}			
		case LockupPoint_e: {mode = LockupPoint_e; break;}
		case Hand: {mode = Hand; break;}
		default: break;
	}

	if(mode == auto_take_put)
	{
		//选择取球还是放球
		switch(cmd.take_or_put)
		{
			case Take:
				target_ball_pos = cmd.ball_position;
				break;

			case ToSilo_one:
				target_ball_pos = pos_silo_one;
				break;
			
			case ToSilo_two:
				target_ball_pos = pos_silo_two;
				break;

			case ToSilo_three:
				target_ball_pos = pos_silo_three;
				break;

			case ToSilo_four:
				target_ball_pos = pos_silo_four;
				break;

			case ToSilo_five:
				target_ball_pos = pos_silo_five;
				break;
			
			default:
				break;
		}
	}

	if(last_mode != mode)
	{
		move_time_counter = control_cycle; 
		path_res=0;
	}

	//满足目标点变化大于阈值才能重新规划路径
	if(target_ball_pos.x != last_ball_pos.x || target_ball_pos.y != last_ball_pos.y || 
		target_ball_pos.z != last_ball_pos.z)
	{
		if(distance_of_two_point(target_ball_pos.x,target_ball_pos.y,last_target_ball_pos.x,last_target_ball_pos.y)>1000)
		{
			path_plan_start = 1;
			ROS_INFO("path_plan_start:%d",path_plan_start);
		}
	}
}