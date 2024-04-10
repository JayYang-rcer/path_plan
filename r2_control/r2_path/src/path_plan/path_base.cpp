#include "r2_path/path_base.h"

namespace path_base_ns
{
	path_base::path_base()
	{
		first_time_flag=1;
	}

	PATH_ENUM path_base::stringToPATH_ENUM(const std::string& path_str){
			static const std::map<std::string,PATH_ENUM> path_map = {
			{"LockupPoint", LockupPoint_e},
			{"PATH1", PATH1},
			{"PATH2",PATH2}
    	};

		auto it = path_map.find(path_str);

		return (it!=path_map.end()?it->second : LockupPoint_e); //默认为LockupPoint
	}

	void path_base::Angle_Limit(float *angle)
	{
		static uint8_t recursiveTimes = 0;
		recursiveTimes++;

		if(recursiveTimes<100)
		{
			if(*angle > 180.0f)
			{
				*angle -= 360.0f;
				Angle_Limit(angle);
			}
			else if(*angle < -180.0f)
			{
				*angle += 180.0f;
				Angle_Limit(angle);
			}
			else{}
		}

		recursiveTimes--;
	}

	int path_base::TrapezoidPlaning_set(float S, float POS_X, float POS_Y, float POS_YAW, float V_max, float R_ac, float R_de)
	{
		Yaw_Adjust(POS_YAW);
		float error;
		float target_POS,target_VX,target_VY;
		float Ssu;   //总路程
		float Sac;   //加速路程
		float Sde;   //减速路程
		float Sco;   //匀速路程
		float Aac;   //加速加速度
		float Ade;   //减速加速度
		
		
		//avoid unexpected case
		if((R_ac<0||R_ac>1)||(R_de>1||R_de<0))
		{
			target_VX=0;
			target_VY=0;
		}
		
		//calculate error
		error = distance_of_two_point(robot_pos.x,robot_pos.y,POS_X,POS_Y);

		//calculate Sac Sde Sco
		Sac = R_ac*S;
		Sde = R_de*S;
		Sco = S - Sac - Sde;

		Aac = V_max*V_max/(2*Sac);
		Ade = V_max*V_max/(2*Sde);

		if(Sac < S-error)
		{
			target_POS = sqrt(2*Aac*(Sac-error));
			target_VX = -(robot_pos.x-POS_X)/error*target_POS;
			target_VY = -(robot_pos.y-POS_Y)/error*target_POS;
		}
		else if(Sac+Sco < S-error)
		{
			target_VX = -V_max*(robot_pos.x-POS_X)/error;
			target_VY = -V_max*(robot_pos.y-POS_Y)/error;
		}
		else
		{
			target_POS = sqrt(2*Ade*(S-error));
			target_VX = -(robot_pos.x-POS_X)/error*target_POS;
			target_VY = -(robot_pos.y-POS_Y)/error*target_POS;
		}

		if(error<100)
		{
			target_VX = 0;
			target_VY = 0;

			robot_target_vel.linear.x = target_VX;	
			robot_target_vel.linear.y = target_VY;
			return 1;
		}
			
		robot_target_vel.linear.x = target_VX;
		robot_target_vel.linear.y = target_VY;
		return 0;
	} 

	/**
	* @brief  YawAdjust偏航角控制
	* @note		将偏航角控制在目标角度
	* @param  Target_angle:要限制的值
	* @retval 
	*/
	void path_base::Yaw_Adjust(float target_angle)
	{
		//计算误差
		float yaw_error;
		if(robot_pos.z * target_angle > 0)
		{
			yaw_error = target_angle - robot_pos.z;
		}
		else
		{
			if(abs(robot_pos.z) + abs(target_angle) <= 180)
				yaw_error = target_angle - robot_pos.z;
			else
				Angle_Limit(&yaw_error);
		}

		//PID输出角速度
		robot_target_vel.angular.z = path_pid_yaw_.PID_Position_Calculate_by_error(yaw_error);
	}

	/**
	* @brief  LockupPoint锁定车
	* @note		将车锁定在某一点上
	* @param  POS_X:要限制的X值，POS_Y:要限制的Y值，POS_YAW:要限制的偏航角
	* @retval 
	*/
	void path_base::LockupPoint(float POS_X, float POS_Y, float POS_YAW, float Vxy_max, float Vw_Max)
	{
		Yaw_Adjust(POS_YAW);
		robot_target_vel.linear.x = path_pid_x_.PID_Position_Calculate_by_error(POS_X-robot_pos.x);
		robot_target_vel.linear.y = path_pid_y_.PID_Position_Calculate_by_error(POS_Y-robot_pos.y);
		abs_limit(robot_target_vel.linear.x,Vxy_max);
		abs_limit(robot_target_vel.linear.y,Vxy_max);
		abs_limit(robot_target_vel.angular.z,Vw_Max);
		// ROS_INFO("%f %f", cmd_vel.linear.x, cmd_vel.linear.y);
	}


	/**
	* @brief  PDController跟踪器
	* @note		跟踪规划好的路径
	* @param  target_point:单位时间要跟踪的点（需先规划好速度），robot_now_pos:机器人当前世界坐标下的位置
	* @retval 
	*/
	void path_base::PD_Controller(geometry_msgs::Point target_point, geometry_msgs::Point robot_position)
	{
		Yaw_Adjust(target_point.z);

		//计算误差
		error_X = target_point.x - robot_position.x;
		error_Y = target_point.y - robot_position.y;

		robot_target_vel.linear.x = path_pid_x_.PID_Position_Calculate_by_error(error_X) + pre_feedx *now_path_vel.linear.x;
		robot_target_vel.linear.y = path_pid_y_.PID_Position_Calculate_by_error(error_Y) + pre_feedy *now_path_vel.linear.y;
	}

	/**
	* @brief  PathPlan规划+跟踪
	* @note		贝塞尔曲线规划，误差直接赋值，到达终点返回1，否则返回0
	* @param  t_real:真实经过的时间，t_target:目标总时间，num:控制点数目+1，X、Y:控制点数组
	* @retval 
	*/
	int path_base::Bezier_PathPlan(float t_real, float t_target, int num, float *X, float *Y, float *Yaw)
	{
		float t;
		float x=0,y=0;
		t = t_real/t_target;   

		for (int i = 0; i < num; ++i)
		{
			float b = (float)(factorial(num - 1)/(factorial(i)*factorial(num-1-i)))*pow(1-t,num-1-i)*pow(t,i);
			x += b*X[i];
			y += b*Y[i];
		}
		
		if(first_time_flag)
		{
			now_path_vel.linear.x = 0;
			now_path_vel.linear.y = 0;
			now_path_vel.angular.z = 0;
			first_time_flag = 0;
			threeB_Hz = 1 / t_real;
		}
		else
		{
			now_path_vel.linear.x = (now_path_point.x - last_X)/1000 * threeB_Hz;
			now_path_vel.linear.y = (now_path_point.y - last_Y)/1000 * threeB_Hz;
			now_path_vel.angular.z = (now_path_point.z - last_Yaw)/1000 * threeB_Hz; 
		}

		PD_Controller(now_path_point,robot_pos);

		//保留本次值
		last_X = now_path_point.x;
		last_Y = now_path_point.y;
		last_Yaw = now_path_point.z;

		//到达终点
		if(t_real > t_target)
		{
			robot_target_vel.linear.x = 0;
			robot_target_vel.linear.y = 0;
			first_time_flag = 1;
			return 1;
		} 

		return 0;
	}

	/**
	* @brief  PathPlan规划+跟踪
	* @note		三次B样条规划，误差直接赋值，到达终点返回1，否则返回0
	* @param  t_real:真实经过的时间，t_target:目标总时间，num:控制点数目+1，X、Y:控制点数组
	* @retval 
	*/
	int path_base::ThreeB_PathPlan(float t_real, float t_target, int num, float *X , float *Y, float *Yaw)
	{
		k=(int)(t_real*num / t_target);     //第k段
		threeb_t=t_real - k*t_target / num;        //第k段时间
		threeb_t=threeb_t*num / t_target;

		//位置样条函数
		threeb_f1s = (1 - threeb_t) * (1 - threeb_t) * (1 - threeb_t) / 6;
		threeb_f2s = (3 * threeb_t * threeb_t * threeb_t - 6 * threeb_t * threeb_t + 4) / 6;
		threeb_f3s = (-3 * threeb_t * threeb_t * threeb_t + 3 * threeb_t * threeb_t + 3 * threeb_t + 1) / 6;
		threeb_f4s = (threeb_t * threeb_t * threeb_t) / 6;

		// 计算目标跟踪点
		now_path_point.x = X[k]*threeb_f1s + X[k+1]*threeb_f2s + X[k+2]*threeb_f3s + X[k+3]*threeb_f4s;
		now_path_point.y = Y[k]*threeb_f1s + Y[k+1]*threeb_f2s + Y[k+2]*threeb_f3s + Y[k+3]*threeb_f4s;
		now_path_point.z = Yaw[k]*threeb_f1s + Yaw[k+1]*threeb_f2s + Yaw[k+2]*threeb_f3s + Yaw[k+3]*threeb_f4s;

		if(first_time_flag)
		{
			now_path_vel.linear.x = 0;
			now_path_vel.linear.y = 0;
			now_path_vel.angular.z = 0;
			threeB_Hz = 1 / t_real;
			first_time_flag = 0;
		}
		else
		{
			now_path_vel.linear.x = (now_path_point.x - last_X) * threeB_Hz/1000;
			now_path_vel.linear.y= (now_path_point.y - last_Y) * threeB_Hz/1000;
			now_path_vel.angular.z = (now_path_point.z - last_Yaw) * threeB_Hz/1000;
		}

		//PD跟踪器
		PD_Controller(now_path_point,robot_pos);
		
		last_X = now_path_point.x;
		last_Y = now_path_point.y;
		last_Yaw = now_path_point.z;

		//判断是否到达终点
		if(t_real > t_target)
		{
			robot_target_vel.linear.x = 0;
			robot_target_vel.linear.y = 0;
			first_time_flag = 1;
			return 1;
		}
		return 0;
	}


	geometry_msgs::Twist path_base::RobotMove_To_WorldMove(geometry_msgs::Twist target_robot_vel)
	{
		COS = cos (robot_pos.z*PI/180);
		SIN = sin (robot_pos.z*PI/180);

		geometry_msgs::Twist world_vel;
		world_vel.linear.x  = (target_robot_vel.linear.x * COS + target_robot_vel.linear.y * SIN);
		world_vel.linear.y  = -(target_robot_vel.linear.x * SIN - target_robot_vel.linear.y * COS);
		world_vel.angular.z = target_robot_vel.angular.z;
	
		return world_vel;
	}
    
}
