#pragma once
#include "ros/ros.h"
#include "r2_msgs/path_cmd.h"
#include "geometry_msgs/Point.h"
#include "r2_msgs/path_status.h"
#include "r2_msgs/stm32.h"
#include "r2_msgs/controller_cmd.h"
#include "r2_msgs/decision.h"

enum PATH_ENUM
{
    Hand=0,
	LockupPoint_e=1,
	PATH1=2,
	PATH2=3,
	TAKE_BALLS,
    STOP
};

enum AUTO_TAKE_PUT
{
	Take=1,
	Put=2,
};

enum SILO_ENUM
{
	ToSilo_one=1,
	ToSilo_two=2,
	ToSilo_three=3,
	ToSilo_four=4,
	ToSilo_five=5,
};

enum PATH_STATUS
{
	SUCCCEED = 12,
	ACTIVE = 13,
	CANCEL = 14,
};

typedef enum
{
    BALL_OUTSIDE=0,     //未拿到球(放球成功以及第一次进入三区时的状态) 0
    BALL_HAVE_TAKEN,    //已拿到球(机器人拿到球，但夹爪还没碰到微动开关) 1
    BALL_IN_CLAW,        //拿到球且夹爪到位(球在夹爪中，已经触碰到微动开关) 2
}BALL_STATUS;

typedef enum
{
    TAKE_BALL = 2,   //取球 2
    SHOOT_BALL,     //出球 3
    CONTROLLER_OFF, //关闭 4
    CONTROLLER_ERROR    //错误状态 5
}CONTROLLER_STATE;

typedef enum
{
    NO_BALL = 0,    //无球
    BLUE_BALL,      //蓝球
    PURPLE_BALL,    //紫球
    RED_BALL        //红球
}BALL_COLOR;

typedef enum
{
    NORMAL_TAKE_BALL = 0,   //正常吸球
    LEFT_TAKE_BALL,         //左吸球
    RIGHT_TAKE_BALL         //右吸球
}TAKE_BALL_STATE;

namespace take_ball_ns
{
class take_ball
{
public:
    take_ball();
    void robot_decision(r2_msgs::path_cmd &path_cmd,r2_msgs::controller_cmd &controller_cmd);
private:
    ros::NodeHandle nh;
    ros::Subscriber cameraData_sub;
    ros::Subscriber robotPos_sub;
    ros::Subscriber path_status_sub;
	ros::Subscriber controller_status_sub;
    ros::Subscriber decision_sub;
    ros::Subscriber zone_decision_sub;

    geometry_msgs::Point ball_position;
    geometry_msgs::Point robot_position;
    r2_msgs::stm32 controller_status;
    r2_msgs::path_status now_path_status;
    r2_msgs::decision decision;

    bool if_use_decision = 0;
    bool if_use_debug = true;
    float y_max,y_min,x_abs_max;
    std::string decision_topic;
    std::string zone_decision_topic;
    std::string field_direction;
    int target_silo; int mirror_flag;
    int camera_data_first = 1;

    void cameraDataCallback(const geometry_msgs::Point::ConstPtr& msg);
    void robotPosCallback(const geometry_msgs::Point::ConstPtr& msg);
    void pathStatusCallback(const r2_msgs::path_status::ConstPtr& msg);
	void controllerStatusCallback(const r2_msgs::stm32::ConstPtr& msg);
    void decisionCallback(const r2_msgs::decision::ConstPtr& msg);
    // void zoneDecisionCallback(const r2_msgs::decision::ConstPtr& msg);

    void take_ball_planner(r2_msgs::path_cmd &path_cmd,r2_msgs::controller_cmd &controller_cmd);
    void put_ball_planner(r2_msgs::path_cmd &path_cmd,r2_msgs::controller_cmd &controller_cmd);
};


} // namespace name
 