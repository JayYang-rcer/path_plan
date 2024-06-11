/**
 * @brief 里程计服务，处理里程计消息，把角度转化为-180 ～ 180
*/

#include "ros/ros.h"
#include <cmath>
#include "r2_msgs/action_cmd.h"


struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

enum FIELD_ENUM
{
    RIGHT = 1,
    LEFT = 2,
};

bool if_use_gazebo;
FIELD_ENUM field_direction;

FIELD_ENUM stringToPATH_ENUM(const std::string& path_str)
{
	static const std::map<std::string,FIELD_ENUM> path_map = {
		{"right", RIGHT},
		{"left", LEFT},
    };

	auto it = path_map.find(path_str);
	return (it!=path_map.end()?it->second : RIGHT); //默认为LockupPoint
}

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;
 
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);
 
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);
 
    return angles;
}

bool doReq(r2_msgs::action_cmd::Request& req,r2_msgs::action_cmd::Response& resp)
{
    EulerAngles Angles;
    Quaternion q;
    q.w = req.odomData.pose.pose.orientation.w;
    q.x = req.odomData.pose.pose.orientation.x;
    q.y = req.odomData.pose.pose.orientation.y;
    q.z = req.odomData.pose.pose.orientation.z;

    Angles = ToEulerAngles(q);
    
    resp.Yaw.data = Angles.yaw/3.14159 * 180;
    if(if_use_gazebo==false && field_direction==RIGHT)
    {
        resp.POS_X.data = req.odomData.pose.pose.position.x + 5.5;
        resp.POS_Y.data = req.odomData.pose.pose.position.y + 0.5;
    }
    else if(if_use_gazebo==false && field_direction==LEFT)
    {
        resp.POS_X.data = req.odomData.pose.pose.position.x - 5.5;
        resp.POS_Y.data = req.odomData.pose.pose.position.y + 0.5;
    }

    if(if_use_gazebo==true)
    {
        resp.POS_X.data = req.odomData.pose.pose.position.x;
        resp.POS_Y.data = req.odomData.pose.pose.position.y;
    }

    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"odom_server");
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("/action_msgs_porcess", doReq);
    std::string direction;

    if(!nh.param<bool>("if_use_gazebo",if_use_gazebo,true))
    {
        ROS_ERROR("get if_use_gazebo fail");
    }

    if(nh.param("field_direction",direction,std::string("right")))
    {
        field_direction = stringToPATH_ENUM(direction);
        ROS_INFO("get direction ok!!!");
    }
    else
    {
        ROS_ERROR("get direction failed !!!");
    }
    ROS_INFO("Server have started !");

    ros::spin();
    return 0;
}

