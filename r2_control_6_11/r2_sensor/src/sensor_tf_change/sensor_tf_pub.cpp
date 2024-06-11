/**
 * @brief 里程计坐标系的发布方
*/
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "r2_msgs/action_cmd.h"

ros::ServiceClient odom_client_;
ros::ServiceClient tf_client_;
ros::Publisher pub;
geometry_msgs::Point coordinate;
bool if_use_gazebo;


void doPose(const nav_msgs::Odometry::ConstPtr& pose){

    r2_msgs::action_cmd action_srv;
    action_srv.request.odomData = *pose;
    float theta = 0;

    if(odom_client_.call(action_srv))
	{
		theta = action_srv.response.Yaw.data/180*3.14159;
	}
	else
	{
		ROS_ERROR("Failed to call the service");
	}

    //creat a broadcaster
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;
    
    //set the header
    tfs.header.frame_id = "world";
    tfs.header.stamp = ros::Time::now();

    //child frame
    tfs.child_frame_id = "robot";

    tfs.transform.translation.x = action_srv.response.POS_X.data;
    tfs.transform.translation.y = action_srv.response.POS_Y.data;
    tfs.transform.translation.z = 0.0;
    
    //set the rotation
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,-theta);
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();

    // publish the transform
    broadcaster.sendTransform(tfs);

    if(if_use_gazebo==1)    //使用仿真
    {
        coordinate.x = pose->pose.pose.position.x;
        coordinate.y = pose->pose.pose.position.y;
        coordinate.z = action_srv.response.Yaw.data;
        pub.publish(coordinate);
        // ROS_INFO("x=%f,y=%f,z=%f",coordinate.x,coordinate.y,coordinate.z);
    }
    else
    {
        r2_msgs::action_cmd tf_srv;
        if(tf_client_.call(tf_srv))
        {
            coordinate.x = tf_srv.response.POS_X.data;
            coordinate.y = tf_srv.response.POS_Y.data;
            coordinate.z = -action_srv.response.Yaw.data;
            pub.publish(coordinate);
            // ROS_INFO("x=%f,y=%f,z=%f",coordinate.x,coordinate.y,coordinate.z);
        }
        else
        {
            ROS_ERROR("Failed to call the service");
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"dynamic_tf_pub");
   
    ros::NodeHandle nh;
    if(!nh.param<bool>("if_use_gazebo",if_use_gazebo,true))
    {
        ROS_ERROR("get if_use_gazebo fail");
    }

    std::string odom_topic;
    if(!nh.param("odom_topic",odom_topic,std::string("/odom")))
    {
        ROS_ERROR("get odom_topic fail");
    }
    
    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(odom_topic,1,doPose);
    odom_client_ = nh.serviceClient<r2_msgs::action_cmd>("/action_msgs_porcess");
    tf_client_ = nh.serviceClient<r2_msgs::action_cmd>("/tf_server");
    pub = nh.advertise<geometry_msgs::Point>("/robot_coordinate",1);    
        
    // 6.spin
    ros::spin();
    return 0;
}
