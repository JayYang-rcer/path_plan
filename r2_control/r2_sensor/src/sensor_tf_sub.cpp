#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" 
#include "r2_msgs/action_cmd.h"

float offset_x,offset_y,offset_z;

bool doReq(r2_msgs::action_cmd::Request& req,r2_msgs::action_cmd::Response& resp)
{
    //create a buffer
    static tf2_ros::Buffer buffer;
    static tf2_ros::TransformListener listener(buffer);
    geometry_msgs::PointStamped point_laser;
    point_laser.header.frame_id = "action";
    point_laser.header.stamp = ros::Time();

    //the point in action frame
    point_laser.point.x = offset_x;
    point_laser.point.y = offset_y;
    point_laser.point.z = offset_z;
    
    try
    {
        geometry_msgs::PointStamped point_base;
        point_base = buffer.transform(point_laser,"world",ros::Duration(0.5));
        resp.POS_X.data = point_base.point.x;
        resp.POS_Y.data = point_base.point.y;
        // coordinate.x = point_base.point.x;
        // coordinate.y = point_base.point.y;
        // pub.publish(coordinate);
    }
    catch(const std::exception& e)
    {
        // std::cerr << e.what() << '\n';
        ROS_INFO("ERROR:%s",e.what());
    }
    
    return true;
}


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"dynamic_tf_sub");
    ros::NodeHandle nh;

    if(!nh.param<float>("/tf_server/offset_x",offset_x,0.0)||
       !nh.param<float>("/tf_server/offset_y",offset_y,0.0)||
       !nh.param<float>("/tf_server/offset_z",offset_z,0.0))
    {
        ROS_ERROR("Failed to get the offset value!");
    }

    //create a coordinate publisher
    ros::Publisher pub = nh.advertise<geometry_msgs::Point>("/robot_coordinate",1);
    ros::ServiceServer server = nh.advertiseService("/tf_server", doReq);

    ROS_INFO("TF Server have started !");
    ros::spin();
    return 0;
}
