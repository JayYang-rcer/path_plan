//1.包含头文件
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件

float ball_x = 0.0, ball_y=0.0;



void camera_callback(const geometry_msgs::PoseStamped::ConstPtr& camera)
{
    ball_y = camera->pose.position.x/1000;  
    ball_x = -camera->pose.position.y/1000;
}


int main(int argc, char *argv[])
{
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"dynamic_tf_sub");
    ros::NodeHandle nh;
    setlocale(LC_ALL,"");

    std::string camera_topic;
    if(!nh.param<std::string>("camera_topic",camera_topic,"/camera"))
    {
        ROS_ERROR("get camera_topic fail");
    }

    ros::Subscriber camera_sub = nh.subscribe<geometry_msgs::PoseStamped>("/camera1/ball_position",1,camera_callback);
    ros::Publisher ball_position_pub = nh.advertise<geometry_msgs::Point>("/ball_worldPos",1);

    geometry_msgs::Point ball_position;
    // 3.创建 TF 订阅节点
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Rate r(50);
    while (ros::ok())
    {
    // 4.生成一个坐标点(相对于子级坐标系)
        geometry_msgs::PointStamped point_laser;
        point_laser.header.frame_id = "robot";
        point_laser.header.stamp = ros::Time();
        point_laser.point.x = ball_x;
        point_laser.point.y = ball_y;
        point_laser.point.z = 0;
        ROS_INFO("坐标点相对于 world 的坐标为:(%.2f,%.2f,%.2f)",point_laser.point.x,point_laser.point.y,point_laser.point.z);
        
    // 5.转换坐标点(相对于父级坐标系)
        //新建一个坐标点，用于接收转换结果  
        //--------------使用 try 语句或休眠，否则可能由于缓存接收延迟而导致坐标转换失败------------------------
        try
        {
            geometry_msgs::PointStamped point_base;
            point_base = buffer.transform(point_laser,"world");
            ROS_INFO("坐标点相对于 world 的坐标为:(%.2f,%.2f,%.2f)",point_base.point.x,point_base.point.y,point_base.point.z);

            ball_position.x = point_base.point.x;
            ball_position.y = point_base.point.y;
            ball_position.z = 0;
            ball_position_pub.publish(ball_position);
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("程序异常:%s",e.what());
        }

        r.sleep();  
        ros::spinOnce();
    }
    return 0;
}
