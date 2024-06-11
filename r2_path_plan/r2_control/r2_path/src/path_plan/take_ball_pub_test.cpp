#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "r2_msgs/path_cmd.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"take_ball_pub_test");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<r2_msgs::path_cmd>("/ball_pos",1);

    r2_msgs::path_cmd ball_pos;

    ball_pos.ball_position.x = 3000;
    ball_pos.ball_position.y = 3000;
    ball_pos.ball_position.z = 0;
    ball_pos.PathMode = ball_pos.ThreeB_e;

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        pub.publish(ball_pos);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
