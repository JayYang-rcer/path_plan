#include "ros/ros.h"
#include "rc_msgs/IbusData.h"
#include "hand_control/hand_control.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hand_control");
    
    hand_control_ns::HandControl hand_control;

    hand_control.run();
    return 0;
}
