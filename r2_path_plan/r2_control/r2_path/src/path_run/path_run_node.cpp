#include "r2_path/path_run.h"
#include "ros/ros.h"
#include "r2_path/path_base.h"

int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc,argv,"plan_run");
    
    path_run p;
    p.run();
    
    return 0;
}

