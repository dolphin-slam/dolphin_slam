#include<dolphin_slam/local_view_module.h>
#include<ros/ros.h>

using namespace dolphin_slam;

int main(int argc, char **argv){


    ros::init(argc, argv, "local_view_module");

    LocalViewModule local_view_module;

    ros::spin();

    ROS_INFO_STREAM("Shutting Down Local View Module");

}
