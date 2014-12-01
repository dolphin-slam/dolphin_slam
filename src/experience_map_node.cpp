#include<dolphin_slam/experience_map.h>
#include<ros/ros.h>

using namespace dolphin_slam;
int main(int argc, char **argv){


    ros::init(argc, argv, "experience_map");


    ExperienceMap experience_map;


    ros::spin();

    ROS_INFO_STREAM("Shutting Down Experience Map");

}
