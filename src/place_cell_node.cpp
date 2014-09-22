#include<dolphin_slam/place_cell_network.h>
#include<ros/ros.h>
#include<boost/thread.hpp>

using namespace dolphin_slam;

int main(int argc, char **argv){


    ros::init(argc, argv, "pose_cell_network");


    PlaceCellNetwork place_cell_network;

    ros::spin();

    ROS_INFO_STREAM("Shutting Down Pose Cell Network");

}
