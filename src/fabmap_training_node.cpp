#include "dolphin_slam/fabmap_training.h"
#include <ros/ros.h>
int main(int argc, char **argv)
{
    ros::init(argc,argv,"training");

    FabmapTraining training;


    ros::spin();

}
