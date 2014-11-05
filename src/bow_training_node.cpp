#include "dolphin_slam/bow_training.h"
#include<ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"training");

    BoWTraining training;


    ros::spin();

}

