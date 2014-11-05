//#include "dolphin_slam/bow_training.cpp"
#include "dolphin_slam/fabmap_training.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"training");

    FabmapTraining training;


    ros::spin();

}
