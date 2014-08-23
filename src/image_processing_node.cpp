#include <ros/ros.h>
#include <image_processing.h>

using namespace dolphin_slam;

int main(int argc, char **argv)

{
    ros::init(argc,argv,"image_processing");

    ImageProcessing image_processing;

    ros::spin();


}
