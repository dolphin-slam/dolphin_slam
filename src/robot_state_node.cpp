#include<dolphin_slam/robot_state.h>
#include<ros/ros.h>

using namespace dolphin_slam;

int main(int argc, char **argv){


    ros::init(argc, argv, "robot_state_node");

    RobotState robot_state;


//    robotState.createROSTimers();

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    ROS_INFO_STREAM("Shutting Down Robot State Node");

}
