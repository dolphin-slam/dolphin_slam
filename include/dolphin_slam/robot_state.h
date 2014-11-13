#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/Imu.h>
#include <underwater_sensor_msgs/DVL.h>

#include <dolphin_slam/RobotPose.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>


#include <boost/random.hpp>

#include <utils.h>


namespace dolphin_slam
{

struct RobotStateParameters
{
    std::string dvl_topic_;
    std::string imu_topic_;
    std::string gt_topic_;
};

//! \todo Mudar a interface para um tf, possivelmente

class RobotState
{
public:
    //! Construtor
    RobotState();

    void loadParameters();
    void createROSSubscribers();
    void createROSTimers();

private:
    //! função para calcular a distancia percorrida
    void computeTraveledDistances(float elapsed_time);

    //! ROS callbacks
    void dvlCallback(const underwater_sensor_msgs::DVLConstPtr &message); //!< \todo trocar para a mensagem de DVL
    void imuCallback(const sensor_msgs::ImuConstPtr &message);

    //! Tf2 callbacks
    void groundTruthCallback(const ros::TimerEvent &e);


    RobotStateParameters parameters_;

    //! Robot velocity and orientation
    tf2::Vector3 velocity_;

    tf2::Transform dr_pose_;
    unsigned int dr_seq_;   //! dead reckoning sequencer
    ros::Time dr_stamp_;


    //! Ground Truth
    tf2::Transform gt_pose_;
    tf2::Transform gt_pose_origin_;
    bool has_gt_;

    //! ROS Node Handle
    ros::NodeHandle node_handle_;

    //! ROS Topics
    ros::Subscriber dvl_subscriber_;
    ros::Subscriber imu_subscriber_;

    //! TF2
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;


};

} //namespace


#endif // ROBOT_STATE_H
