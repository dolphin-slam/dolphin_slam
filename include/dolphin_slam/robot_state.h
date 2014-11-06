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

#include <opencv/cv.h>

#include <boost/random.hpp>

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
    void createROSServices();
    void createTfTimers();

private:
    //! função para calcular a distancia percorrida
    void computeTraveledDistances(float elapsed_time);

    //! ROS callbacks
    void DVLCallback(const underwater_sensor_msgs::DVLConstPtr &message); //!< \todo trocar para a mensagem de DVL
    void IMUCallback(const sensor_msgs::ImuConstPtr &message);
    void groundTruthCallback(const nav_msgs::OdometryConstPtr &message);

    //! Tf2 callbacks
    void tf2GroundTruthCallback(const ros::TimerEvent &e);
    void tf2DeadReckoningPublish(const ros::TimerEvent &e);

    bool pcService(RobotPose::Request &req, RobotPose::Response &res);
    bool emService(RobotPose::Request &req, RobotPose::Response &res);


    ros::Time timestamp_;

    RobotStateParameters parameters_;

    //! Novas velocidades e orientação do robô
    cv::Point3f vel_dvl_;
    float yaw_imu_;

    //! Velocidades e orientação atuais do robo
    cv::Point3f robot_vel_;
    float robot_yaw_;

    //! Distâncias percorridas. Controle independente para pose cell e mapa de experiências
    cv::Point3f delta_pc_;
    cv::Point3f delta_em_;

    //! Primeira e atual posições exatas do robô
    cv::Point3f first_ground_truth_;
    cv::Point3f ground_truth_;


    //! ROS Node Handle
    ros::NodeHandle node_handle_;

    //! ROS Topics
    ros::Subscriber DVL_subscriber_;
    ros::Subscriber IMU_subscriber_;
    ros::Subscriber ground_truth_subscriber_;

    //! ROS Timers
    ros::Timer gt_listener_timer_;
    ros::Timer dead_publisher_timer_;

    //! Tf2 Buffers
    tf2_ros::Buffer buffer_;

    //! Tf2 Listeners
    tf2_ros::TransformListener ground_truth_tf2_listener_;

    //! Tf2 Broadcasters
    tf2_ros::TransformBroadcaster dead_reckoning_tf2_broadcaster_;

    //! Tf2 Transforms
    geometry_msgs::TransformStamped gt_transform_;
    geometry_msgs::TransformStamped dead_transform_;


    //! Boost Randon Number Generator
    boost::mt19937 rng_; ///< Boost random number generator
    boost::normal_distribution<> normal_;
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor;

    //! ROS Services
    ros::ServiceServer pc_service_;
    ros::ServiceServer em_service_;

    //! Variáveis de controle para a primeira iteração
    bool has_imu_, has_ground_truth_, has_dvl_;


};

} //namespace


#endif // ROBOT_STATE_H
