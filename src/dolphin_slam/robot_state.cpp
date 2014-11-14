#include "robot_state.h"

const float DVL_ERROR = 0.1;

namespace dolphin_slam
{

RobotState::RobotState(): tf_listener_(buffer_)
{
    has_gt_ = false;

    dr_seq_ = 0;

    loadParameters();
    createROSSubscribers();

}

void RobotState::loadParameters()
{

    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("dvl_topic", parameters_.dvl_topic_, "/g500/dvl");
    private_nh.param<std::string>("imu_topic", parameters_.imu_topic_, "/g500/imu");
    private_nh.param<std::string>("gt_topic", parameters_.gt_topic_, "/ground_truth");


}

void RobotState::createROSSubscribers()
{
    dvl_subscriber_ = node_handle_.subscribe(parameters_.dvl_topic_,1,&RobotState::dvlCallback,this);
    imu_subscriber_ = node_handle_.subscribe(parameters_.imu_topic_,1,&RobotState::imuCallback,this);
}


void RobotState::dvlCallback(const underwater_sensor_msgs::DVLConstPtr &message)
{
    double elapsed_time;
    geometry_msgs::TransformStamped msg;
    tf2::Transform traveled_distance;

    if(dr_seq_ = 0)
    {
        dr_pose_.setIdentity();
    }
    else
    {
        if(fabs(message->bi_error) < 1)
        {
            velocity_.setValue(message->bi_x_axis,message->bi_y_axis,message->bi_z_axis);
        }
        else
        {
            velocity_.setValue(0,0,0);
        }

        elapsed_time = (message->header.stamp - dr_stamp_).toSec();

        traveled_distance = tf2::Transform(dr_pose_.getRotation(),velocity_*elapsed_time);

        dr_pose_ = dr_pose_*traveled_distance;

    }

    dr_stamp_ = message->header.stamp;
    dr_seq_++;

    msg = createTransformStamped(dr_pose_,message->header.stamp,"world","dolphin_slam_dr");
    msg.header.seq = dr_seq_;
    tf_broadcaster_.sendTransform(msg);

}

void RobotState::imuCallback(const sensor_msgs::ImuConstPtr &message)
{
    tf2::Quaternion orientation;

    orientation.setValue(message->orientation.x,message->orientation.y,message->orientation.z,message->orientation.w);

    dr_pose_.setRotation(orientation);
}

void RobotState::groundTruthCallback(const ros::TimerEvent &e)
{
    geometry_msgs::TransformStamped msg;

    try
    {
        msg = buffer_.lookupTransform("world", "girona500", ros::Time(0));

    }
    catch (tf2::TransformException ex )
    {
        ROS_ERROR("%s",ex.what());
        return;
    }

    if(!has_gt_)
    {
        gt_pose_origin_ = getTransform(msg);

        gt_pose_.setIdentity();

        has_gt_ = true;
    }
    else
    {
        gt_pose_ = getTransform(msg);

        gt_pose_ = gt_pose_origin_*gt_pose_;
    }

    createTransformStamped(gt_pose_,msg.header.stamp,"world","dolphin_slam_gt");

    tf_broadcaster_.sendTransform(msg);

}


}   //namespace

