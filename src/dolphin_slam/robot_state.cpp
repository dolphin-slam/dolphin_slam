#include "robot_state.h"

namespace dolphin_slam
{

RobotState::RobotState(): tf_listener_(buffer_)
{
    has_gt_ = false;
    dr_seq_ = 0;

    loadParameters();
    createROSSubscribers();
    createROSTimers();

}



void RobotState::loadParameters()
{

    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("dvl_topic", parameters_.dvl_topic_, "/g500/dvl");
    private_nh.param<std::string>("imu_topic", parameters_.imu_topic_, "/g500/imu");

    private_nh.param<std::string>("base_frame", parameters_.base_frame_, "girona500");

    private_nh.param<std::string>("dvl_frame", parameters_.dvl_frame_, "DVLSensor");


}

void RobotState::createROSSubscribers()
{
    dvl_subscriber_ = node_handle_.subscribe(parameters_.dvl_topic_,1,&RobotState::dvlCallback,this);
    imu_subscriber_ = node_handle_.subscribe(parameters_.imu_topic_,1,&RobotState::imuCallback,this);
}

void RobotState::createROSTimers()
{
    gt_timer_ = node_handle_.createTimer(ros::Duration(0.1),&RobotState::groundTruthCallback,this);
}

void RobotState::dvlCallback(const underwater_sensor_msgs::DVLConstPtr &message)
{
    double elapsed_time;
    geometry_msgs::TransformStamped msg;
    tf2::Transform traveled_distance;

    ros::Time stamp;

    stamp = message->header.stamp;
    if(stamp.sec == 0){
        stamp = ros::Time::now();
    }

    msg = buffer_.lookupTransform(parameters_.dvl_frame_, parameters_.base_frame_, ros::Time(0));
    dvl2base_transform = getTransform(msg).inverse();

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

        //! found velocity of base frame, or translation of base frame
        velocity_ = dvl2base_transform*velocity_;

        elapsed_time = (stamp - dr_stamp_).toSec();

        traveled_distance = tf2::Transform(dr_pose_.getRotation(),velocity_*elapsed_time);


        dr_pose_ = dr_pose_*traveled_distance;

    }

    ROS_DEBUG_STREAM("DVL Velocity = [" << velocity_.x() << " " << velocity_.y() << " " << velocity_.z() << " ]" << "elapsed_time = " << elapsed_time);

    dr_stamp_ = stamp;
    dr_seq_++;

    msg = createTransformStamped(dr_pose_,stamp,"world","dolphin_slam_dr");
    msg.header.seq = dr_seq_;
    tf_broadcaster_.sendTransform(msg);

}

void RobotState::imuCallback(const sensor_msgs::ImuConstPtr &message)
{
    tf2::Quaternion orientation;

    orientation.setValue(message->orientation.x,message->orientation.y,message->orientation.z,message->orientation.w);

    dr_pose_.setRotation(orientation);
}

void RobotState::groundTruthCallback(const ros::TimerEvent &event)
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

        gt_pose_ = gt_pose_origin_.inverseTimes(gt_pose_);
    }

    msg = createTransformStamped(gt_pose_,msg.header.stamp,"world","dolphin_slam_gt");

    tf_broadcaster_.sendTransform(msg);

}


}   //namespace

