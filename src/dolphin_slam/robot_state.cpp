#include "robot_state.h"

namespace dolphin_slam
{

RobotState::RobotState(): tf_listener_(buffer_)
{
    tf2::Quaternion quat;


    has_gt_ = false;

    has_imu_ = false;

    dr_seq_ = 0;

    //! inicializa a posição e orientação da dvl com valor padrões
    parameters_.dvl_position_.resize(3,0.0);
    parameters_.dvl_orientation_.resize(3,0);

    loadParameters();
    createROSSubscribers();
    createROSTimers();



    dvl2base_transform_.setOrigin(tf2::Vector3 (parameters_.dvl_position_[0],parameters_.dvl_position_[1],parameters_.dvl_position_[2]));
    quat.setRPY(parameters_.dvl_orientation_[0],parameters_.dvl_orientation_[1],parameters_.dvl_orientation_[2]);
    dvl2base_transform_.setRotation(quat);//! ordem inversa dos angulos de euler

}



void RobotState::loadParameters()
{

    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("dvl_topic", parameters_.dvl_topic_, "/g500/dvl");
    private_nh.param<std::string>("imu_topic", parameters_.imu_topic_, "/g500/imu");

    private_nh.param<std::string>("base_frame", parameters_.base_frame_, "girona500");

    private_nh.getParam("dvl_position",parameters_.dvl_position_);
    private_nh.getParam("dvl_orientation",parameters_.dvl_orientation_);




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


    //! DVL pose in world frame at t = T1
    tf2::Transform dvl_pose_t1;

    //! DVL pose in world frame at t = T2
    tf2::Transform dvl_pose_t2;

    //! Robot pose in world frame at t = T1
    tf2::Transform robot_pose_t1;

    //! Robot pose in world frame at t = T2
    tf2::Transform robot_pose_t2;

    tf2::Vector3 dvl_translation;



    ros::Time stamp;


    ROS_DEBUG_STREAM("DVL Velocity = [" << message->bi_x_axis << " " << message->bi_y_axis  << " " << message->bi_z_axis  << " ]" << "elapsed_time = " << elapsed_time);

    stamp = message->header.stamp;
    if(stamp.sec == 0){
        stamp = ros::Time::now();
    }

    if(dr_seq_ = 0)
    {
        dr_pose_.setIdentity();
        dr_stamp_ = stamp;
        dr_seq_++;

        msg = createTransformStamped(dr_pose_,stamp,"world","rp2");
        msg.header.seq = dr_seq_;
        tf_broadcaster_.sendTransform(msg);

    }
    else if(fabs(message->bi_error) < 1)
    {
        velocity_.setValue(message->bi_x_axis,message->bi_y_axis,message->bi_z_axis);

        //! Compute elapsed time in seconds
        elapsed_time = (stamp - dr_stamp_).toSec();

        //! robot pose on time t = T1
        robot_pose_t1 = dr_pose_;

        msg = createTransformStamped(robot_pose_t1,stamp,"world","rp1");
        msg.header.seq = dr_seq_;
        tf_broadcaster_.sendTransform(msg);

        //! dvl pose on time t = T1
        dvl_pose_t1 = robot_pose_t1*dvl2base_transform_;
        msg = createTransformStamped(dvl_pose_t1,stamp,"world","dvl1");
        msg.header.seq = dr_seq_;
        tf_broadcaster_.sendTransform(msg);

        //! dvl pose on time t= T2
        dvl_translation = velocity_*elapsed_time;
        dvl_pose_t2.setOrigin(dvl_pose_t1*dvl_translation);
        //! dvl orientation is set to robot orientation
        dvl_pose_t2.setRotation(orientation_);

        msg = createTransformStamped(dvl_pose_t2,stamp,"world","dvl2");
        msg.header.seq = dr_seq_;
        tf_broadcaster_.sendTransform(msg);

        robot_pose_t2.setOrigin(dvl_pose_t2* -(dvl2base_transform_.getOrigin()));
        robot_pose_t2.setRotation(orientation_);

        dr_pose_ = robot_pose_t2;

//        //! compute relative dvl rotation and translation
//        dvl_rotation = last_orientation * new_orientation.transpose();
//        dvl_translation = velocity_*elapsed_time;

//        //! create dvl transform
//        dvl_transform.setOrigin(dvl_translation);
//        dvl_transform.setBasis(dvl_rotation);

//        std::cout << "dvl_translation = " << dvl_translation.x() << " " <<dvl_translation.y() << " " << dvl_translation.z() << " " << std::endl;

//        relative_robot_position = dvl2base_transform_.inverse()*relative_robot_position;
//        relative_robot_position = dvl_transform*relative_robot_position;
//        relative_robot_position = dvl2base_transform_*relative_robot_position;

//        absolute_robot_position = dr_pose_ * relative_robot_position;

//        dr_pose_.setOrigin(absolute_robot_position);
//        dr_pose_.setRotation(orientation_);

        dr_stamp_ = stamp;
        dr_seq_++;

        msg = createTransformStamped(dr_pose_,stamp,"world","rp2");
        msg.header.seq = dr_seq_;
        tf_broadcaster_.sendTransform(msg);

    }




}

void RobotState::imuCallback(const sensor_msgs::ImuConstPtr &message)
{
    tf2::Quaternion imu;

    if(!has_imu_)
    {
        imu_origin_.setValue(message->orientation.x,message->orientation.y,message->orientation.z,message->orientation.w);
        has_imu_ = true;
        orientation_.setEulerZYX(0,0,0);
    }
    else
    {
        imu.setValue(message->orientation.x,message->orientation.y,message->orientation.z,message->orientation.w);
        orientation_ = imu_origin_.inverse()*imu;
    }


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

    msg = createTransformStamped(gt_pose_,msg.header.stamp,"world","dolphin_slam/ground_truth");

    tf_broadcaster_.sendTransform(msg);

}


}   //namespace

