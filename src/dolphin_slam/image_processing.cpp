#include "image_processing.h"

namespace dolphin_slam {



ImageProcessing::ImageProcessing() : it_(node_handle_)
{

    loadParameters();

    init();

    createROSSubscribers();

    createROSPublishers();

    log_file_.open("image_processing.log");

}

ImageProcessing::~ImageProcessing()
{
    log_file_.close();
}


void ImageProcessing::loadParameters()
{
    ros::NodeHandle private_nh_("~");

    //! int surf_threshold;
    private_nh_.param<int>("surf_threshold",parameters_.surf_threshold_,100);

    //! string image_topic;
    private_nh_.param<string>("image_topic",parameters_.image_topic_,"/image_raw");

    //! string image_transport;
    private_nh_.param<string>("image_transport",parameters_.image_transport_,"raw");

    //! string output_topic;
    private_nh_.param<string>("output_topic",parameters_.descriptors_topic_,"/descriptors");

    //! string output_topic;
    private_nh_.param<string>("output_topic",parameters_.image_keypoints_topic_,"/image_keypoints");

    //! int frames_to_jump;
    private_nh_.param<int>("frames_to_jump",parameters_.frames_to_jump_,0);


}


void ImageProcessing::createROSSubscribers()
{

    //! hint to modify the image_transport. Here I use raw transport
    image_transport::TransportHints hints(parameters_.image_transport_,ros::TransportHints(),node_handle_);

    //! image subscription
    image_subscriber_ = it_.subscribe(parameters_.image_topic_,1,&ImageProcessing::imageCallback,this,hints);

}

void ImageProcessing::createROSPublishers()
{

   image_publisher_ = it_.advertise(parameters_.image_keypoints_topic_, 1);

   descriptors_publisher_ = node_handle_.advertise<dolphin_slam::Descriptors>(parameters_.descriptors_topic_, 100);

}



bool ImageProcessing::init()
{
    surf_ = new cv::SURF(parameters_.surf_threshold_);
}


void ImageProcessing::publishImageKeypoints()
{
    cv_bridge::CvImage image_keypoints;

    image_keypoints.header.stamp = image_->header.stamp;
    image_keypoints.header.seq = image_->header.seq;
    image_keypoints.header.frame_id = image_->header.frame_id;

    image_keypoints.encoding = sensor_msgs::image_encodings::BGR8;

    cv::drawKeypoints(image_->image,keypoints_,image_keypoints.image);

    image_publisher_.publish(image_keypoints.toImageMsg());

}


void ImageProcessing::publishDescriptors()
{
    Descriptors msg;

    msg.header.stamp = image_->header.stamp;
    msg.header.seq = image_->header.seq;
    msg.header.frame_id = image_->header.frame_id;

    msg.image_index_ = image_->header.seq;

    msg.descriptor_count_ = descriptors_.rows;
    msg.descriptor_length_ = descriptors_.cols;
    msg.data_.resize(descriptors_.rows*descriptors_.cols);
    std::copy(descriptors_.begin<float>(),descriptors_.end<float>(),msg.data_.begin());
    descriptors_publisher_.publish(msg);

}

void ImageProcessing::imageCallback(const sensor_msgs::ImageConstPtr &image)
{
    static int count = 0;

    if (count == 0){

        image_ = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::MONO8);

        //! Detect SURF keypoints in the image
        surf_->detect(image_->image,keypoints_);
        //! Compute SURF descriptors
        surf_->compute(image_->image,keypoints_,descriptors_);

        publishDescriptors();

        publishImageKeypoints();

    }

    if(parameters_.frames_to_jump_)
        count = (count + 1)%parameters_.frames_to_jump_;


}

} //dolphin_slam
