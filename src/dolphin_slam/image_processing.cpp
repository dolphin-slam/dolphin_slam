#include "image_processing.h"

namespace dolphin_slam {



ImageProcessing::ImageProcessing()
{

    loadParameters();

    createROSSubscribers();

    createROSPublishers();

}

void ImageProcessing::loadParameters()
{
    ros::NodeHandle private_nh_("~");

    //! int surf_threshold;
    private_nh_.param<int>("surf_threshold",parameters_.surf_threshold_,100);

    //! string bof_dictionary_path; Full path
    private_nh_.param<string>("bof_dictionary_path",parameters_.bof_dictionary_path_,"dictionary.xml");

    //! string image_topic;
    private_nh_.param<string>("image_topic",parameters_.image_topic_,"/image_raw");

    //! string image_transport;
    private_nh_.param<string>("image_transport",parameters_.image_transport_,"raw");

    //! string output_topic;
    private_nh_.param<string>("output_topic",parameters_.output_topic_,"/image_histogram");

}


void ImageProcessing::createROSSubscribers()
{
    image_transport::ImageTransport it(node_handle_);

    //! hint to modify the image_transport. Here I use raw transport
    image_transport::TransportHints hints(parameters_.image_transport_,ros::TransportHints(),node_handle_);

    //! image subscription
    image_subscriber_ = it.subscribe(parameters_.image_topic_,1,&ImageProcessing::imageCallback,this,hints);

}

void ImageProcessing::createROSPublishers()
{
    output_publisher_ = node_handle_.advertise<dolphin_slam::ImageHistogram>(parameters_.output_topic_, 100);
}


bool ImageProcessing::initBoF()
{
    bag_of_features_.setThreshold(parameters_.surf_threshold_);

    bag_of_features_.readDictionary(parameters_.bof_dictionary_path_.c_str());

}


void ImageProcessing::publishOutput()
{
    ImageHistogram msg;

    current_image_->toImageMsg(msg.image);

    current_histogram_.copyTo(msg.histogram);

    output_publisher_.publish(msg);

}

bool ImageProcessing::computeBoFHistogram()
{
    current_histogram_ = bag_of_features_.createHistogram(current_image_->image);
}


bool ImageProcessing::update()
{
    //! Do all work here
    computeBoFHistogram();





}


void ImageProcessing::imageCallback(const sensor_msgs::ImageConstPtr &image)
{


    current_image_ = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::MONO8);

    update();

}

} //dolphin_slam
