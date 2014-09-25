#include "image_processing.h"

namespace dolphin_slam {



ImageProcessing::ImageProcessing()
{

    loadParameters();

    initBoF();

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

    //! int bof_groups;
    private_nh_.param<int>("bof_groups",parameters_.bof_groups_,100);

    //! string bof_dictionary_path
    private_nh_.param<string>("bof_vocabulary_path",parameters_.bof_vocabulary_path_,"bof_vocabularies");

    //! string image_topic;
    private_nh_.param<string>("image_topic",parameters_.image_topic_,"/image_raw");

    //! string image_transport;
    private_nh_.param<string>("image_transport",parameters_.image_transport_,"raw");

    //! string output_topic;
    private_nh_.param<string>("output_topic",parameters_.output_topic_,"/image_histogram");

    //! int frames_to_jump;
    private_nh_.param<int>("frames_to_jump",parameters_.frames_to_jump_,0);


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
    string fullpath = parameters_.bof_vocabulary_path_ +
            string("vocabulary_s") + boost::lexical_cast<string>(parameters_.surf_threshold_) +
            string("_g") + boost::lexical_cast<string>(parameters_.bof_groups_) + string(".xml");

    cv::FileStorage fs(fullpath,cv::FileStorage::READ);

    fs["BoF"] >> bag_of_features_;

    //! Testa se os valores do threshold e numero de grupos estão de acordo
    ROS_ASSERT(parameters_.surf_threshold_ == bag_of_features_.getThreshold());
    ROS_ASSERT(parameters_.bof_groups_ == bag_of_features_.getGroups());

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

    publishOutput();

}


void ImageProcessing::imageCallback(const sensor_msgs::ImageConstPtr &image)
{
    static int count = 0;

    if (count == 0){

        current_image_ = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::MONO8);

        update();
    }

    if(parameters_.frames_to_jump_)
        count = (count + 1)%parameters_.frames_to_jump_;


}

} //dolphin_slam
