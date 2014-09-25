#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <image_transport/image_transport.h>
#include <bag_of_features.h>
#include <opencv/cv.h>
#include <dolphin_slam/ImageHistogram.h>
#include <sensor_msgs/image_encodings.h>

#include <fstream>


using namespace std;

namespace dolphin_slam
{

/**
 * @brief The ImageProcessing class.
 * The class works as stated below:
 * * Receive an image on a ROS topic
 * * Extract the SURF descriptors of the image
 * * Create the Bag of Features histogram
 * * Send a message containing the image and the BoF histogram
 */

struct ImageProcessingParameters
{
    int surf_threshold_; //! quanto menor o threshold, maior o número de keypoints encontrados.
    int bof_groups_;
    string bof_vocabulary_path_;
    string image_topic_;
    string image_transport_;
    string output_topic_;
    int frames_to_jump_;
};


class ImageProcessing
{
public:
    ImageProcessing();
    ~ImageProcessing();

    void imageCallback(const sensor_msgs::ImageConstPtr &image);

    void loadParameters();

    void createROSSubscribers();
    void createROSPublishers();

private:

    //Bag of Features related code
    bool computeBoFHistogram();
    bool initBoF();
    void publishOutput();

    bool update();

    std::ofstream log_file_;

    BoF bag_of_features_;

    ImageProcessingParameters parameters_;

    ros::NodeHandle node_handle_;
    ros::Publisher output_publisher_;
    image_transport::Subscriber image_subscriber_;


    cv_bridge::CvImageConstPtr current_image_;

    cv::Mat current_histogram_;

};

} // namespace

#endif // IMAGE_PROCESSING_H
