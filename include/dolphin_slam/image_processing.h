#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv/cv.h>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/foreach.hpp>

#include <string>
#include <fstream>
#include <vector>
#include <queue>

#include <dolphin_slam/Descriptors.h>
#include <dolphin_slam/ImageRequest.h>


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
    string image_topic_;
    string sonar_topic_;
    string image_transport_;
    string sonar_transport_;
    string descriptors_topic_;
    string image_keypoints_topic_;
    int frames_to_jump_;
    string source_;
    string sonar_mask_;
    string image_detector_;
    bool apply_roi_;
    bool use_selected_images_;
    string selected_images_file_;

};


class ImageProcessing
{
public:
    ImageProcessing();
    ~ImageProcessing();

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    bool imageRequest(ImageRequest::Request &req, ImageRequest::Response &res);

    void loadParameters();

    void createROSSubscribers();
    void createROSPublishers();
    void createROSServices();

    void publishImageKeypoints();
    void publishDescriptors();

    bool computeShapeDescriptors(cv::Mat &image);


private:

    bool init();

    std::ofstream log_file_;

    cv::Ptr<cv::SURF> surf_;
    cv::Ptr<cv::SIFT> sift_;
    cv::Ptr<cv::GFTTDetector> gftt_;

    cv::Ptr<cv::CLAHE> clahe;


    ImageProcessingParameters parameters_;

    std::vector<int> selected_images;

    ros::NodeHandle node_handle_;

    ros::Publisher descriptors_publisher_;

    image_transport::ImageTransport it_;
    image_transport::Publisher image_publisher_;
    image_transport::Subscriber image_subscriber_;

    ros::ServiceServer image_service;

    cv::Mat descriptors_;
    std::vector<cv::KeyPoint> keypoints_;

    std::queue < std::pair<int,cv::Mat> > image_buffer_;

    cv_bridge::CvImagePtr image_;

    cv::Mat sonar_mask_;

    bool is_selected_image(int id);

};

} // namespace

#endif // IMAGE_PROCESSING_H
