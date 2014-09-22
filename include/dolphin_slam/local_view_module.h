#ifndef LOCAL_VIEW_MODULE_H
#define LOCAL_VIEW_MODULE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <dolphin_slam/LocalViewNetwork.h>
#include <dolphin_slam/LocalViewCell.h>
#include <dolphin_slam/ExecutionTime.h>
#include <dolphin_slam/ImageHistogram.h>

#include <opencv/cv.h>


#include <string>

#include <boost/foreach.hpp>

#include <fstream>
#include <time_monitor/time_monitor.h>

#define foreach BOOST_FOREACH

namespace dolphin_slam
{

struct Cell
{
    int id_;
    float rate_;
    cv::Mat data_;
};

struct LocalViewParameters
{
    double similarity_threshold;
    int frames_to_jump_;
};


class LocalViewModule
{
public:
    LocalViewModule();
    ~LocalViewModule();


    void loadParameters();
    void createROSSubscribers();
    void createROSPublishers();


    void createROSTimers();

private:
    //! ROS related functions
    void callback(const dolphin_slam::ImageHistogramConstPtr &message);
    int createViewTemplate(const cv::Mat &histogram);
    void publishViewTemplate();
    void publishExecutionTime();

    void timerCallback(const ros::TimerEvent &event);

    bool computeLocalViewCellActivation(const cv::Mat & histogram);

    bool detectChanges();
    bool detectChanges(const cv::Mat &image);

    LocalViewParameters parameters_;

    std::vector<Cell> cells_;
    int most_active_cell_;
    std::vector <int> active_cells_;

    ros::NodeHandle node_handle_;
    ros::Subscriber image_histogram_subscriber_;
    ros::Publisher view_template_publisher_;
    ros::Publisher execution_time_publisher_;

    std::string image_topic_;

    std::string image_transport_;

//    std::ofstream view_template_file_;
//    std::ofstream local_view_file_;

    TimeMonitor time_monitor_;

    ros::Timer timer_;

    //! Metrics
    int number_of_created_local_views;
    int number_of_recognized_local_views;
    double execution_time;

    bool has_new_local_view_cell;




};
} //namespace

#endif // LOCAL_VIEW_MODULE_H
