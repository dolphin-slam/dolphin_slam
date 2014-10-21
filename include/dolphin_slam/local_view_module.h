#ifndef LOCAL_VIEW_MODULE_H
#define LOCAL_VIEW_MODULE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <dolphin_slam/ExecutionTime.h>
#include <dolphin_slam/ActiveLocalViewCells.h>
#include <dolphin_slam/ImageHistogram.h>

#include <opencv/cv.h>

#include <string>

#include <boost/foreach.hpp>

#include <fstream>
#include <time_monitor/time_monitor.h>

#include "boost/date_time/gregorian/gregorian.hpp"


#define foreach BOOST_FOREACH

namespace dolphin_slam
{

struct LocalViewCell
{
    int id_;
    double rate_;
    bool active_;
    cv::Mat data_;
};

struct LocalViewParameters
{
    double similarity_threshold_;
    std::string local_view_activation_;
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
    int createNewCell(const cv::Mat &histogram);
    void publishOutput();
    void publishExecutionTime();

    void timerCallback(const ros::TimerEvent &event);

    void computeRate(const cv::Mat & histogram);

    bool detectChanges();
    bool detectChanges(const cv::Mat &image);

    LocalViewParameters parameters_;

    std::vector<LocalViewCell> cells_;

    ros::NodeHandle node_handle_;
    ros::Subscriber image_histogram_subscriber_;
    ros::Publisher output_publisher_;
    ros::Publisher execution_time_publisher_;


    std::ofstream log_file_;
//    std::ofstream view_template_file_;
//    std::ofstream local_view_file_;

    ActiveLocalViewCells output_message_;

    TimeMonitor time_monitor_;

    ros::Timer timer_;

    //! Metrics
    int number_of_created_local_views;
    int number_of_recognized_local_views;
    double execution_time;

    bool has_new_local_view_cell;

    ros::Time start_stamp_;



};
} //namespace

#endif // LOCAL_VIEW_MODULE_H
