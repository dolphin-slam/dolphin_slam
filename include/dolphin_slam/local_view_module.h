#ifndef LOCAL_VIEW_MODULE_H
#define LOCAL_VIEW_MODULE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <dolphin_slam/ExecutionTime.h>
#include <dolphin_slam/ActiveLocalViewCells.h>
#include <dolphin_slam/Descriptors.h>

#include <opencv2/core/core.hpp>
#include <opencv2/contrib/openfabmap.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include <dolphin_slam/bow_descriptor_extractor.h>

#include <string>

#include <boost/foreach.hpp>

#include <fstream>
#include <time_monitor/time_monitor.h>

#include "boost/date_time/gregorian/gregorian.hpp"

#include <fabmap_training.h>

#include <FabMap.h>


#define foreach BOOST_FOREACH

namespace dolphin_slam
{

struct LocalViewParameters
{
    double similarity_threshold_;
    std::string matching_algorithm_;
    std::string bow_vocab_;
    std::string fabmap_vocab_;
    std::string fabmap_tree_;
    std::string fabmap_descriptors_;
    std::string fabmap_algorithm_;
    std::string fabmap_config_;
    std::string descriptors_topic_;
    bool fabmap_confirm_match_;

};


struct LocalViewMetrics
{
    int creation_count_;
    int recognition_count_;
    double execution_time_;
};

struct LocalViewCell
{
    int id_;
    double rate_;
    bool active_;

};

class LocalViewModule
{
public:
    LocalViewModule();
    ~LocalViewModule();


    void loadParameters();
    void createROSSubscribers();
    void createROSPublishers();

    void init();


    void createROSTimers();

private:
    //! ROS related functions
    void descriptors_callback(const dolphin_slam::DescriptorsConstPtr &msg);
    void createNewCell();
    void publishExecutionTime();
    void publishActiveCells();

    void writeLog();

    void timerCallback(const ros::TimerEvent &event);

    void computeMatches();
    void computeCorrelations();
    void computeFabmap();

    void computeImgDescriptor(cv::Mat &descriptors);
    void compare();

    LocalViewParameters parameters_;


    cv::Ptr<BOWImgDescriptorExtractor> bow_extractor_;//! \todo Usar a versao do opencv ao atualizar para o opencv 3.0
    cv::Mat bow_vocabulary_;

    cv::Ptr<cv::of2::FabMap> fabmap_open_;

    cv::Ptr<FabMapCalculator> fabmap_original_;

    LocationProbabilityContainer computed_location_probability;

    cv::Mat cltree_;
    cv::Mat bow_training_descriptors_;

    cv::Mat bow_current_descriptor_;
    std::vector<cv::Mat> bow_descriptors_;
    std::vector<LocalViewCell> cells_;
    bool new_place_;

    double new_rate_;
    int best_match_id_;

    int last_best_match_id_;

    int image_seq_;
    ros::Time image_stamp_;

    ros::NodeHandle node_handle_;
    ros::Subscriber descriptors_subscriber_;
    ros::Publisher active_cells_publisher_;
    ros::Publisher execution_time_publisher_;


    std::ofstream log_file_rate_;
    std::ofstream log_file_metrics_;
    std::ofstream log_file_bow_;


    TimeMonitor time_monitor_;

    ros::Timer timer_;

    //! Metrics
    LocalViewMetrics metrics_;

    bool has_new_local_view_cell;

    ros::Time start_stamp_;



};
} //namespace

#endif // LOCAL_VIEW_MODULE_H
