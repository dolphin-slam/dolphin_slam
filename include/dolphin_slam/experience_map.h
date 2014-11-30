#ifndef EXPERIENCE_MAP_H
#define EXPERIENCE_MAP_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <utils.h>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <fstream>


#include <experience_map_types.h>

//! Boost
#include <boost/graph/adjacency_list.hpp> //!< biblioteca para grafos
#include <boost/graph/graph_traits.hpp>

#include <visualization_msgs/Marker.h>
#include <boost/foreach.hpp>

#include <dolphin_slam/ExperienceEvent.h>
#include <dolphin_slam/Error.h>
#include <dolphin_slam/ExecutionTime.h>
#include <dolphin_slam/ImageRequest.h>

#include <angles/angles.h>

#include <time_monitor/time_monitor.h>

#define foreach BOOST_FOREACH
#define reverse_foreach BOOST_REVERSE_FOREACH

namespace dolphin_slam
{

struct ExperienceMapParameters
{
    std::string image_topic_;
    std::string image_transport_;
    double match_threshold_;
    double focal_length_;
};

class ExperienceMap
{
public:
    ExperienceMap();
    ~ExperienceMap();

    void loadParameters();
    void createROSSubscribers();
    void createROSPublishers();
    void createROSServices();

    void storeMaps();

private:
    void calculeLocalisationError();

    void createExperience(const ExperienceEventConstPtr &event);

    void computeMatches();

    void getImage(cv::Mat &image, int seq);

    void getGroundTruth(tf2::Transform & gt_pose, ros::Time stamp);

    void getDeadReckoning(tf2::Transform & dr_pose, ros::Time stamp);

    void iterateMap();

    void calculeExperienceMapError();
    void calculeDeadReckoningError();

    bool lookForMatches(const ExperienceEventConstPtr &event, ExperienceDescriptor &similar_experience);

    //! Callbacks
    void experienceEventCallback(const dolphin_slam::ExperienceEventConstPtr &event);

    void createROSMessageMap(visualization_msgs::Marker &message);
    void createROSMessageDeadReckoning(visualization_msgs::Marker &message);
    void createROSMessageGroundTruth(visualization_msgs::Marker &message);
    void createROSMessageError(dolphin_slam::Error &message);

    void publishExperienceMap();
    void publishGroundTruth();
    void publishDeadReckoning();
    void publishError();
    void publishExecutionTime();

    void publishTfDeadReckoning();
    void publishTFPoses();


    ExperienceMapParameters parameters_;

    ExperienceDescriptor current_experience_descriptor_;
    ExperienceDescriptor current_dead_reckoning_descriptor_;

    //! Map
    Map map_;
    Map dead_reckoning_map_;

    float localisationErrorEM_;
    float localisationErrorDR_;

    double experience_map_error_;
    double dead_reckoning_error_;
    cv::Point3d experience_map_independent_error_;
    cv::Point3d dead_reckoning_independent_error_;
    std::ofstream experience_map_error_file_;
    std::ofstream dead_reckoning_error_file_;

    std::ofstream experience_map_file_;

    //ROS related variables
    //! ROS Node Handle
    ros::NodeHandle node_handle_;

    //! ROS Topics
    ros::Publisher map_publisher_;
    ros::Publisher dead_reckoning_publisher_;
    ros::Publisher ground_truth_publisher_;
    ros::Publisher error_publisher_;
    ros::Publisher execution_time_publisher_;
    ros::Subscriber experience_event_subscriber_;

    ros::ServiceClient image_client_;

    //! Transformation Frames Library
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    int max_id_experience_;
    int test_number_;

    TimeMonitor time_monitor_;

    //! metrics
    int number_of_created_experiences_;
    int number_of_recognized_experiences_;
    double execution_time_;


    image_transport::ImageTransport it_;
    image_transport::Publisher image_publisher_;
    cv::Ptr<cv::SIFT> sift_;

    int image_index_begin_;
    int image_index_end_;

    cv::FlannBasedMatcher matcher_;


    tf2::Vector3 getImageTransform(cv::Mat &current_image, cv::Mat &image);
};

}  //namespace

#endif // EXPERIENCE_MAP_H
