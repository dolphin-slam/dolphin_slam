#ifndef EXPERIENCE_MAP_H
#define EXPERIENCE_MAP_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


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

#include <angles/angles.h>

#include <time_monitor/time_monitor.h>

#define foreach BOOST_FOREACH
#define reverse_foreach BOOST_REVERSE_FOREACH

namespace dolphin_slam
{

struct ExperienceMapParameters
{
    double match_threshold_;
};

class ExperienceMap
{
public:
    ExperienceMap();
    ~ExperienceMap();

    void loadParameters();
    void createROSSubscribers();
    void createROSPublishers();

    void storeMaps();

private:
    void calculeLocalisationError();

    void createFirstExperience(const ExperienceEventConstPtr &event);
    void createNewExperience(const ExperienceEventConstPtr &event);
    void linkSimilarExperience(const ExperienceEventConstPtr &event, ExperienceDescriptor &similar_experience);

    void createFirstDeadReckoning(const ExperienceEventConstPtr &event);
    void createDeadReckoning(const ExperienceEventConstPtr &event);

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
    void publishTfExperienceMap();

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

    //! Tf2 Broadcaster
    tf2_ros::TransformBroadcaster experience_map_tf_broadcaster_;

    //! Geometry Messages for broadcasting
    geometry_msgs::TransformStamped transform_dead_;
    geometry_msgs::TransformStamped transform_map_;

    int max_id_experience_;
    int test_number_;

    TimeMonitor time_monitor_;

    //! metrics
    int number_of_created_experiences_;
    int number_of_recognized_experiences_;
    double execution_time_;


};

}  //namespace

#endif // EXPERIENCE_MAP_H
