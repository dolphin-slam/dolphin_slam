#ifndef FABMAP_TRAINING_H
#define FABMAP_TRAINING_H

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/contrib/openfabmap.hpp>
#include <dolphin_slam/Descriptors.h>
#include <fstream>
#include <string>

namespace dolphin_slam
{

struct FabmapTrainingParameters
{
    std::string fabmap_descriptors_;
    std::string fabmap_tree_;
    double cluster_size_;
    std::string descriptors_topic_;
    std::string fabmap_vocab_;
    int cluster_count_;
    std::string clustering_algorithm_;
    std::string fabmap_implementation_;
    std::string dataset_name_;
    std::string dataset_path_;
    int times_to_use_samples_;
};

class FabmapTraining
{
public:
    FabmapTraining();

    void loadParameters();
    void createROSSubscribers();
    void createROSTimers();
    void descriptorsCallback(const dolphin_slam::DescriptorsConstPtr &msg);

    void init();

    void trainOpen(const ros::TimerEvent&);

    void trainOriginal(const ros::TimerEvent &);
    void trainBoW();
    void trainChowLiuTree();
    void computeBoWDescriptors();
    void storeTrainingData();

    void storeOXV();
    void storeOXS();

    void computeBoWIntegerDescriptors();
private:

    ros::NodeHandle node_handle_;
    ros::Subscriber descriptors_subscriber_;

    FabmapTrainingParameters parameters_;

    std::vector<cv::Mat> surf_descriptors_;
    std::vector<cv::Mat> bow_descriptors_;


    cv::Mat bow_vocabulary_;
    cv::Ptr<cv::BOWTrainer> bow_trainer_;

    cv::Mat cl_tree_;
    cv::Ptr<cv::of2::ChowLiuTree> cl_trainer_;

    ros::Timer timeout_;


};

}
#endif // FABMAP_TRAINING_H
