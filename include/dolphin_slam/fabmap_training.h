#ifndef FABMAP_TRAINING_H
#define FABMAP_TRAINING_H

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/contrib/openfabmap.hpp>
#include <dolphin_slam/Descriptors.h>

namespace dolphin_slam
{

struct FabmapTrainingParameters
{
    std::string bow_descriptors_path_;
    std::string cltree_path_;
    double cluster_size_;
    std::string descriptors_topic_;
    std::string bow_vocabulary_path_;
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

    void train(const ros::TimerEvent&);

    void trainBoW();
    void trainChowLiuTree();
    void computeBoWDescriptors();
    void storeTrainingData();

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
