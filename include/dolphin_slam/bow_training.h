#ifndef BOW_TRAINING_H
#define BOW_TRAINING_H

#include <opencv/cv.h>
#include <dolphin_slam/Descriptors.h>
#include <ros/ros.h>


struct BoWTrainingParameters
{
    std::string descriptors_topic_;
    std::string bow_vocabulary_path_;
    int cluster_count_;
};

class BoWTraining
{
public:
    BoWTraining();

    void loadParameters();

    void createROSSubscribers();

    void createROSTimers();

    void init();

    void descriptorsCallback(const dolphin_slam::DescriptorsConstPtr &msg);

    void train(const ros::TimerEvent&);

    void storeTrainingData();

private:

    ros::NodeHandle node_handle_;
    ros::Subscriber descriptors_subscriber_;

    BoWTrainingParameters parameters_;

    std::vector<cv::Mat> surf_descriptors_;

    cv::Ptr<cv::BOWTrainer> bow_trainer_;

    cv::Mat bow_vocabulary_;

    ros::Timer timeout_;

};

#endif // BOW_TRAINING_H
