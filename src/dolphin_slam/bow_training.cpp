#include "bow_training.h"

#include "bow_descriptor_extractor.h"

BoWTraining::BoWTraining()
{

    loadParameters();

    createROSSubscribers();

    createROSTimers();

    init();

}

void BoWTraining::loadParameters()
{
    ros::NodeHandle private_nh_("~");

    private_nh_.param<std::string>("descriptors_topic",parameters_.descriptors_topic_,"\descriptors");

    private_nh_.param<std::string>("bow_vocabulary_path",parameters_.bow_vocabulary_path_,"vocabulary.xml");

    private_nh_.param<double>("cluster_size",parameters_.cluster_count_,0.4);
}

void BoWTraining::createROSSubscribers()
{
    descriptors_subscriber_ = node_handle_.subscribe(parameters_.descriptors_topic_,1,&BoWTraining::descriptorsCallback,this);

}

void BoWTraining::createROSTimers()
{
    timeout_ = node_handle_.createTimer(ros::Duration(5),&BoWTraining::train,this,true);
}

void BoWTraining::init()
{

    bow_trainer_ = new cv::BOWKMeansTrainer(parameters_.cluster_count_,
                                            cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,50,0.001),
                                            5,
                                            cv::KMEANS_PP_CENTERS);
}

void BoWTraining::descriptorsCallback(const dolphin_slam::DescriptorConstPtr &msg)
{
    cv::Mat descriptors(msg->data);

    //! Stop the timeout
    timeout_.stop();

    surf_descriptors_.push_back(descriptors);

    bow_trainer_->add(descriptors);

    timeout_.start();
}

void BoWTraining::train(const ros::TimerEvent &)
{

    ROS_DEBUG_STREAM("Start training");

    bow_vocabulary_ = bow_trainer_->cluster();

    storeTrainingData();

    ROS_DEBUG_STREAM("Finish training");

    ros::requestShutdown();
}



void BoWTraining::storeTrainingData()
{

    cv::FileStorage fs;

    fs.open(parameters_.bow_vocabulary_path_,cv::FileStorage::WRITE);
    fs << "vocabulary" << bow_vocabulary_;
    fs.release();



}
