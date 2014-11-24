#include "fabmap_training.h"
#include "bow_descriptor_extractor.h"
#include <utils.h>
#include <boost/foreach.hpp>
#include <iostream>

namespace dolphin_slam
{

FabmapTraining::FabmapTraining()
{

    loadParameters();

    createROSSubscribers();

    createROSTimers();

    init();

}


void FabmapTraining::loadParameters()
{
    ros::NodeHandle private_nh_("~");

    private_nh_.param<std::string>("descriptors_topic",parameters_.descriptors_topic_,"/descriptors");

    private_nh_.param<std::string>("bow_vocabulary_path",parameters_.bow_vocabulary_path_,"vocabulary.xml");

    private_nh_.param<std::string>("cltree_path",parameters_.cltree_path_,"cltree.xml");

    private_nh_.param<std::string>("bow_descriptors_path",parameters_.bow_descriptors_path_,"descriptors.xml");

    private_nh_.param<double>("cluster_size",parameters_.cluster_size_,0.4);
}

void FabmapTraining::createROSSubscribers()
{
    descriptors_subscriber_ = node_handle_.subscribe(parameters_.descriptors_topic_,1,&FabmapTraining::descriptorsCallback,this);
}

void FabmapTraining::createROSTimers()
{
    timeout_ = node_handle_.createTimer(ros::Duration(20),&FabmapTraining::train,this,false);
}

void FabmapTraining::init()
{
    bow_trainer_ = new cv::of2::BOWMSCTrainer(parameters_.cluster_size_);

    cl_trainer_ = new cv::of2::ChowLiuTree();

}

void FabmapTraining::descriptorsCallback(const dolphin_slam::DescriptorsConstPtr &msg)
{
    cv::Mat_<float> descriptors(msg->descriptor_count_,msg->descriptor_length_);
    std::copy(msg->data_.begin(),msg->data_.end(),descriptors.begin());

    //! Stop the timeout
    timeout_.stop();

    surf_descriptors_.push_back(descriptors);

    bow_trainer_->add(descriptors);

    timeout_.start();
}


void FabmapTraining::train(const ros::TimerEvent &)
{

    ROS_DEBUG_STREAM("Start training");

    ROS_DEBUG_STREAM("Number of images: " << surf_descriptors_.size());

    bow_vocabulary_ = bow_trainer_->cluster();

    computeBoWDescriptors();

    cl_trainer_->add(bow_descriptors_);

    cl_tree_ = cl_trainer_->make();

    storeTrainingData();

    ROS_DEBUG_STREAM("Finish training");

    ros::requestShutdown();
}


void FabmapTraining::computeBoWDescriptors()
{
    //! Compute BoW descriptors

    BOWImgDescriptorExtractor extractor(cv::DescriptorMatcher::create("FlannBased")); //! \todo Usar a versao do opencv ao atualizar para o opencv 3.0
    cv::Mat image_descriptor;

    extractor.setVocabulary(bow_vocabulary_);


    for(int i=0;i<surf_descriptors_.size();i++)
    {
        extractor.compute(surf_descriptors_[i],image_descriptor);

        bow_descriptors_.push_back(image_descriptor);

    }
}

void FabmapTraining::storeTrainingData()
{

    cv::FileStorage fs;

    fs.open(parameters_.bow_vocabulary_path_,cv::FileStorage::WRITE);
    fs << "vocabulary" << bow_vocabulary_;
    fs.release();

    fs.open(parameters_.cltree_path_,cv::FileStorage::WRITE);
    fs << "tree" << cl_tree_;
    fs.release();

    //! \todo Transformar em uma matriz primeiro, para depois salvar os valores
    fs.open(parameters_.bow_descriptors_path_,cv::FileStorage::WRITE);
    cv::Mat_ <float>descriptors;
    convert(bow_descriptors_,descriptors);
    fs << "descriptors" << descriptors;
    fs.release();


}

}
