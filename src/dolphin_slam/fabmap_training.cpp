#include "fabmap_training.h"
#include "bow_descriptor_extractor.h"

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

    private_nh_.param<std::string>("descriptors_topic",parameters_.descriptors_topic_,"\descriptors");

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
    timeout_ = node_handle_.createTimer(ros::Duration(5),&FabmapTraining::train,this,true);
}

void FabmapTraining::init()
{
    cl_trainer_ = new cv::of2::ChowLiuTree();

    bow_trainer_ = new cv::of2::BOWMSCTrainer(parameters_.cluster_size_);

}

void FabmapTraining::descriptorsCallback(const dolphin_slam::DescriptorConstPtr &msg)
{
    cv::Mat descriptors(msg->data);

    //! Stop the timeout
    timeout_.stop();

    surf_descriptors_.push_back(descriptors);

    bow_trainer_->add(descriptors);

    timeout_.start();
}

void FabmapTraining::train(const ros::TimerEvent &)
{

    ROS_DEBUG_STREAM("Start training");

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
    //fs << "descriptors" << bow_descriptors_;
    fs.release();


}
