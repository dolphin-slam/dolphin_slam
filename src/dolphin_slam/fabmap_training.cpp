#include "fabmap_training.h"
#include "bow_descriptor_extractor.h"
#include <utils.h>
#include <boost/foreach.hpp>
#include <iostream>

#include <pAcceleratedChowLiu_CompactMemory/include/AcceleratedCL.h>

using std::endl;

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

    private_nh_.param<std::string>("fabmap_vocab",parameters_.fabmap_vocab_,"vocabulary.xml");

    private_nh_.param<std::string>("fabmap_tree",parameters_.fabmap_tree_,"cltree.xml");

    private_nh_.param<std::string>("fabmap_descriptors",parameters_.fabmap_descriptors_,"descriptors.xml");

    private_nh_.param<double>("cluster_size",parameters_.cluster_size_,0.4);

    private_nh_.param<int>("cluster_count",parameters_.cluster_count_,100);

    private_nh_.param<std::string>("clustering_algorithm",parameters_.clustering_algorithm_,"radial");

    private_nh_.param<std::string>("fabmap_implementation",parameters_.fabmap_implementation_,"original");

    private_nh_.param<std::string>("dataset_name",parameters_.dataset_name_,"dataset_path");

    private_nh_.param<std::string>("dataset_path",parameters_.dataset_path_,"dataset");

    private_nh_.param<int>("times_to_use_samples",parameters_.times_to_use_samples_,1);

}


void FabmapTraining::createROSSubscribers()
{
    descriptors_subscriber_ = node_handle_.subscribe(parameters_.descriptors_topic_,1,&FabmapTraining::descriptorsCallback,this);
}

void FabmapTraining::createROSTimers()
{

    if(parameters_.fabmap_implementation_ == "original")
    {
        timeout_ = node_handle_.createTimer(ros::Duration(20),&FabmapTraining::trainOriginal,this,true,false);
    }
    else if (parameters_.fabmap_implementation_ == "open")
    {
        timeout_ = node_handle_.createTimer(ros::Duration(20),&FabmapTraining::trainOpen,this,true,false);
    }
    else
    {
        ROS_ERROR_STREAM("Wrong fabmap implementation: " << parameters_.fabmap_implementation_);
    }
}

void FabmapTraining::init()
{

    if(parameters_.clustering_algorithm_ == "radial")
    {
        ROS_DEBUG_STREAM("clustering algorithm = radial");
        bow_trainer_ = new cv::of2::BOWMSCTrainer(parameters_.cluster_size_);
    }
    else if(parameters_.clustering_algorithm_ == "kmeans")
    {
        ROS_DEBUG_STREAM("clustering algorithm = kmeans");
        bow_trainer_ = new cv::BOWKMeansTrainer(parameters_.cluster_count_,
                                                cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,50,0.001),
                                                3,
                                                cv::KMEANS_PP_CENTERS);
    }



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


void FabmapTraining::trainOpen(const ros::TimerEvent &)
{

    ROS_DEBUG_STREAM("Start training BoW vocabulary and Chow Liu Tree ...");

    ROS_DEBUG_STREAM("Number of images: " << surf_descriptors_.size());

    bow_vocabulary_ = bow_trainer_->cluster();

    ROS_DEBUG_STREAM("Vocabulary succesfully trained");

    computeBoWDescriptors();

    ROS_DEBUG_STREAM("Start Chow Liu Training");

    cl_trainer_->add(bow_descriptors_);

    cl_tree_ = cl_trainer_->make();

    storeTrainingData();

    ROS_DEBUG_STREAM("Finish training");

    ros::requestShutdown();
}


void FabmapTraining::trainOriginal(const ros::TimerEvent &)
{

    ROS_DEBUG_STREAM("Start training BoW vocabulary...");

    ROS_DEBUG_STREAM("Number of images: " << surf_descriptors_.size());

    bow_vocabulary_ = bow_trainer_->cluster();

    ROS_DEBUG_STREAM("Vocabulary succesfully trained");

    computeBoWIntegerDescriptors();

    storeTrainingData();

    storeOXV();

    storeOXS();

    AcceleratedCLCalculator chowLiuCalculator(parameters_.dataset_path_,parameters_.dataset_name_,parameters_.dataset_path_,0.0);
    chowLiuCalculator.DoCalc();

    ROS_DEBUG_STREAM("Finish training");

    ros::requestShutdown();
}

void FabmapTraining::storeOXS()
{
    std::string filename = parameters_.dataset_path_  +  parameters_.dataset_name_ +  std::string(".oxs");


    std::ofstream file(filename.c_str());

    file << "VOCABULARY:" << parameters_.dataset_name_ + std::string(".oxv") << endl;
    file << "SCENES:" << bow_descriptors_.size()*parameters_.times_to_use_samples_ << endl;

    for(int times = 0; times < parameters_.times_to_use_samples_;times++)
    {
        for(int i=0;i<bow_descriptors_.size();i++)
        {
            file << "SCENE:" << endl << endl << endl;

            for(int j=0;j<bow_descriptors_[i].cols;j++)
            {
                if( bow_descriptors_[i].at<int>(0,j) != 0 )
                    file << j << " ";
            }
            file << endl;

            for(int j=0;j<bow_descriptors_[i].cols;j++)
            {
                if( bow_descriptors_[i].at<int>(0,j) != 0 )
                    file << bow_descriptors_[i].at<int>(0,j) << " ";
            }
            file << endl;
        }
    }

    file << "WORDS:" << bow_vocabulary_.rows << endl;

    file.close();

}

void FabmapTraining::storeOXV()
{
    std::string filename = parameters_.dataset_path_ + parameters_.dataset_name_ + std::string(".oxv");

    std::ofstream file(filename.c_str());

    file << "WORDS:" << bow_vocabulary_.rows << endl;
    file << "CLUSTER_THRESHOLD:" << parameters_.cluster_size_ << endl;

    for(int i=0;i<bow_vocabulary_.rows;i++)
    {
        file << "WORD:" << i << endl;
        for(int j=0;j<bow_vocabulary_.cols;j++)
        {
            file << bow_vocabulary_.at<float>(i,j) << " ";
        }
        file << endl;
    }

    file.close();
}

void FabmapTraining::computeBoWDescriptors()
{
    //! Compute BoW descriptors

    BOWImgDescriptorExtractor extractor(cv::DescriptorMatcher::create("FlannBased"));
    cv::Mat image_descriptor;

    extractor.setVocabulary(bow_vocabulary_);


    for(int i=0;i<surf_descriptors_.size();i++)
    {
        extractor.compute(surf_descriptors_[i],image_descriptor);

        bow_descriptors_.push_back(image_descriptor);
    }
}

void FabmapTraining::computeBoWIntegerDescriptors()
{
    //! Compute BoW descriptors

    BOWImgDescriptorExtractor extractor(cv::DescriptorMatcher::create("FlannBased"));
    cv::Mat image_descriptor;

    extractor.setVocabulary(bow_vocabulary_);


    for(int i=0;i<surf_descriptors_.size();i++)
    {
        extractor.computeBowInteger(surf_descriptors_[i],image_descriptor);

        bow_descriptors_.push_back(image_descriptor);
    }
}

void FabmapTraining::storeTrainingData()
{

    cv::FileStorage fs;

    fs.open(parameters_.fabmap_vocab_,cv::FileStorage::WRITE);
    fs << "vocabulary" << bow_vocabulary_;
    fs.release();

    fs.open(parameters_.fabmap_tree_,cv::FileStorage::WRITE);
    fs << "tree" << cl_tree_;
    fs.release();

    //! \todo Transformar em uma matriz primeiro, para depois salvar os valores
    fs.open(parameters_.fabmap_descriptors_,cv::FileStorage::WRITE);
    cv::Mat_ <float>descriptors;
    convert(bow_descriptors_,descriptors);
    fs << "descriptors" << descriptors;
    fs.release();


}

}
