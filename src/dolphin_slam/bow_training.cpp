#include "bow_training.h"

#include "bow_descriptor_extractor.h"

BoWTraining::BoWTraining()
{

    loadParameters();

    createROSSubscribers();

    createROSTimers();

    init();

}

BoWTraining::~BoWTraining()
{
    std::ofstream descriptors_file("descriptors.txt");

    for(int i=0; i<surf_descriptors_.size();i++)
    {
        for(int j=0;j<surf_descriptors_[i].rows;j++)
        {
            for(int k=0;k<surf_descriptors_[i].cols;k++)
            {
                descriptors_file << surf_descriptors_[i].at<float>(j,k) << " ";
            }
            descriptors_file << std::endl;
        }
    }
    descriptors_file.close();

    std::ofstream vocab_file("vocabulary.txt");

    for(int j=0;j<bow_vocabulary_.rows;j++)
    {
        for(int k=0;k<bow_vocabulary_.cols;k++)
        {
            vocab_file << bow_vocabulary_.at<float>(j,k) << " ";
        }
        vocab_file << std::endl;
    }
    vocab_file.close();
}

void BoWTraining::loadParameters()
{
    ros::NodeHandle private_nh_("~");

    private_nh_.param<std::string>("descriptors_topic",parameters_.descriptors_topic_,"/descriptors");

    private_nh_.param<std::string>("bow_vocab",parameters_.bow_vocab_,"vocabulary.xml");

    private_nh_.param<int>("cluster_count",parameters_.cluster_count_,100);
}

void BoWTraining::createROSSubscribers()
{
    descriptors_subscriber_ = node_handle_.subscribe(parameters_.descriptors_topic_,1,&BoWTraining::descriptorsCallback,this);

}

void BoWTraining::createROSTimers()
{
    timeout_ = node_handle_.createTimer(ros::Duration(20),&BoWTraining::train,this,true,false);
}

void BoWTraining::init()
{

    bow_trainer_ = new cv::BOWKMeansTrainer(parameters_.cluster_count_,
                                            cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,50,0.001),
                                            5,
                                            cv::KMEANS_PP_CENTERS);

}

void BoWTraining::descriptorsCallback(const dolphin_slam::DescriptorsConstPtr &msg)
{
    cv::Mat_<float> descriptors(msg->descriptor_count_,msg->descriptor_length_);
    std::copy(msg->data_.begin(),msg->data_.end(),descriptors.begin());

    //! Stop the timeout
    timeout_.stop();

    if(descriptors.rows > 0)

    {
        surf_descriptors_.push_back(descriptors);

        bow_trainer_->add(descriptors);
    }

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

    fs.open(parameters_.bow_vocab_,cv::FileStorage::WRITE);
    fs << "vocabulary" << bow_vocabulary_;
    fs.release();



}
