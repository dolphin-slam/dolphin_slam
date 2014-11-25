#include "local_view_module.h"



const float ROS_TIMER_STEP = 0.25;

using std::endl;

namespace dolphin_slam
{

LocalViewModule::LocalViewModule()
{

    metrics_.creation_count_ = 0;
    metrics_.recognition_count_ = 0;

    start_stamp_ = ros::Time::now();

    loadParameters();

    createROSSubscribers();

    createROSPublishers();

    init();

    log_file_rate_.open("localviews_rate.log");
    log_file_metrics_.open("localviews_metrics.log");

}

LocalViewModule::~LocalViewModule()
{
    log_file_rate_.close();
    log_file_metrics_.close();
}

void LocalViewModule::loadParameters()
{
    ros::NodeHandle private_nh("~");

    private_nh.param<double>("similarity_threshold",parameters_.similarity_threshold_,0.85);

    private_nh.param<std::string>("matching_algorithm",parameters_.matching_algorithm_,"correlation");

    private_nh.param<std::string>("descriptors_topic",parameters_.descriptors_topic_,"/descriptors");

    private_nh.param<std::string>("bow_vocabulary_path",parameters_.bow_vocabulary_path_,"vocabulary.xml");

    private_nh.param<std::string>("cltree_path",parameters_.cltree_path_,"cltree.xml");

    private_nh.param<std::string>("bow_descriptors_path",parameters_.bow_descriptors_path_,"descriptors.xml");

}

void LocalViewModule::createROSSubscribers()
{
    descriptors_subscriber_ = node_handle_.subscribe(parameters_.descriptors_topic_,1,&LocalViewModule::descriptors_callback,this);
}

void LocalViewModule::createROSPublishers()
{
    active_cells_publisher_ = node_handle_.advertise<dolphin_slam::ActiveLocalViewCells>("local_view_cells",1);

    execution_time_publisher_ = node_handle_.advertise<dolphin_slam::ExecutionTime>("execution_time",1,false);
}


void LocalViewModule::init()
{
    cv::FileStorage fs;

    fs.open(parameters_.bow_vocabulary_path_,cv::FileStorage::READ);
    fs["vocabulary"] >> bow_vocabulary_;
    fs.release();

    fs.open(parameters_.cltree_path_,cv::FileStorage::READ);
    fs["tree"] >> cltree_;
    fs.release();

    //! \todo Transformar em uma matriz primeiro, para depois salvar os valores
    fs.open(parameters_.bow_descriptors_path_,cv::FileStorage::READ);
    fs["descriptors"] >> bow_training_descriptors_;
    fs.release();


    bow_extractor_ = new BOWImgDescriptorExtractor(cv::DescriptorMatcher::create("FlannBased"));
    bow_extractor_->setVocabulary(bow_vocabulary_);

    fabmap_ = new cv::of2::FabMap1(cltree_,0.39,0,cv::of2::FabMap::SAMPLED | cv::of2::FabMap::CHOW_LIU,bow_training_descriptors_.rows);

    fabmap_->addTraining(bow_training_descriptors_);

    //! Estudar se tem mais coisas a adicionar na inicialização
    //! colocar os parametros de entrada do fabmap como parametros do ros.


}


void LocalViewModule::createROSTimers()
{
    timer_ = node_handle_.createTimer(ros::Duration(0.5), &LocalViewModule::timerCallback,this);

}

void LocalViewModule::timerCallback(const ros::TimerEvent& event)
{

}



void LocalViewModule::descriptors_callback(const DescriptorsConstPtr &msg)
{

    ROS_DEBUG_STREAM("Descriptors received. seq = " << msg->image_seq_);

    time_monitor_.start();

    last_best_match_id_ = best_match_id_;

    image_seq_ = msg->image_seq_;
    image_stamp_ = msg->image_stamp_;

    // Copy descriptors into a cv::Mat
    cv::Mat_<float> descriptors(msg->descriptor_count_,msg->descriptor_length_);
    std::copy(msg->data_.begin(),msg->data_.end(),descriptors.begin());

    computeImgDescriptor(descriptors);

    ROS_DEBUG_STREAM("Descriptors received. seq = " << msg->image_seq_);

    computeMatches();

    time_monitor_.finish();

    publishActiveCells();

    //! Compute metrics
    if(new_place_)
    {
        metrics_.creation_count_++;
    }
    else
    {
        if(best_match_id_ != last_best_match_id_)
        {
            metrics_.recognition_count_++;
        }
    }
    metrics_.execution_time_ = time_monitor_.getDuration();

    writeLog();

    publishExecutionTime();

}


void  LocalViewModule::computeMatches()
{
    if (cells_.size() == 0)
    {
        new_place_ = true;
    }
    else
    {
        if(parameters_.matching_algorithm_ == "correlation")
        {
            computeCorrelations();
        }
        else if(parameters_.matching_algorithm_ == "fabmap")
        {
            computeFabmap();
        }
        else
        {
            ROS_ERROR_STREAM("Matching algorithm is wrong.");
            exit(0);
        }
    }

    if(new_place_)
    {
        createNewCell();
    }


}

void LocalViewModule::writeLog()
{
    double stamp = (ros::Time::now() - start_stamp_).toSec();
    log_file_rate_ << stamp << " ";

    std::vector<LocalViewCell>::iterator cell_iterator_;
    for(cell_iterator_ = cells_.begin();cell_iterator_!= cells_.end();cell_iterator_++)
    {
        log_file_rate_ << cell_iterator_->rate_ << " " ;
    }
    log_file_rate_ << std::endl;

    log_file_metrics_ << stamp << " "
                      << best_match_id_ << " "
                      << metrics_.execution_time_ << " "
                      << metrics_.creation_count_ << " "
                      << metrics_.recognition_count_ << " "
                      << std::endl;
}


void LocalViewModule::publishExecutionTime()
{
    ExecutionTime msg;
    msg.header.stamp = ros::Time::now();

    msg.module = "lv";
    msg.iteration_time = time_monitor_.getDuration();

    execution_time_publisher_.publish(msg);

}


void LocalViewModule::computeCorrelations()
{
    new_place_ = false;

    std::vector<LocalViewCell>::iterator cell_iterator_;
    std::vector<LocalViewCell>::iterator best_match;
    best_match = cells_.begin();
    for(cell_iterator_ = cells_.begin();cell_iterator_!= cells_.end();cell_iterator_++)
    {
        cell_iterator_->rate_ = cv::compareHist(bow_descriptors_[cell_iterator_->id_],bow_current_descriptor_,CV_COMP_CORREL);

        //! test activation against a similarity threshold;
        cell_iterator_->active_ = (cell_iterator_->rate_ > parameters_.similarity_threshold_);

        new_place_ = new_place_ || cell_iterator_->active_;

        //! compute best match
        if(best_match->rate_ < cell_iterator_->rate_)
        {
            best_match = cell_iterator_;
        }
    }


    if(!new_place_)
    {
        new_rate_ = 0;
        best_match_id_ = best_match->id_;
    }
    else
    {
        new_rate_ = 1;
    }

}

void LocalViewModule::computeFabmap()
{
    std::vector<cv::of2::IMatch> imatch;

    ROS_DEBUG_STREAM("Compute Fabmap");


    fabmap_->compare(bow_current_descriptor_,bow_descriptors_,imatch);

    //! Test for new places
    std::vector<cv::of2::IMatch>::iterator imatch_iterator, best_match;

    best_match = imatch.begin();

    for(imatch_iterator = imatch.begin();imatch_iterator != imatch.end();imatch_iterator++)
    {
        //! test if image index is valid index
        if(imatch_iterator->imgIdx != -1)
        {
            //! store the match probability on the cell structure
            cells_[imatch_iterator->imgIdx].rate_ = imatch_iterator->match;
            cells_[imatch_iterator->imgIdx].active_ = false;
        }
        else
        {
            new_rate_ = imatch_iterator->match;
        }

        //! compute best match
        if(best_match->match < imatch_iterator->match)
        {
            best_match = imatch_iterator;
        }

    }

    ROS_DEBUG_STREAM("Number of stored places: " << imatch.size()-1 << " best match = " << best_match->imgIdx);

    new_place_ = (best_match->imgIdx == -1);

    if(!new_place_)
    {
        best_match_id_ = best_match->imgIdx;
    }



}

void LocalViewModule::createNewCell()
{
    LocalViewCell new_cell;

    new_cell.id_ = cells_.size();
    new_cell.rate_ = new_rate_;
    new_cell.active_ = true;

    best_match_id_ = new_cell.id_;

    bow_descriptors_.push_back(bow_current_descriptor_);

    cells_.push_back(new_cell);

}


void LocalViewModule::computeImgDescriptor(cv::Mat & descriptors)
{
    bow_extractor_->compute(descriptors,bow_current_descriptor_);
}

void LocalViewModule::publishActiveCells(){

    ActiveLocalViewCells msg;

    msg.header.stamp = ros::Time::now();

    msg.image_seq_ = image_seq_;
    msg.image_stamp_ = image_stamp_;

    msg.most_active_cell_ = best_match_id_;

    if(parameters_.matching_algorithm_ == "fabmap")
    {
        msg.cell_id_.push_back(best_match_id_);
        msg.cell_rate_.push_back(cells_[best_match_id_].rate_);

    }
    else if(parameters_.matching_algorithm_ == "correlation")
    {
        log_file_rate_ << (ros::Time::now() - start_stamp_).toSec() << " ";

        std::vector<LocalViewCell>::iterator cell_iterator_;
        for(cell_iterator_ = cells_.begin();cell_iterator_!= cells_.end();cell_iterator_++)
        {
            if(cell_iterator_->active_)
            {
                msg.cell_id_.push_back(cell_iterator_->id_);
                msg.cell_rate_.push_back(cell_iterator_->rate_);
            }
        }
    }
    else
    {
        ROS_ERROR_STREAM("Wrong matching algorithm: " << parameters_.matching_algorithm_);
    }

    active_cells_publisher_.publish(msg);

}



} //namespace
