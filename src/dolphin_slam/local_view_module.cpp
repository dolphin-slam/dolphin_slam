#include "local_view_module.h"

const float ROS_TIMER_STEP = 0.25;

using std::endl;

namespace dolphin_slam
{

LocalViewModule::LocalViewModule()
{

    number_of_created_local_views = 0;
    number_of_recognized_local_views = 0;

    start_stamp_ = ros::Time::now();



    loadParameters();

    createROSSubscribers();

    createROSPublishers();

    log_file_.open("local_view.log");

}

LocalViewModule::~LocalViewModule()
{
    log_file_.close();
}

void LocalViewModule::loadParameters()
{
    ros::NodeHandle private_nh("~");

    private_nh.param<double>("similarity_threshold",parameters_.similarity_threshold,0.85);

    private_nh.param<std::string>("local_view_activation",parameters_.local_view_activation_,"multiple");

}

void LocalViewModule::createROSSubscribers()
{
    image_histogram_subscriber_ = node_handle_.subscribe("/image_histogram",1,&LocalViewModule::callback,this);

}

void LocalViewModule::createROSPublishers()
{
    output_publisher_ = node_handle_.advertise<dolphin_slam::ActiveLocalViewCells>("local_view_cells",1);

    execution_time_publisher_ = node_handle_.advertise<dolphin_slam::ExecutionTime>("execution_time",1,false);
}




void LocalViewModule::createROSTimers()
{
    timer_ = node_handle_.createTimer(ros::Duration(0.5), &LocalViewModule::timerCallback,this);

}

void LocalViewModule::timerCallback(const ros::TimerEvent& event)
{

}

/*!
*   \brief Function to publish view template
*/
void LocalViewModule::publishOutput(){

    double max_rate = 0;
    int max_id;

    output_message_.header.stamp = ros::Time::now();

    if(parameters_.local_view_activation_ == "single")
    {
        BOOST_FOREACH(LocalViewCell &lvc,cells_)
        {
            if(lvc.active_)
            {
                if(lvc.rate_ > max_rate)
                {
                    max_rate = lvc.rate_;
                    max_id = lvc.id_;
                }
            }
        }
        output_message_.most_active_id_ = max_id;
        output_message_.cell_id_.push_back(max_id);
        output_message_.cell_rate_.push_back(max_rate);

        log_file_ << (ros::Time::now() - start_stamp_).toSec() << " " << max_id << " " << max_rate << endl;
    }
    else if(parameters_.local_view_activation_ == "multiple")
    {
        log_file_ << (ros::Time::now() - start_stamp_).toSec() << " ";
        BOOST_FOREACH(LocalViewCell &lvc,cells_)
        {
            if(lvc.active_)
            {
                if(lvc.rate_ > max_rate)
                {
                    max_rate = lvc.rate_;
                    max_id = lvc.id_;
                }
                output_message_.cell_id_.push_back(lvc.id_);
                output_message_.cell_rate_.push_back(lvc.rate_);
                log_file_ << lvc.id_ << " " << lvc.rate_ << " " ;
            }
        }
        log_file_ << endl;
        output_message_.most_active_id_ = max_id;

    }
    else
    {
        ROS_ERROR("Wrong local view cell activation. Use single or multiple");
    }

    //! Publica a mensagem
    output_publisher_.publish(output_message_);

}

/*!
*   \brief Callback function
*
*   Receive an image and the corresponding histogram.
*   Compute the local view cells activations.
*/
void LocalViewModule::callback(const dolphin_slam::ImageHistogramConstPtr &message)
{

    time_monitor_.start();

    output_message_.image_ = message->image;
    output_message_.histogram_ = message->histogram;

    //! convert to opencv matrix
    cv::Mat histogram(message->histogram);

    ROS_DEBUG_STREAM("histogram = "<< histogram);

    computeRate(histogram);

    publishOutput();

    time_monitor_.finish();
    time_monitor_.print();

    execution_time = time_monitor_.getDuration();

    publishExecutionTime();

}


void LocalViewModule::publishExecutionTime()
{
    ExecutionTime msg;
    msg.header.stamp = ros::Time::now();

    msg.module = "lv";
    msg.iteration_time = time_monitor_.getDuration();

    execution_time_publisher_.publish(msg);

}


int LocalViewModule::createNewCell(const cv::Mat & histogram)
{

    cells_.resize(cells_.size() + 1);
    LocalViewCell* cell = &(*(cells_.end() - 1));

    cell->id_ = cells_.size() - 1;

    cell->rate_ = 1.0;
    cell->active_ = true;
    cell->data_ = histogram.clone();

    ROS_DEBUG_STREAM("View Template created. ID = " << cell->id_);

    return cell->id_;
}



void LocalViewModule::computeRate(const cv::Mat & histogram)
{
    bool has_active_cell = false;

    foreach (LocalViewCell &cell, cells_){

        cell.rate_ = cv::compareHist(cell.data_,histogram,CV_COMP_CORREL);

        if(cell.rate_ > parameters_.similarity_threshold)
        {
            cell.active_ = true;
            has_active_cell = true;
        }
        else
        {
            cell.active_ = false;
        }
    }

    if(!has_active_cell)
        createNewCell(histogram);
}



} //namespace
