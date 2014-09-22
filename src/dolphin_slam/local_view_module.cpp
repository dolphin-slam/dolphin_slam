#include "local_view_module.h"

const float ROS_TIMER_STEP = 0.25;

namespace dolphin_slam
{

LocalViewModule::LocalViewModule()
{

    number_of_created_local_views = 0;
    number_of_recognized_local_views = 0;

    loadParameters();

    createROSSubscribers();

    createROSPublishers();

}

LocalViewModule::~LocalViewModule()
{

}

void LocalViewModule::loadParameters()
{
    ros::NodeHandle private_nh("~");

    private_nh.param<double>("similarity_threshold",parameters_.similarity_threshold,0.85);

}

void LocalViewModule::createROSSubscribers()
{
    image_histogram_subscriber_ = node_handle_.subscribe("/image_histogram",1,&LocalViewModule::callback,this);

}

void LocalViewModule::createROSPublishers()
{
    view_template_publisher_ = node_handle_.advertise<dolphin_slam::LocalViewNetwork>("local_view_cells",1);

    execution_time_publisher_ = node_handle_.advertise<dolphin_slam::ExecutionTime>("execution_time",1,false);
}




void LocalViewModule::createROSTimers()
{
    timer_ = node_handle_.createTimer(ros::Duration(0.5), &LocalViewModule::timerCallback,this);

}

void LocalViewModule::timerCallback(const ros::TimerEvent& event)
{
    static int cont = 0;

    has_new_local_view_cell = true;

    most_active_cell_ = cont++;


    //current_view_template_ = cont++;


    publishViewTemplate();
}

/*!
*   \brief Function to publish view template
*/
void LocalViewModule::publishViewTemplate(){

    LocalViewNetwork message;
    Cell *cell;

    message.header.stamp = ros::Time::now();

    message.has_new_cell_ = has_new_local_view_cell;


    message.number_of_cells_ = cells_.size();
    message.most_active_cell_ = most_active_cell_;
    message.active_cells_.resize(active_cells_.size());

    for(int i=0;i<active_cells_.size();i++)
    {
        //! Assert para testar se o id está confizente com o indice do vetor
        cell = &cells_[active_cells_[i]];
        message.active_cells_[i].id_ = cell->id_;
        message.active_cells_[i].rate_ = cell->rate_;

    }

    //! Publica a mensagem
    view_template_publisher_.publish(message);

}

/*!
*   \brief Callback function
*
*   Receive an image and the corresponding histogram.
*   Compute the local view cells activations.
*/
void LocalViewModule::callback(const dolphin_slam::ImageHistogramConstPtr &message)
{
    static int count = 0;

    if(count == 0)
    {
        time_monitor_.start();

        cv::Mat histogram(message->histogram);

        if(computeLocalViewCellActivation(histogram))
        {
            number_of_created_local_views++;
            has_new_local_view_cell = true;
            ROS_DEBUG_STREAM("View Template created. ID = " << cells_.size()-1);
        }else
        {
            has_new_local_view_cell = false;
            ROS_DEBUG_STREAM("View Template recognized.");
        }

        publishViewTemplate();

        time_monitor_.finish();
        time_monitor_.print();

        execution_time = time_monitor_.getDuration();

        publishExecutionTime();

    }

    count = (count + 1)%parameters_.frames_to_jump_;
}




void LocalViewModule::publishExecutionTime()
{
    ExecutionTime msg;
    msg.header.stamp = ros::Time::now();

    msg.module = "lv";
    msg.iteration_time = time_monitor_.getDuration();

    execution_time_publisher_.publish(msg);

}


int LocalViewModule::createViewTemplate(const cv::Mat & histogram)
{

    cells_.resize(cells_.size() + 1);
    Cell* cell = &(*(cells_.end() - 1));

    cell->id_ = cells_.size() - 1;

    cell->rate_ = 1.0;
    cell->data_ = histogram.clone();

    return cell->id_;
}



bool LocalViewModule::computeLocalViewCellActivation(const cv::Mat & histogram)
{
    float correlation;

    float major_activation = 0;

    bool ret = false;

    active_cells_.clear();
    foreach (Cell &cell, cells_){

        correlation = cv::compareHist(cell.data_,histogram,CV_COMP_CORREL);

        if(correlation > parameters_.similarity_threshold)
        {
            //            cell.rate_ = (correlation - match_threshold_)/(1.0 - match_threshold_);

            cell.rate_ = correlation;

            active_cells_.push_back(cell.id_);
            ret = true;

            if (cell.rate_ > major_activation){
                major_activation = cell.rate_;
                most_active_cell_ = cell.id_;
            }
        }
        else
        {
            cell.rate_ = 0.0;
        }
    }

    //! Não encontrou nenhuma célula parecida
    if(!ret)
    {
        //! Cria uma nova local view
        most_active_cell_ = createViewTemplate(histogram);
        active_cells_.push_back(most_active_cell_);
    }

    return ret;

}

} //namespace
