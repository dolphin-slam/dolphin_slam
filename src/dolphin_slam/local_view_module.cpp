#include "local_view_module.h"



const float ROS_TIMER_STEP = 0.25;

using std::endl;
using std::cout;

void PrintLocationProbabilityVector(LocationProbabilityContainer ComputedLocationProbability) {
    cout << "Number of locations in the map: "<< ComputedLocationProbability.size() << endl;
    cout << "Probability of each location: " << endl;
    for (unsigned int i = 0; i < ComputedLocationProbability.size();  i++) {
        cout << ComputedLocationProbability[i] << " ";
    }
    cout << endl;
}

namespace dolphin_slam
{

LocalViewModule::LocalViewModule()
{

    metrics_.creation_count_ = 0;
    metrics_.recognition_count_ = 0;

    loadParameters();

    createROSSubscribers();

    createROSPublishers();

    init();

    log_file_rate_.open("localviews_rate.log");
    log_file_metrics_.open("localviews_metrics.log");
    log_file_bow_.open("bow_descriptors.txt");

}

LocalViewModule::~LocalViewModule()
{
    log_file_rate_.close();
    log_file_metrics_.close();
    log_file_bow_.close();
}

void LocalViewModule::loadParameters()
{
    ros::NodeHandle private_nh("~");

    private_nh.param<double>("similarity_threshold",parameters_.similarity_threshold_,0.85);

    private_nh.param<std::string>("matching_algorithm",parameters_.matching_algorithm_,"correlation");

    private_nh.param<std::string>("descriptors_topic",parameters_.descriptors_topic_,"/descriptors");

    private_nh.param<std::string>("bow_vocab",parameters_.bow_vocab_,"vocabulary.xml");

    private_nh.param<std::string>("fabmap_vocab",parameters_.fabmap_vocab_,"vocabulary.xml");

    private_nh.param<std::string>("fabmap_tree",parameters_.fabmap_tree_,"cltree.xml");

    private_nh.param<std::string>("fabmap_descriptors",parameters_.fabmap_descriptors_,"descriptors.xml");

    private_nh.param<std::string>("fabmap_algorithm",parameters_.fabmap_algorithm_,"original");

    private_nh.param<std::string>("fabmap_config",parameters_.fabmap_config_,"fabmap.mooos");

    private_nh.param<bool>("fabmap_confirm_match",parameters_.fabmap_confirm_match_,true);




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

    if(parameters_.matching_algorithm_ == "fabmap")
    {
        if(parameters_.fabmap_algorithm_ == "open")
        {
            fs.open(parameters_.fabmap_vocab_,cv::FileStorage::READ);
            fs["vocabulary"] >> bow_vocabulary_;
            fs.release();

            fs.open(parameters_.fabmap_tree_,cv::FileStorage::READ);
            fs["tree"] >> cltree_;
            fs.release();

            ROS_DEBUG_STREAM("Fabmap algorithm = open = " << parameters_.fabmap_algorithm_);

            //! \todo Transformar em uma matriz primeiro, para depois salvar os valores
            fs.open(parameters_.fabmap_descriptors_,cv::FileStorage::READ);
            fs["descriptors"] >> bow_training_descriptors_;
            fs.release();


            bow_extractor_ = new BOWImgDescriptorExtractor(cv::DescriptorMatcher::create("FlannBased"));
            bow_extractor_->setVocabulary(bow_vocabulary_);

            fabmap_open_ = new cv::of2::FabMap1(cltree_,0.39,0,cv::of2::FabMap::SAMPLED | cv::of2::FabMap::CHOW_LIU,bow_training_descriptors_.rows);

            fabmap_open_->addTraining(bow_training_descriptors_);

        }
        else if (parameters_.fabmap_algorithm_ == "original")
        {
            //! Load vocabulary
            fs.open(parameters_.fabmap_vocab_,cv::FileStorage::READ);
            fs["vocabulary"] >> bow_vocabulary_;
            fs.release();

            bow_extractor_ = new BOWImgDescriptorExtractor(cv::DescriptorMatcher::create("FlannBased"));
            bow_extractor_->setVocabulary(bow_vocabulary_);

            ROS_DEBUG_STREAM("Fabmap algorithm = " << parameters_.fabmap_algorithm_);
            ROS_DEBUG_STREAM("Fabmap config = " << parameters_.fabmap_config_);


            string vocab_path, vocabName;
            double p_observe_given_exists, p_observe_given_not_exists, p_at_new_place, df_likelihood_smoothing_factor;
            unsigned int vocab_size;

            CProcessConfigReader FileReader;
            FileReader.SetFile(parameters_.fabmap_config_);

            if (!FileReader.IsOpen()) {
                ROS_ERROR_STREAM("Could not find config file: " << parameters_.fabmap_config_ );
            }


            FileReader.GetValue("VocabPath", vocab_path);
            FileReader.GetValue("VocabName", vocabName);
            FileReader.GetValue("P_OBSERVE_GIVEN_EXISTS", p_observe_given_exists);
            FileReader.GetValue("P_OBSERVE_GIVEN_NOT_EXISTS", p_observe_given_not_exists);
            FileReader.GetValue("LIKELIHOOD_SMOOTHING_FACTOR", df_likelihood_smoothing_factor);
            FileReader.GetValue("P_AT_NEW_PLACE", p_at_new_place);

            ParseOXV_PeekDimensions(vocab_path + vocabName + ".oxv",vocab_size);

            ROS_DEBUG_STREAM("Vocab = " << vocab_path + vocabName);

            fabmap_original_ = new FabMapCalculator(vocab_path, vocabName,
                                                    p_observe_given_exists, p_observe_given_not_exists,
                                                    p_at_new_place, vocab_size,df_likelihood_smoothing_factor);

            fabmap_original_->ConfigureForExternalCalls(parameters_.fabmap_config_);
        }
    }
    else if (parameters_.matching_algorithm_== "correlation")
    {
        fs.open(parameters_.bow_vocab_,cv::FileStorage::READ);
        fs["vocabulary"] >> bow_vocabulary_;
        fs.release();

        bow_extractor_ = new BOWImgDescriptorExtractor(cv::DescriptorMatcher::create("FlannBased"));
        bow_extractor_->setVocabulary(bow_vocabulary_);

    }
    else
    {
        ROS_ERROR("Invalid matching algorithm");
    }

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


    ROS_DEBUG_STREAM("Descriptors received. seq = " << msg->image_seq_ << " Number of descriptors = "  << msg->descriptor_count_);


    if(metrics_.creation_count_ == 0)
    {
        start_stamp_ = ros::Time::now();
    }

    time_monitor_.start();

    last_best_match_id_ = best_match_id_;

    image_seq_ = msg->image_seq_;
    image_stamp_ = msg->image_stamp_;

    // Copy descriptors into a cv::Mat
    cv::Mat_<float> descriptors(msg->descriptor_count_,msg->descriptor_length_);
    std::copy(msg->data_.begin(),msg->data_.end(),descriptors.begin());


    computeImgDescriptor(descriptors);

    cout << bow_current_descriptor_ << endl;

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

    log_file_bow_ << image_seq_ << " ";

    for(int i=0;i<bow_current_descriptor_.cols;i++)
    {
        if(parameters_.matching_algorithm_ == "fabmap" && parameters_.fabmap_algorithm_ == "original")
        {
            log_file_bow_ << bow_current_descriptor_.at<int>(0,i)<< " ";
        }
        else
        {
            log_file_bow_ << bow_current_descriptor_.at<float>(0,i)<< " ";
        }

    }
    log_file_bow_ << endl;

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
    new_place_ = true;

    std::vector<LocalViewCell>::iterator cell_iterator_;
    std::vector<LocalViewCell>::iterator best_match;
    best_match = cells_.begin();
    cout << "correlations ";
    for(cell_iterator_ = cells_.begin();cell_iterator_!= cells_.end();cell_iterator_++)
    {
        cell_iterator_->rate_ = cv::compareHist(bow_descriptors_[cell_iterator_->id_],bow_current_descriptor_,CV_COMP_CORREL);

        cout << cell_iterator_->rate_ << " " ;
        //! test activation against a similarity threshold;
        cell_iterator_->active_ = (cell_iterator_->rate_ > parameters_.similarity_threshold_);

        if(cell_iterator_->active_)
        {
            new_place_ = false;
        }

        //! compute best match
        if(best_match->rate_ < cell_iterator_->rate_)
        {
            best_match = cell_iterator_;
        }
    }

    cout << endl;

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


    if (parameters_.fabmap_algorithm_ == "open")
    {


        std::vector<cv::of2::IMatch> imatch;

        ROS_DEBUG_STREAM("Compute Fabmap");

        fabmap_open_->compare(bow_current_descriptor_,bow_descriptors_,imatch);

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
    else if (parameters_.fabmap_algorithm_ == "original")
    {
        Observation observation(bow_current_descriptor_.cols);
        std::copy(bow_current_descriptor_.begin<int>(),bow_current_descriptor_.end<int>(),observation.begin());

        double best_match = 0;

        fabmap_original_->ProcessObservation(observation,computed_location_probability);


        PrintLocationProbabilityVector(computed_location_probability);

        for(int i=0;i<computed_location_probability.size();i++)
        {
            if(i < computed_location_probability.size()-1) //new place probability
            {
                cells_[i].rate_ = computed_location_probability[i];
                cells_[i].active_ = false;

                if(computed_location_probability[i] > best_match)
                {
                    best_match_id_ = i;
                    best_match = computed_location_probability[i];
                }

            }
            else
            {
                new_rate_ = computed_location_probability[i];

                if(new_rate_ > best_match)
                {
                    best_match = new_rate_;
                    new_place_ = true;
                }
                else
                {
                    new_place_ = false;
                }

            }
        }


        if(parameters_.fabmap_confirm_match_)
        {
            fabmap_original_->ConfirmLastMatch();
        }





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
    if(parameters_.matching_algorithm_ == "fabmap")
    {

    if(parameters_.fabmap_algorithm_ == "open")
    {
        bow_extractor_->compute(descriptors,bow_current_descriptor_);
    }
    else if(parameters_.fabmap_algorithm_ == "original")
    {
        bow_extractor_->computeBowInteger(descriptors,bow_current_descriptor_);
    }
    }else if (parameters_.matching_algorithm_ == "correlation")
    {
        bow_extractor_->compute(descriptors,bow_current_descriptor_);
    }

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
