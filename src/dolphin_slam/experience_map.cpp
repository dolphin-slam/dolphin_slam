#include "experience_map.h"

const bool ROTATE_MAPS = true;

namespace dolphin_slam
{

/*!
 * \brief Constructor
 */
ExperienceMap::ExperienceMap(): tf_listener_(tf_buffer_), image_buffer(100), it_(node_handle_)
{
    image_index_begin = 0;
    image_index_end = 0;

    max_id_experience_ = 0;
    number_of_recognized_experiences_ = 0;
    number_of_created_experiences_ = 0;
    test_number_ = 0;

    loadParameters();

    createROSPublishers();

    createROSSubscribers();

    sift_ = new cv::SIFT();//! arrumar os parametros do sift para um melhor aproveitamento

    std::string filename;
    filename = "experience_map_error_" + boost::lexical_cast<std::string>(test_number_) + ".txt";
    experience_map_error_file_.open(filename.c_str());

    filename = "dead_reckoning_error_" + boost::lexical_cast<std::string>(test_number_) + ".txt";
    dead_reckoning_error_file_.open(filename.c_str());

    filename = "experience_map_info_" + boost::lexical_cast<std::string>(test_number_) + ".txt";
    experience_map_file_.open(filename.c_str());
}

ExperienceMap::~ExperienceMap()
{
    experience_map_error_file_.close();
    dead_reckoning_error_file_.close();

    experience_map_file_.close();

}

void ExperienceMap::loadParameters()
{

    ros::NodeHandle private_nh_("~");

    private_nh_.param<double>("match_threshold",parameters_.match_threshold_,0.8);

    //! string image_topic;
    private_nh_.param<std::string>("image_topic",parameters_.image_topic_,"/image_raw");

    //! string image_transport;
    private_nh_.param<std::string>("image_transport",parameters_.image_transport_,"raw");

}

void ExperienceMap::createROSSubscribers()
{
    experience_event_subscriber_ = node_handle_.subscribe("experience_event",1000,&ExperienceMap::experienceEventCallback,this);

    //! hint to modify the image_transport. Here I use raw transport
    image_transport::TransportHints hints(parameters_.image_transport_,ros::TransportHints(),node_handle_);

    //! image subscription
    image_subscriber_ = it_.subscribe(parameters_.image_topic_,1,&ExperienceMap::imageCallback,this,hints);

}

void ExperienceMap::createROSPublishers()
{
    map_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("map",1,false);

    dead_reckoning_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("dead_reckoning",1,false);

    ground_truth_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("ground_truth",1,false);

    execution_time_publisher_ = node_handle_.advertise<dolphin_slam::ExecutionTime>("execution_time",1,false);

    error_publisher_ = node_handle_.advertise<dolphin_slam::Error>("error",1,false);
}

void ExperienceMap::imageCallback(const sensor_msgs::ImageConstPtr &image)
{
    static cv_bridge::CvImageConstPtr image_;

    //! convert to opencv image
    image_ = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::MONO8);

    //! assign new image to the buffer
    image_buffer[image_index_end] = std::make_pair(image_->image,image->header.seq);

    //! increase index to last image
    image_index_end = (image_index_end+1)%image_buffer.size();


    if(image_index_end == image_index_begin)
    {
        ROS_ERROR("Image buffer is full");
    }


}


void ExperienceMap::getGroundTruth(tf2::Transform & gt_pose, ros::Time stamp)
{
    geometry_msgs::TransformStamped transform;

    try{
        transform = tf_buffer_.lookupTransform("world","dolphin_slam/gt",stamp);
    }
    catch (tf2::LookupException e)
    {
        //! First time, there is no transform published yet
        transform = geometry_msgs::TransformStamped();
    }
    catch (tf2::ExtrapolationException e)
    {
        //! The time was not published. Get the last published time
        try{
            transform = tf_buffer_.lookupTransform("world","dolphin_slam/gt",ros::Time(0));
        }
        catch (tf2::TransformException e)
        {
            ROS_ERROR_STREAM("TF exception" << e.what());
        }
    }
    catch(tf2::TransformException e)
    {
        ROS_ERROR_STREAM("TF exception" << e.what());
    }

    gt_pose = getTransform(transform);
}

void ExperienceMap::getDeadReckoning(tf2::Transform & dr_pose, ros::Time stamp)
{
    geometry_msgs::TransformStamped transform;

    try{
        transform = tf_buffer_.lookupTransform("world","dolphin_slam/dr",stamp);
    }
    catch (tf2::LookupException e)
    {
        //! First time, there is no transform published yet
        transform = geometry_msgs::TransformStamped();
    }
    catch (tf2::ExtrapolationException e)
    {
        //! The time was not published. Get the last published time
        try{
            transform = tf_buffer_.lookupTransform("world","dolphin_slam/dr",ros::Time(0));
        }
        catch (tf2::TransformException e)
        {
            ROS_ERROR_STREAM("TF exception" << e.what());
        }
    }
    catch(tf2::TransformException e)
    {
        ROS_ERROR_STREAM("TF exception" << e.what());
    }

    dr_pose = getTransform(transform);
}

void ExperienceMap::createExperience(const ExperienceEventConstPtr &event)
{
    ExperienceDescriptor new_experience_descriptor;
    Experience * new_experience;
    Experience * current_experience;
    LinkDescriptor new_link_descriptor;
    tf2::Vector3 translation;

    //! add a new experience
    new_experience_descriptor = boost::add_vertex(map_);
    new_experience = &map_[new_experience_descriptor];

    //! set experience id
    new_experience->id_ = number_of_created_experiences_;

    //! copy active pc_index
    std::copy(event->pc_activity_.active_index_.begin(),event->pc_activity_.active_index_.end(),new_experience->pc_index_);

    //! set local view cell id
    new_experience->lv_cell_id_ = event->lv_cells_.most_active_cell_;

    //! set experience activation rate
    new_experience->rate_total_ = new_experience->rate_lv_ = new_experience->rate_pc_ = 1.0;

    //! set image
    bool image_found = false;
    for(int i=image_index_begin;i != image_index_end;i++)
    {
        //! look for same image seq
        if(image_buffer[i].second == event->lv_cells_.image_seq_)
        {
            image_found= true;
            new_experience->image_ = image_buffer[i].first;
            image_index_begin = i+1;
            break;
        }
    }

    if(!image_found)
        ROS_ERROR("Image not found on image_buffer");

    //! set ground truth
    getGroundTruth(new_experience->gt_pose_,event->lv_cells_.image_stamp_);

    //! set dead reckoning
    getDeadReckoning(new_experience->dr_pose_,event->lv_cells_.image_stamp_);


    if(number_of_created_experiences_ == 0)
    {
        //! set first pose at the same place as the ground truth
        new_experience->pose_ = new_experience->dr_pose_;
    }
    else
    {
        //! get a pointer to current experience
        current_experience = &map_[current_experience_descriptor_];

        //! Compute traveled distance since last experience
        translation = new_experience->dr_pose_.getOrigin() - current_experience->dr_pose_.getOrigin();

        //! compute new pose based on traveled distance and last pose
        new_experience->pose_.setOrigin(current_experience->pose_.getOrigin() + translation);
        new_experience->pose_.setRotation(new_experience->dr_pose_.getRotation());

        //! add a link between last and new experiences
        new_link_descriptor = boost::add_edge(current_experience_descriptor_,new_experience_descriptor,map_).first;

        //! set link transform
        map_[new_link_descriptor].translation_ = translation;

    }

    //! increase experience counter
    number_of_created_experiences_++;

    //! update current experience descriptor
    current_experience_descriptor_ = new_experience_descriptor;


}

/**
 * @brief Compute image transform
 *
 * @param current_image Current captured image
 * @param image Captured image on match experience
 * @return tf2::Transform relative transform to compute position of image in current_image frame
 */
tf2::Transform ExperienceMap::getImageTransform(cv::Mat &current_image,cv::Mat &image)
{
    tf2::Transform transform;

    //! Compute sift descriptors

    //! Compute descriptors matches

    //! Compute transform with RANSAC

    return transform;
}


//! compute matches between new experience(current_experience_descriptor) and all other experiences
void ExperienceMap::computeMatches()
{
    std::vector<ExperienceDescriptor> matches;
    LinkDescriptor link_descriptor;
    Link * link_ptr;
    double similarity;
    tf2::Vector3 translation;
    tf2::Transform image_transform;

    BOOST_FOREACH(ExperienceDescriptor exp, boost::vertices(map_)) {
        if(exp != current_experience_descriptor_)
        {
            similarity = map_[exp].computeSimilarity(map_[current_experience_descriptor_]);
            if(similarity != parameters_.match_threshold_)
            {
                matches.push_back(exp);
            }
        }
    }

    //! create links between current experience and similar ones
    BOOST_FOREACH(ExperienceDescriptor exp, matches) {
        //! \todo
        //! compute transform between images
        image_transform = getImageTransform(map_[current_experience_descriptor_].image_,map_[exp].image_);

        //! create links between experiences
        link_descriptor = boost::add_edge(current_experience_descriptor_,exp,map_).first;
        link_ptr = &map_[link_descriptor];

        link_ptr->translation_ = translation;
    }
}


/*!
 * \brief Callback Function to experience event.
 * \param message
 */
void ExperienceMap::experienceEventCallback(const ExperienceEventConstPtr &event)
{
    ROS_DEBUG_STREAM("Experience Event received");

    time_monitor_.start();

    createExperience(event);

    //computeMatches();

    //iterateMap();

    time_monitor_.finish();

//    calculeExperienceMapError();
//    calculeDeadReckoningError();
//    calculeLocalisationError();

    publishExperienceMap();
    publishDeadReckoning();
    publishGroundTruth();
//    publishError();
//    publishExecutionTime();


    experience_map_error_file_ << experience_map_error_ << " " <<
                                  experience_map_independent_error_.x << " " << experience_map_independent_error_.y << " " << experience_map_independent_error_.z << std::endl;

    dead_reckoning_error_file_ << dead_reckoning_error_ << " " <<
                                  dead_reckoning_independent_error_.x << " " << dead_reckoning_independent_error_.y << " " << dead_reckoning_independent_error_.z << std::endl;


    experience_map_file_ << number_of_created_experiences_ << " " <<  number_of_recognized_experiences_ << " " << time_monitor_.getDuration() << " " <<
                            experience_map_error_ << " " << dead_reckoning_error_ << " " << localisationErrorEM_ << " " << localisationErrorDR_ << std::endl;

}

void ExperienceMap::publishExecutionTime()
{
    ExecutionTime msg;
    msg.header.stamp = ros::Time::now();

    msg.module = "em";
    msg.iteration_time = time_monitor_.getDuration();

    ROS_DEBUG_STREAM("Entrou no publish time no em. time = " << time_monitor_.getDuration());

    execution_time_publisher_.publish(msg);
}

void ExperienceMap::calculeDeadReckoningError()
{

}

void ExperienceMap::calculeExperienceMapError()
{

}

void ExperienceMap::calculeLocalisationError()
{


}

void ExperienceMap::storeMaps()
{
    std::ofstream experience_map_file, dead_reckoning_map_file, experience_map_ground_truth_file, dead_reckoning_ground_truth_file;

    std::string filename;
    filename = "experience_map"+boost::lexical_cast<std::string>(test_number_)+".txt";
    experience_map_file.open(filename.c_str());

    filename = "experience_map_ground_truth"+boost::lexical_cast<std::string>(test_number_)+".txt";
    experience_map_ground_truth_file.open(filename.c_str());

    filename = "dead_reckoning_map"+boost::lexical_cast<std::string>(test_number_)+".txt";
    dead_reckoning_map_file.open(filename.c_str());

    filename = "dead_reckoning_ground_truth"+boost::lexical_cast<std::string>(test_number_)+".txt";
    dead_reckoning_ground_truth_file.open(filename.c_str());

    //! \todo store maps here

    experience_map_file.close();
    dead_reckoning_map_file.close();

}

/*!
 * \brief Publish dead reckoning
 */
void ExperienceMap::publishExperienceMap()
{
    visualization_msgs::Marker message;

    ExperienceDescriptor e1,e2;
    tf2::Vector3 point;

    //! completa o cabeçalho da mensagem
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "world";

    //! configura o tipo e a ação tomada pela mensagem
    message.type = visualization_msgs::Marker::LINE_LIST;
    message.action = visualization_msgs::Marker::ADD;
    message.ns = "experience_map";
    message.id = 0;

    //! configura a pose dos marcadores
    message.pose.position.x = 0.0 ;
    message.pose.position.y = 0.0;
    message.pose.position.z = 0.0;

    message.pose.orientation.x = 0.0;
    message.pose.orientation.y = 0.0;
    message.pose.orientation.z = 0.0;
    message.pose.orientation.w = 1.0;

    message.scale.x = 0.025;
    message.scale.y = 0.025;
    message.scale.z = 0.025;

    //! configura a cor dos marcadores
    message.color.r = 0.0;
    message.color.g = 0.0;
    message.color.b = 1.0;
    message.color.a = 1.0;

    //! configura os marcadores para serem permanentes
    message.lifetime = ros::Duration(0.0);

    message.points.resize(2*boost::num_edges(map_));
    int i = 0;
    foreach (LinkDescriptor l, boost::edges(map_)){
        e1 = boost::source(l, map_);
        e2 = boost::target(l, map_);
        std::cout << " ( " << map_[e1].id_ << " , " << map_[e2].id_ << " ) \t\t";

        point = map_[e1].pose_.getOrigin();

        std::cout << " ( " << point.x() << " , " << point.y() << " , " << point.z() << " ) \t\t";

        message.points[i].x = point.x();
        message.points[i].y = point.y();
        message.points[i].z = point.z();
        i++;
        point = map_[e2].pose_.getOrigin();

        std::cout << " ( " << point.x() << " , " << point.y() << " , " << point.z() << " )" << std::endl;

        message.points[i].x = point.x();
        message.points[i].y = point.y();
        message.points[i].z = point.z();
        i++;

    }

    map_publisher_.publish(message);

}


/*!
 * \brief Publish ground truth
 */
void ExperienceMap::publishGroundTruth()
{
    visualization_msgs::Marker message;

    ExperienceDescriptor e1,e2;
    tf2::Vector3 point;

    //! completa o cabeçalho da mensagem
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "world";

    //! configura o tipo e a ação tomada pela mensagem
    message.type = visualization_msgs::Marker::LINE_LIST;
    message.action = visualization_msgs::Marker::ADD;
    message.ns = "ground_truth";
    message.id = 0;

    //! configura a pose dos marcadores
    message.pose.position.x = 0.0 ;
    message.pose.position.y = 0.0;
    message.pose.position.z = 0.0;

    message.pose.orientation.x = 0.0;
    message.pose.orientation.y = 0.0;
    message.pose.orientation.z = 0.0;
    message.pose.orientation.w = 1.0;

    message.scale.x = 0.025;
    message.scale.y = 0.025;
    message.scale.z = 0.025;

    //! configura a cor dos marcadores
    message.color.r = 0.0;
    message.color.g = 1.0;
    message.color.b = 0.0;
    message.color.a = 1.0;

    //! configura os marcadores para serem permanentes
    message.lifetime = ros::Duration(0.0);

    message.points.resize(2*boost::num_edges(map_));
    int i = 0;
    foreach (LinkDescriptor l, boost::edges(map_)){
        e1 = boost::source(l, map_);
        point = map_[e1].gt_pose_.getOrigin();
        message.points[i].x = point.x();
        message.points[i].y = point.y();
        message.points[i].z = point.z();
        i++;
        e2 = boost::target(l, map_);
        point = map_[e2].gt_pose_.getOrigin();
        message.points[i].x = point.x();
        message.points[i].y = point.y();
        message.points[i].z = point.z();
        i++;
    }

    ground_truth_publisher_.publish(message);

}

/*!
 * \brief Publish dead reckoning
 */
void ExperienceMap::publishDeadReckoning()
{
    visualization_msgs::Marker message;

    ExperienceDescriptor e1,e2;
    tf2::Vector3 point;

    //! completa o cabeçalho da mensagem
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "world";

    //! configura o tipo e a ação tomada pela mensagem
    message.type = visualization_msgs::Marker::LINE_LIST;
    message.action = visualization_msgs::Marker::ADD;
    message.ns = "dead_reckoning";
    message.id = 0;

    //! configura a pose dos marcadores
    message.pose.position.x = 0.0 ;
    message.pose.position.y = 0.0;
    message.pose.position.z = 0.0;

    message.pose.orientation.x = 0.0;
    message.pose.orientation.y = 0.0;
    message.pose.orientation.z = 0.0;
    message.pose.orientation.w = 1.0;

    message.scale.x = 0.025;
    message.scale.y = 0.025;
    message.scale.z = 0.025;

    //! configura a cor dos marcadores
    message.color.r = 1.0;
    message.color.g = 0.0;
    message.color.b = 0.0;
    message.color.a = 1.0;

    //! configura os marcadores para serem permanentes
    message.lifetime = ros::Duration(0.0);

    message.points.resize(2*boost::num_edges(map_));
    int i = 0;
    foreach (LinkDescriptor l, boost::edges(map_)){
        e1 = boost::source(l, map_);
        point = map_[e1].dr_pose_.getOrigin();
        message.points[i].x = point.x();
        message.points[i].y = point.y();
        message.points[i].z = point.z();
        i++;
        e2 = boost::target(l, map_);
        point = map_[e2].dr_pose_.getOrigin();
        message.points[i].x = point.x();
        message.points[i].y = point.y();
        message.points[i].z = point.z();
        i++;
    }

    dead_reckoning_publisher_.publish(message);

}



void ExperienceMap::createROSMessageError(dolphin_slam::Error &message)
{
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "error";

    message.experience_map_error = localisationErrorEM_;
    message.dead_reckoning_error = localisationErrorDR_;

}

/*!
 * \brief Function to publish Tf experience map
 */
void ExperienceMap::publishTFPoses()
{
    geometry_msgs::TransformStamped msg;

    msg = createTransformStamped(map_[current_experience_descriptor_].pose_,ros::Time::now(),"world","dolphin_slam/em");
    tf_broadcaster_.sendTransform(msg);

    //    msg = createTransformStamped(map_[current_experience_descriptor_].gt_pose_,ros::Time::now(),"world","em_gt_pose");
    //    tf_broadcaster_.sendTransform(msg);

    //    msg = createTransformStamped(map_[current_experience_descriptor_].dr_pose_,ros::Time::now(),"world","em_dr_pose");
    //    tf_broadcaster_.sendTransform(msg);

}

/*!
 * \brief ExperienceMap::publishError
 */
void ExperienceMap::publishError()
{
    dolphin_slam::Error message;
    createROSMessageError(message);

    error_publisher_.publish(message);
}



/*!
 * \brief Iterate map to minimize errors after loop closure events
 */
void ExperienceMap::iterateMap()
{
    //    cv::Point3f difference_i,difference_o;
    //    int ni,no;
    //    ExperienceDescriptor adjacent_experience;
    //    cv::Point3f first_experience_position;

    //    for(int n=0;n<10;n++){
    //        //! Para todos os vertices do mapa de experiencias
    //        foreach(ExperienceDescriptor experience, boost::vertices(map_))
    //        {

    //            //            if(index[experience] == 0){
    //            //                continue;
    //            //            }

    //            ni = no = 0;
    //            difference_i = cv::Point3f(0,0,0);
    //            difference_o = cv::Point3f(0,0,0);

    //            //! Seleciona todas as arestas que chegam no vertice escolhido
    //            foreach (LinkDescriptor link, boost::in_edges(experience,map_))
    //            {
    //                adjacent_experience = boost::source(link,map_);


    //                //! Calcula o erro baseado nas experiencias que chegam ao vertice
    //                difference_i += map_[adjacent_experience].position_ + map_[link].delta_position_ - map_[experience].position_;
    //                ni ++;
    //            }

    //            if(ni)
    //            {
    //                difference_i *= (1.0/ni);
    //            }

    //            //! Seleciona todas as arestas que saem do vertice escolhido
    //            foreach (LinkDescriptor link, boost::out_edges(experience,map_))
    //            {
    //                adjacent_experience = boost::target(link,map_);

    //                //! Calcula o erro baseado nas experiencias que saem ao vertice
    //                difference_o += map_[adjacent_experience].position_ - (map_[experience].position_ + map_[link].delta_position_ );
    //                no ++;
    //            }

    //            if(no)
    //            {
    //                difference_o *= (1.0/no);
    //            }

    //            map_[experience].position_ += 0.5*(difference_i + difference_o);

    //            if(map_[experience].id_ == 0)
    //            {
    //                first_experience_position = map_[experience].position_;
    //            }
    //        }

    //    }

    //    ROS_DEBUG_STREAM_NAMED("em","First experience position = " << first_experience_position);

    //    foreach(ExperienceDescriptor experience, boost::vertices(map_))
    //    {
    //        map_[experience].position_ = map_[experience].position_ - first_experience_position;
    //    }

}


} //namespace
