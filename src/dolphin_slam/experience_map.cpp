#include "experience_map.h"

const bool ROTATE_MAPS = true;

namespace dolphin_slam
{

/*!
 * \brief Constructor
 */
ExperienceMap::ExperienceMap(): tf_listener_(tf_buffer_), it_(node_handle_)
{
    image_index_begin_ = 0;
    image_index_end_ = 0;

    max_id_experience_ = 0;
    number_of_recognized_experiences_ = 0;
    number_of_created_experiences_ = 0;

    loadParameters();

    createROSPublishers();

    createROSSubscribers();

    createROSServices();

    sift_ = new cv::SIFT();//! arrumar os parametros do sift para um melhor aproveitamento

    std::string filename;
    filename = "experience_map_error.txt";
    experience_map_error_file_.open(filename.c_str());

    filename = "dead_reckoning_error.txt";
    dead_reckoning_error_file_.open(filename.c_str());

    filename = "experience_map_info.txt";
    experience_map_info_file_.open(filename.c_str());


    filename = "localization_error.txt";
    localization_error_file_.open(filename.c_str());
}

ExperienceMap::~ExperienceMap()
{
    experience_map_error_file_.close();
    dead_reckoning_error_file_.close();

    experience_map_info_file_.close();

    localization_error_file_.close();

    storeMaps();

}

void ExperienceMap::loadParameters()
{

    ros::NodeHandle private_nh_("~");

    private_nh_.param<double>("match_threshold",parameters_.match_threshold_,0.8);

    //! string image_topic;
    private_nh_.param<std::string>("image_topic",parameters_.image_topic_,"/image_raw");

    //! string image_transport;
    private_nh_.param<std::string>("image_transport",parameters_.image_transport_,"raw");

    //! double focal_length;
    private_nh_.param<double>("focal_length",parameters_.focal_length_,1500.1);

    private_nh_.param<double>("lv_factor",parameters_.lv_factor_,0.5);

    private_nh_.param<double>("pc_factor",parameters_.pc_factor_,0.5);

    private_nh_.param<int>("min_experience_age",parameters_.min_experience_age_,10);

}

void ExperienceMap::createROSSubscribers()
{
    experience_event_subscriber_ = node_handle_.subscribe("experience_event",1000,&ExperienceMap::experienceEventCallback,this);

}

void ExperienceMap::createROSPublishers()
{
    map_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("map",1,false);

    dead_reckoning_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("dead_reckoning",1,false);

    ground_truth_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("ground_truth",1,false);

    execution_time_publisher_ = node_handle_.advertise<dolphin_slam::ExecutionTime>("execution_time",1,false);

    error_publisher_ = node_handle_.advertise<dolphin_slam::Error>("error",1,false);

    image_publisher_ = it_.advertise("experience_image", 1);

}

void ExperienceMap::createROSServices()
{
    image_client_ = node_handle_.serviceClient<dolphin_slam::ImageRequest>("image_request");
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

void ExperienceMap::computeActivationRate(const ExperienceEventConstPtr &event)
{
    Experience * exp_ptr;

    int index_pc;
    int neurons0 =  event->pc_activity_.number_of_neurons_[0];
    int neurons1 =  event->pc_activity_.number_of_neurons_[1];

    BOOST_FOREACH(ExperienceDescriptor exp, boost::vertices(map_)) {
        exp_ptr = &map_[exp];

        index_pc = exp_ptr->pc_index_[0]*neurons0*neurons1 +
                exp_ptr->pc_index_[1]*neurons1 +
                exp_ptr->pc_index_[2];


        exp_ptr->rate_pc_ = event->pc_activity_.activity_[index_pc];

        if(exp_ptr->lv_cell_id_ == event->lv_cells_.most_active_cell_)
        {
            exp_ptr->rate_lv_ = 1;
        }
        else
        {
            exp_ptr->rate_lv_ = 0;
        }

        exp_ptr->rate_total_ = parameters_.lv_factor_*exp_ptr->rate_lv_ + parameters_.pc_factor_*exp_ptr->rate_pc_;
    }

}


void ExperienceMap::createExperience(const ExperienceEventConstPtr &event)
{
    ExperienceDescriptor new_experience_descriptor;
    Experience * new_experience;
    Experience * current_experience;
    LinkDescriptor new_link_descriptor;
    tf2::Vector3 translation;

    cv_bridge::CvImage cv_image;

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

    //getImage(new_experience->image_,event->lv_cells_.image_seq_);

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

    experience_route_.push_back(current_experience_descriptor_);

    cv_image.image = new_experience->image_;
    cv_image.header.stamp = ros::Time::now();
    cv_image.encoding = sensor_msgs::image_encodings::MONO8;
    image_publisher_.publish(cv_image.toImageMsg());

    ROS_DEBUG_STREAM("Experience created: lv_id =  " << new_experience->lv_cell_id_  << " pc_index = " <<  new_experience->pc_index_[0] << " " << new_experience->pc_index_[1] << " " << new_experience->pc_index_[2] );

}

/**
 * @brief Compute image transform
 *
 * @param current_image Current captured image
 * @param image Captured image on match experience
 * @return tf2::Transform relative transform to compute position of image in current_image frame
 */
tf2::Vector3 ExperienceMap::getImageTransform(cv::Mat &current_image,cv::Mat &image)
{
    tf2::Vector3 image_translation;

    //! Compute sift descriptors
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;

    sift_->detect(current_image, keypoints_1);
    sift_->detect(image, keypoints_2);

    sift_->compute(current_image, keypoints_1, descriptors_1);
    sift_->compute(image, keypoints_2, descriptors_2);

    //    //! Compute descriptors matches
    //    std::vector< cv::DMatch > matches;
    //    matcher_.match(descriptors_1, descriptors_2, matches);

    //    //! Receiving only good matches
    //    std::vector< cv::DMatch > good_matches;

    //    int min_dist = 100;
    //    for (int i = 0; i < matches.size(); ++i)
    //    {
    //        if (matches[i].distance < 2*min_dist)
    //        {
    //            good_matches.push_back(matches[i]);
    //        }
    //    }

    //    std::vector< cv::Point2f > scene_1;
    //    std::vector< cv::Point2f > scene_2;
    //    for( int i = 0; i < keypoints_1.size(); i++ )
    //    {
    //        //-- Get the keypoints from the good matches
    //        scene_1.push_back( keypoints_1[good_matches[i].queryIdx].pt );
    //        scene_2.push_back( keypoints_2[good_matches[i].trainIdx].pt );
    //    }


    //    //! Finding homography with RANSAC
    //    cv::Mat H = cv::findHomography( scene_1, scene_2, CV_RANSAC );

    //    //! Finding the image center
    //    int image_center_x = current_image.cols / 2;
    //    int image_center_y = current_image.rows / 2;

    //    //! Getting the center in the other image
    //    cv::Mat image_center(1, 3, CV_32F);
    //    image_center.at<float>(0, 0) = image_center_x;
    //    image_center.at<float>(0, 1) = image_center_y;
    //    image_center.at<float>(0, 2) = 1;

    //    //! Finding field of view angle
    //    float field_of_view_angle_x = 2*atan(0.5*current_image.cols / parameters_.focal_length_);
    //    float field_of_view_angle_y = 2*atan(0.5*current_image.rows / parameters_.focal_length_);

    //    //! \todo: get the height now
    //    float field_of_view_x = tan(field_of_view_angle_x) * current_image.cols;
    //    float field_of_view_y = tan(field_of_view_angle_y) * current_image.rows;

    //    //! Compute how much a pixel affect the image
    //    float pixel_value_x = field_of_view_x / current_image.cols;
    //    float pixel_value_y = field_of_view_y / current_image.rows;

    //    cv::Mat point = image_center * H;

    //    image_translation.setValue((point.at<float>(0,0) / point.at<float>(0,2)) * pixel_value_x,(point.at<float>(0,1)/point.at<float>(0,2))* pixel_value_y,0);

    return image_translation;
}

//! compute matches between new experience(current_experience_descriptor) and all other experiences
void ExperienceMap::computeMatches()
{
    std::vector<ExperienceDescriptor> matches;
    double similarity;
    int best_match = -1;
    double best_similarity = 0;

    BOOST_FOREACH(ExperienceDescriptor exp, boost::vertices(map_)) {

        if (map_[current_experience_descriptor_].id_  - map_[exp].id_ > parameters_.min_experience_age_) {
            similarity = map_[exp].rate_total_;

            if(similarity >= parameters_.match_threshold_)
            {
                matches.push_back(exp);

                if(similarity > best_similarity)
                {
                    best_match = matches.size()-1;
                    best_similarity = similarity;
                }
            }
        }
    }

    //! change experience position
    if(best_match != -1)
    {
        ROS_DEBUG_STREAM("Match found: " << map_[matches[best_match]].id_ << " " << map_[current_experience_descriptor_].id_);

        best_match_experience_descriptor_ = matches[best_match];

        current_error_ = map_[matches[best_match]].pose_.getOrigin() - map_[current_experience_descriptor_].pose_.getOrigin() ;
        //updateMap();

        updateMap2();

    }

    //! create links between current experience and similar ones
    //    BOOST_FOREACH(ExperienceDescriptor exp, matches) {

    //        ROS_DEBUG_STREAM("Experience match: " << map_[current_experience_descriptor_].id_ << " " << map_[exp].id_);

    //        //! \todo
    //        //! compute transform between images
    //        image_translation = getImageTransform(map_[current_experience_descriptor_].image_,map_[exp].image_);
    //        ROS_DEBUG_STREAM("image_transform = " << image_translation.x() << " " << image_translation.y() << " " << image_translation.z() );

    //        //! create links between experiences
    //        // link_descriptor = boost::add_edge(current_experience_descriptor_,exp,map_).first;
    //        //link_ptr = &map_[link_descriptor];

    //        //link_ptr->translation_ = translation;
    //    }
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

    computeActivationRate(event);

    computeMatches();


    time_monitor_.finish();

    //calculeExperienceMapError();
    //calculeDeadReckoningError();
    calculeLocalisationError();

    publishExperienceMap();
    publishDeadReckoning();
    publishGroundTruth();
    publishError();
    //    publishExecutionTime();


    experience_map_error_file_ << experience_map_error_ << " " <<
                                  experience_map_independent_error_.x << " " << experience_map_independent_error_.y << " " << experience_map_independent_error_.z << std::endl;

    dead_reckoning_error_file_ << dead_reckoning_error_ << " " <<
                                  dead_reckoning_independent_error_.x << " " << dead_reckoning_independent_error_.y << " " << dead_reckoning_independent_error_.z << std::endl;


    experience_map_info_file_ << number_of_created_experiences_ << " " <<  number_of_recognized_experiences_ << " " << time_monitor_.getDuration() << " " <<
                                 experience_map_error_ << " " << dead_reckoning_error_ << " " << localisation_error_em_ << " " << localisation_error_dr_ << std::endl;

    localization_error_file_ << localisation_error_em_ << " " << localisation_error_dr_ << std::endl;

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
    tf2::Vector3 point = map_[current_experience_descriptor_].dr_pose_.getOrigin();
    tf2::Vector3 point_gt = map_[current_experience_descriptor_].gt_pose_.getOrigin();

    dead_reckoning_independent_error_.x = point.x() - point_gt.x();
    dead_reckoning_independent_error_.y = point.y() - point_gt.y();
    dead_reckoning_independent_error_.z = point.z() - point_gt.z();
    dead_reckoning_error_ = sqrt(pow(dead_reckoning_independent_error_.x,2) + pow(dead_reckoning_independent_error_.y,2) + pow(dead_reckoning_independent_error_.z,2));





}

void ExperienceMap::calculeExperienceMapError()
{
    tf2::Vector3 point = map_[current_experience_descriptor_].pose_.getOrigin();
    tf2::Vector3 point_gt = map_[current_experience_descriptor_].gt_pose_.getOrigin();

    experience_map_independent_error_.x = point.x() - point_gt.x();
    experience_map_independent_error_.y = point.y() - point_gt.y();
    experience_map_independent_error_.z = point.z() - point_gt.z();
    experience_map_error_ = sqrt(pow(experience_map_independent_error_.x,2) + pow(experience_map_independent_error_.y, 2) + pow(experience_map_independent_error_.z, 2));

}

void ExperienceMap::calculeLocalisationError()
{
    tf2::Vector3 position_dr_ = map_[current_experience_descriptor_].dr_pose_.getOrigin();
    tf2::Vector3 position_gt_ = map_[current_experience_descriptor_].gt_pose_.getOrigin();
    tf2::Vector3 position_em_ = map_[current_experience_descriptor_].pose_.getOrigin();


    localisation_error_em_ = position_em_.distance(position_gt_);
    localisation_error_dr_ = position_dr_.distance(position_gt_);

}

void ExperienceMap::storeMaps()
{
    std::ofstream experience_map_file("experience_map.txt");
    std::ofstream dead_reckoning_file("dead_reckoning.txt");
    std::ofstream ground_truth_file("ground_truth.txt");
    tf2::Vector3 position;

    //! \todo store maps here
    BOOST_FOREACH(ExperienceDescriptor exp, boost::vertices(map_))
    {
        position = map_[exp].pose_.getOrigin();
        experience_map_file << position.x() << " " << position.y() << " " << position.z() << std::endl;
        position = map_[exp].dr_pose_.getOrigin();
        dead_reckoning_file << position.x() << " " << position.y() << " " << position.z() << std::endl;
        position = map_[exp].gt_pose_.getOrigin();
        ground_truth_file << position.x() << " " << position.y() << " " << position.z() << std::endl;

    }

    experience_map_file.close();
    dead_reckoning_file.close();
    ground_truth_file.close();
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
        //std::cout << " ( " << map_[e1].id_ << " , " << map_[e2].id_ << " ) \t\t";

        point = map_[e1].pose_.getOrigin();

        //std::cout << " ( " << point.x() << " , " << point.y() << " , " << point.z() << " ) \t\t";

        message.points[i].x = point.x();
        message.points[i].y = point.y();
        message.points[i].z = point.z();
        i++;
        point = map_[e2].pose_.getOrigin();

        //std::cout << " ( " << point.x() << " , " << point.y() << " , " << point.z() << " )" << std::endl;

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

    message.localization_error_em_ = localisation_error_em_;
    message.localization_error_dr_ = localisation_error_dr_;

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

void ExperienceMap::getImage(cv::Mat &image, int seq)
{
    dolphin_slam::ImageRequest srv;
    cv_bridge::CvImagePtr cv_image;

    srv.request.seq = seq;

    if (image_client_.call(srv))
    {
        ROS_DEBUG_STREAM("ImageRequest call succesfully. image_seq = " << seq);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to call service image_request. image_seq = " << seq);
        return;
    }

    cv_image = cv_bridge::toCvCopy(srv.response.image,sensor_msgs::image_encodings::MONO8);

    image = cv_image->image;

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


void ExperienceMap::updateMap2()
{

    LinkDescriptor route;


    experience_route_.clear();

    experience_route_.push_back(best_match_experience_descriptor_);

    ExperienceDescriptor exp_aux = best_match_experience_descriptor_;

    while(exp_aux != current_experience_descriptor_)
    {

        exp_aux = boost::target(*boost::out_edges(exp_aux,map_).first,map_);
        experience_route_.push_back(exp_aux);
    }


    ROS_DEBUG_STREAM("Map update:  current_error_ = " << current_error_.length());

    tf2::Vector3 delta_error = current_error_/(experience_route_.size()-1);

    for(int i=0;i<experience_route_.size()-1;i++)
    {

        BOOST_FOREACH(LinkDescriptor link, boost::out_edges(experience_route_[i],map_))
        {
            if(experience_route_[i+1] == boost::target(link,map_))
            {
                route = link;
                break;
            }
        }

        map_[route].translation_ += delta_error;

        map_[experience_route_[i+1]].pose_.setOrigin(map_[experience_route_[i]].pose_.getOrigin() + map_[route].translation_);

    }


    experience_route_.clear();
    experience_route_.push_back(current_experience_descriptor_);

}

/*!
 * \brief Iterate map to minimize errors after loop closure events
 */
void ExperienceMap::updateMap()
{

    LinkDescriptor route;

    ROS_DEBUG_STREAM("Map update:  current_error_ = " << current_error_.length());

    tf2::Vector3 delta_error = current_error_/(experience_route_.size()-1);

    for(int i=0;i<experience_route_.size()-1;i++)
    {

        BOOST_FOREACH(LinkDescriptor link, boost::out_edges(experience_route_[i],map_))
        {
            if(experience_route_[i+1] == boost::target(link,map_))
            {
                route = link;
                break;
            }
        }

        map_[route].translation_ += delta_error;

        map_[experience_route_[i+1]].pose_.setOrigin(map_[experience_route_[i]].pose_.getOrigin() + map_[route].translation_);

    }


    experience_route_.clear();
    experience_route_.push_back(current_experience_descriptor_);




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
