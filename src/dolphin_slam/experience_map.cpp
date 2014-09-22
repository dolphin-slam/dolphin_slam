#include "experience_map.h"

const bool ROTATE_MAPS = true;

namespace dolphin_slam
{

/*!
 * \brief Constructor
 */
ExperienceMap::ExperienceMap()
{
    max_id_experience_ = 0;
    number_of_recognized_experiences_ = 0;
    number_of_created_experiences_ = 0;
    test_number_ = 0;

    loadParameters();

    createROSPublishers();

    createROSSubscribers();

    std::string filename;
    filename = "experience_map_error_" + boost::lexical_cast<std::string>(test_number_) + ".txt";
    experience_map_error_file_.open(filename.c_str());

    filename = "dead_reckoning_error_" + boost::lexical_cast<std::string>(test_number_) + ".txt";
    dead_reckoning_error_file_.open(filename.c_str());

    filename = "experience_map_info_" + boost::lexical_cast<std::string>(test_number_) + ".txt";
    experience_map_file_.open(filename.c_str());
}

void ExperienceMap::loadParameters()
{
    double match_threshold;
    node_handle_.param<double>("match_threshold",match_threshold,0.8);





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
}

void ExperienceMap::createFirstExperience(const ExperienceEventConstPtr &event)
{
    Experience * experience_ptr;

    //! adiciona a nova experiencia
    current_experience_descriptor_ = boost::add_vertex(map_);
    experience_ptr = &map_[current_experience_descriptor_];

    //! Atribui id, posição e ground truth
    experience_ptr->id_ = 0;
    experience_ptr->position_ = cv::Point3f(0.0,0.0,0.0);
    experience_ptr->ground_truth_= cv::Point3f(event->ground_truth_.x,event->ground_truth_.y,event->ground_truth_.z);

    //! Atribui o indice ativo da rede neural
    std::copy(event->pc_activity_.active_neuron_.begin(),event->pc_activity_.active_neuron_.end(),experience_ptr->active_neuron_);

    //! Atribui as local view ativas
    experience_ptr->lv_cells_.resize(event->lv_cells_active_.size());
    for(int i=0;i<event->lv_cells_active_.size();i++)
    {
        experience_ptr->lv_cells_[i].id_ = event->lv_cells_active_[i].id_;
        experience_ptr->lv_cells_[i].rate_ = event->lv_cells_active_[i].rate_;
    }
    experience_ptr->most_active_lv_cell_ = event->most_active_lv_cell_;

    number_of_created_experiences_ = 1;
}

void ExperienceMap::createFirstDeadReckoning(const ExperienceEventConstPtr &event)
{
    Experience * dead_reckoning_ptr;

    //! adiciona a nova experiencia
    current_dead_reckoning_descriptor_ = boost::add_vertex(dead_reckoning_map_);
    dead_reckoning_ptr = &dead_reckoning_map_[current_dead_reckoning_descriptor_];

    //! Atribui posição e ground truth
    dead_reckoning_ptr->position_ = cv::Point3f(0.0,0.0,0.0);
    dead_reckoning_ptr->ground_truth_= cv::Point3f(event->ground_truth_.x,event->ground_truth_.y,event->ground_truth_.z);
}

void ExperienceMap::createDeadReckoning(const ExperienceEventConstPtr &event)
{
    ExperienceDescriptor new_dead_reckoning_descriptor;
    Experience * dead_reckoning_ptr;
    LinkDescriptor link_descriptor;
    Link * link_ptr;
    cv::Point3f traveled_distance;

    //! adiciona a nova experiencia
    new_dead_reckoning_descriptor = boost::add_vertex(dead_reckoning_map_);
    dead_reckoning_ptr = &dead_reckoning_map_[new_dead_reckoning_descriptor];

    //! Atribui posição e ground truth
    traveled_distance = cv::Point3f(event->traveled_distance_.x,event->traveled_distance_.y,event->traveled_distance_.z);
    dead_reckoning_ptr->position_ = dead_reckoning_map_[current_dead_reckoning_descriptor_].position_ + traveled_distance;
    dead_reckoning_ptr->ground_truth_ = cv::Point3f(event->ground_truth_.x,event->ground_truth_.y,event->ground_truth_.z);

    //! Cria um link entre a experiência atual e a nova experiência
    link_descriptor = boost::add_edge(current_dead_reckoning_descriptor_,new_dead_reckoning_descriptor,dead_reckoning_map_).first;
    link_ptr = &dead_reckoning_map_[link_descriptor];
    link_ptr->delta_position_ = traveled_distance;

    //! Atualiza a experiência atual
    current_dead_reckoning_descriptor_ = new_dead_reckoning_descriptor;
}


ExperienceMap::~ExperienceMap()
{
    experience_map_error_file_.close();
    dead_reckoning_error_file_.close();

    experience_map_file_.close();

}




void ExperienceMap::createNewExperience(const ExperienceEventConstPtr &event)
{
    ExperienceDescriptor new_experience_descriptor;
    Experience * experience_ptr;
    LinkDescriptor link_descriptor;
    Link * link_ptr;
    cv::Point3f traveled_distance;

    //! adiciona a nova experiencia
    new_experience_descriptor = boost::add_vertex(map_);
    experience_ptr = &map_[new_experience_descriptor];

    //! Atribui id, posição e ground truth
    experience_ptr->id_ = boost::num_vertices(map_) - 1;

    traveled_distance = cv::Point3f(event->traveled_distance_.x,event->traveled_distance_.y,event->traveled_distance_.z);
    experience_ptr->position_ = map_[current_experience_descriptor_].position_ + traveled_distance;
    experience_ptr->ground_truth_ = cv::Point3f(event->ground_truth_.x,event->ground_truth_.y,event->ground_truth_.z);

    //! Atribui o indice ativo da rede neural
    std::copy(event->pc_activity_.active_neuron_.begin(),event->pc_activity_.active_neuron_.end(),experience_ptr->active_neuron_);

    //! Atribui as local view ativas
    experience_ptr->lv_cells_.resize(event->lv_cells_active_.size());
    for(int i=0;i<event->lv_cells_active_.size();i++)
    {
        experience_ptr->lv_cells_[i].id_ = event->lv_cells_active_[i].id_;
        experience_ptr->lv_cells_[i].rate_ = event->lv_cells_active_[i].rate_;
    }
    experience_ptr->most_active_lv_cell_ = event->most_active_lv_cell_;

    number_of_created_experiences_++;

    //! Cria um link entre a experiência atual e a nova experiência
    link_descriptor = boost::add_edge(current_experience_descriptor_,new_experience_descriptor,map_).first;
    link_ptr = &map_[link_descriptor];
    link_ptr->delta_position_ = traveled_distance;

    //! Atualiza a experiência atual
    current_experience_descriptor_ = new_experience_descriptor;
}

//! linka a experiência similar
void ExperienceMap::linkSimilarExperience(const ExperienceEventConstPtr &event,ExperienceDescriptor &similar_experience)
{
    LinkDescriptor link_descriptor;
    Link * link_ptr;
    cv::Point3f traveled_distance;

    //! \todo Analisar a adaptação da atividade neuronal


    //! Cria um link entre a experiência atual e a nova experiência
    link_descriptor = boost::add_edge(current_experience_descriptor_,similar_experience,map_).first;
    link_ptr = &map_[link_descriptor];
    traveled_distance = cv::Point3f(event->traveled_distance_.x,event->traveled_distance_.y,event->traveled_distance_.z);
    link_ptr->delta_position_ = traveled_distance;

    //! \todo Analizar a existência de falsas correspondencias, isto é, erro no fechamento de loop, comparando o ground truth das duas experiências

    //! Atualiza a experiência atual
    current_experience_descriptor_ = similar_experience;


}



bool ExperienceMap::lookForMatches(const ExperienceEventConstPtr &event, ExperienceDescriptor &similar_experience)
{
    float greatest_activity = 0;
    bool match_found = false;

    //! iterar por todas as experiências em busca de um match
    foreach (ExperienceDescriptor exp, boost::vertices(map_)) {
        map_[exp].computeActivity(event);

        if(map_[exp].rate_total_ > parameters_.match_threshold_)
        {
            match_found = true;
            if(map_[exp].rate_total_ > greatest_activity)
            {
                greatest_activity = map_[exp].rate_total_;
                similar_experience = exp;
            }
        }

    }

    return match_found;
}


/*!
 * \brief Callback Function to experience event.
 * \param message
 */
void ExperienceMap::experienceEventCallback(const ExperienceEventConstPtr &event)
{
    ExperienceDescriptor similar_experience;

    time_monitor_.start();

    //! \todo Copiar toda a atividade da pose cell e as local view ativas, para fins de comparação

    if(number_of_created_experiences_ == 0)
    {
        createFirstExperience(event);
        createFirstDeadReckoning(event);
    }
    else
    {
//        if(lookForMatches(event,similar_experience))
//        {
//            number_of_recognized_experiences_++;
//            ROS_DEBUG_STREAM_NAMED("em","Encontrou experiencia similar: ID atual = " << map_[current_experience_descriptor_].id_ <<
//                                   "ID match = " << map_[similar_experience].id_);
//            linkSimilarExperience(event,similar_experience);
//        }
//        else
        {
            number_of_created_experiences_++;
            createNewExperience(event);
            ROS_DEBUG_STREAM_NAMED("em","New experience ID = " << map_[current_experience_descriptor_].id_ );
        }
    }

    createDeadReckoning(event);

    iterateMap();

    time_monitor_.finish();

    calculeExperienceMapError();
    calculeDeadReckoningError();
    calculeLocalisationError();

    publishExperienceMap();
    publishDeadReckoning();
    publishGroundTruth();
    publishError();
    publishExecutionTime();


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
    float x_err=0, y_err=0, z_err=0,error=0;
    foreach (ExperienceDescriptor e, boost::vertices(dead_reckoning_map_)) {
        x_err +=  pow(dead_reckoning_map_[e].ground_truth_.x - dead_reckoning_map_[e].position_.x,2);
        y_err +=  pow(dead_reckoning_map_[e].ground_truth_.y - dead_reckoning_map_[e].position_.y,2);
        z_err +=  pow(dead_reckoning_map_[e].ground_truth_.z - dead_reckoning_map_[e].position_.z,2);
    }

    x_err /= boost::num_vertices(dead_reckoning_map_);
    y_err /= boost::num_vertices(dead_reckoning_map_);
    z_err /= boost::num_vertices(dead_reckoning_map_);

    foreach (ExperienceDescriptor e, boost::vertices(dead_reckoning_map_)) {
        error +=  pow(dead_reckoning_map_[e].ground_truth_.x - dead_reckoning_map_[e].position_.x,2) +
                pow(dead_reckoning_map_[e].ground_truth_.y - dead_reckoning_map_[e].position_.y,2) +
                pow(dead_reckoning_map_[e].ground_truth_.z - dead_reckoning_map_[e].position_.z,2);
    }

    dead_reckoning_error_ = error/boost::num_vertices(dead_reckoning_map_);
    dead_reckoning_independent_error_= cv::Point3f(x_err,y_err,z_err);

}

void ExperienceMap::calculeExperienceMapError()
{
    float x_err=0, y_err=0, z_err=0,error=0;
    foreach (ExperienceDescriptor e, boost::vertices(map_)) {
        x_err +=  pow(map_[e].ground_truth_.x - map_[e].position_.x,2);
        y_err +=  pow(map_[e].ground_truth_.y - map_[e].position_.y,2);
        z_err +=  pow(map_[e].ground_truth_.z - map_[e].position_.z,2);
    }

    x_err /= boost::num_vertices(map_);
    y_err /= boost::num_vertices(map_);
    z_err /= boost::num_vertices(map_);

    foreach (ExperienceDescriptor e, boost::vertices(map_)) {
        error +=  pow(map_[e].ground_truth_.x - map_[e].position_.x,2) +
                pow(map_[e].ground_truth_.y - map_[e].position_.y,2) +
                pow(map_[e].ground_truth_.z - map_[e].position_.z,2);
    }

    experience_map_error_ = error/boost::num_vertices(map_);
    experience_map_independent_error_ = cv::Point3f(x_err,y_err,z_err);

}

void ExperienceMap::calculeLocalisationError()
{

    localisationErrorEM_  =  sqrt(pow(map_[current_experience_descriptor_].ground_truth_.x - map_[current_experience_descriptor_].position_.x,2) +
                                  pow(map_[current_experience_descriptor_].ground_truth_.y - map_[current_experience_descriptor_].position_.y,2) +
                                  pow(map_[current_experience_descriptor_].ground_truth_.z - map_[current_experience_descriptor_].position_.z,2));


    localisationErrorDR_  =  sqrt(pow(dead_reckoning_map_[current_dead_reckoning_descriptor_].ground_truth_.x - dead_reckoning_map_[current_dead_reckoning_descriptor_].position_.x,2) +
                                  pow(dead_reckoning_map_[current_dead_reckoning_descriptor_].ground_truth_.y - dead_reckoning_map_[current_dead_reckoning_descriptor_].position_.y,2) +
                                  pow(dead_reckoning_map_[current_dead_reckoning_descriptor_].ground_truth_.z - dead_reckoning_map_[current_dead_reckoning_descriptor_].position_.z,2));

}

void ExperienceMap::storeMaps()
{
    std::ofstream experience_map_file, dead_reckoning_map_file, experience_map_ground_truth_file, dead_reckoning_ground_truth_file;

    ExperienceDescriptor exp1,exp2;

    //! \todo Abrir e fechar os arquivos aqui
    boost::graph_traits<Map>::edge_iterator ei, ei_end;

    std::string filename;
    filename = "experience_map"+boost::lexical_cast<std::string>(test_number_)+".txt";
    experience_map_file.open(filename.c_str());

    filename = "experience_map_ground_truth"+boost::lexical_cast<std::string>(test_number_)+".txt";
    experience_map_ground_truth_file.open(filename.c_str());

    filename = "dead_reckoning_map"+boost::lexical_cast<std::string>(test_number_)+".txt";
    dead_reckoning_map_file.open(filename.c_str());

    filename = "dead_reckoning_ground_truth"+boost::lexical_cast<std::string>(test_number_)+".txt";
    dead_reckoning_ground_truth_file.open(filename.c_str());

    bool first = true;
    for (boost::tie(ei, ei_end) = boost::edges(map_); ei != ei_end; ++ei)
    {
        if(first)
        {
            first = false;
            exp1 = boost::source(*ei, map_);
            experience_map_file << map_[exp1].position_.x << " " << map_[exp1].position_.y << " " << map_[exp1].position_.z << std::endl;
            experience_map_ground_truth_file << map_[exp1].ground_truth_.x << " " << map_[exp1].ground_truth_.y << " " << map_[exp1].ground_truth_.z << std::endl;
        }

        exp2 = boost::target(*ei, map_);
        experience_map_file << map_[exp2].position_.x << " " << map_[exp2].position_.y << " " << map_[exp2].position_.z << std::endl;
        experience_map_ground_truth_file << map_[exp2].ground_truth_.x << " " << map_[exp2].ground_truth_.y << " " << map_[exp2].ground_truth_.z << std::endl;

    }


    first = true;
    for (boost::tie(ei, ei_end) = boost::edges(dead_reckoning_map_); ei != ei_end; ++ei)
    {
        if(first)
        {
            first = false;
            exp1 = boost::source(*ei, dead_reckoning_map_);
            dead_reckoning_map_file << dead_reckoning_map_[exp1].position_.x << " " << dead_reckoning_map_[exp1].position_.y << " " << dead_reckoning_map_[exp1].position_.z << std::endl;
            dead_reckoning_ground_truth_file << dead_reckoning_map_[exp1].ground_truth_.x << " " << dead_reckoning_map_[exp1].ground_truth_.y << " " << dead_reckoning_map_[exp1].ground_truth_.z << std::endl;
        }

        exp2 = boost::target(*ei, dead_reckoning_map_);
        dead_reckoning_map_file << dead_reckoning_map_[exp2].position_.x << " " << dead_reckoning_map_[exp2].position_.y << " " << dead_reckoning_map_[exp2].position_.z << std::endl;
        dead_reckoning_ground_truth_file << dead_reckoning_map_[exp2].ground_truth_.x << " " << dead_reckoning_map_[exp2].ground_truth_.y << " " << dead_reckoning_map_[exp2].ground_truth_.z << std::endl;
    }

    experience_map_file.close();
    dead_reckoning_map_file.close();


}




/*!
 * \brief Experience::createROSMessageMap
 */
void ExperienceMap::createROSMessageGroundTruth(visualization_msgs::Marker & message)
{
    //! completa o cabeçalho da mensagem
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "world";

    //! configura o tipo e a ação tomada pela mensagem
    message.type = visualization_msgs::Marker::LINE_STRIP;
    //    message.type = visualization_msgs::Marker::POINTS;
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

    message.scale.x = 0.1;
    message.scale.y = 0.1;
    message.scale.z = 0.1;

    //! configura a cor dos marcadores
    message.color.r = 0.0;
    message.color.g = 1.0;
    message.color.b = 0.0;
    message.color.a = 1.0;

    //! configura os marcadores para serem permanentes
    message.lifetime = ros::Duration(0.0);

    message.points.resize(boost::num_vertices(map_));
    int i = 0;
    foreach (ExperienceDescriptor e, boost::vertices(map_)) {
        if ( ROTATE_MAPS){
            message.points[i].x = map_[e].ground_truth_.x;
            message.points[i].y = -map_[e].ground_truth_.y;
            message.points[i].z = -map_[e].ground_truth_.z;
        }else{
            message.points[i].x = map_[e].ground_truth_.x;
            message.points[i].y = map_[e].ground_truth_.y;
            message.points[i].z = map_[e].ground_truth_.z;
        }
        i++;
    }

    //    graph_traits<Map>::edge_iterator ei, ei_end;
    //    message.points.resize(2*num_edges(map));
    //    int i=0;
    //    for (tie(ei, ei_end) = edges(map); ei != ei_end; ++ei,i+=2){
    //        Experience v1 = source(*ei, map);
    //        Experience v2 = target(*ei, map);
    //        tf::pointTFToMsg(map[v1].pose.getOrigin(),message.points[i]);
    //        tf::pointTFToMsg(map[v2].pose.getOrigin(),message.points[i+1]);
    //    }

}

void ExperienceMap::createROSMessageError(dolphin_slam::Error &message)
{
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "error";

    //message.experience_map_error = localisationErrorEM_;
    //message.dead_reckoning_error = localisationErrorDR_;

    static float last_index = 0;
    message.experience_map_error = map_[current_experience_descriptor_].active_neuron_[0];
    message.dead_reckoning_error = message.experience_map_error - last_index;
    last_index = message.experience_map_error;
}
/*!
 * \brief Experience::createROSMessageMap
 */
void ExperienceMap::createROSMessageMap(visualization_msgs::Marker & message)
{
    //! completa o cabeçalho da mensagem
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "world";

    //! configura o tipo e a ação tomada pela mensagem
    message.type = visualization_msgs::Marker::LINE_STRIP;
    //    message.type = visualization_msgs::Marker::POINTS;
    message.action = visualization_msgs::Marker::ADD;
    message.ns = "map";
    message.id = 0;

    //! configura a pose dos marcadores
    message.pose.position.x = 0.0 ;
    message.pose.position.y = 0.0;
    message.pose.position.z = 0.0;

    message.pose.orientation.x = 0.0;
    message.pose.orientation.y = 0.0;
    message.pose.orientation.z = 0.0;
    message.pose.orientation.w = 1.0;

    message.scale.x = 0.1;
    message.scale.y = 0.1;
    message.scale.z = 0.1;

    //! configura a cor dos marcadores
    message.color.r = 0.0;
    message.color.g = 0.0;
    message.color.b = 1.0;
    message.color.a = 1.0;

    //! configura os marcadores para serem permanentes
    message.lifetime = ros::Duration(0.0);

    message.points.resize(boost::num_vertices(map_));
    int i = 0;
    foreach (ExperienceDescriptor e, boost::vertices(map_)) {
        if ( ROTATE_MAPS){
            message.points[i].x = map_[e].position_.x;
            message.points[i].y = -map_[e].position_.y;
            message.points[i].z = -map_[e].position_.z;
        }else{
            message.points[i].x = map_[e].position_.x;
            message.points[i].y = map_[e].position_.y;
            message.points[i].z = map_[e].position_.z;
        }

        //OS_DEBUG_STREAM_NAMED("em","active neuroun = " << map_[e].active_neuron_[0] << " " <<map_[e].active_neuron_[1] << " " <<map_[e].active_neuron_[2] << " " <<map_[e].active_neuron_[3] );
        i++;
    }

    //    graph_traits<Map>::edge_iterator ei, ei_end;
    //    message.points.resize(2*num_edges(map));
    //    int i=0;
    //    for (tie(ei, ei_end) = edges(map); ei != ei_end; ++ei,i+=2){
    //        Experience v1 = source(*ei, map);
    //        Experience v2 = target(*ei, map);
    //        tf::pointTFToMsg(map[v1].pose.getOrigin(),message.points[i]);
    //        tf::pointTFToMsg(map[v2].pose.getOrigin(),message.points[i+1]);
    //    }

}

/*!
 * \brief Experience::createROSMessageMap
 */
void ExperienceMap::createROSMessageDeadReckoning(visualization_msgs::Marker & message)
{
    //! completa o cabeçalho da mensagem
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "world";

    //! configura o tipo e a ação tomada pela mensagem
    message.type = visualization_msgs::Marker::LINE_STRIP;
    //    message.type = visualization_msgs::Marker::POINTS;
    message.action = visualization_msgs::Marker::ADD;
    message.ns = "dead_reckoning_map";
    message.id = 0;

    //! configura a pose dos marcadores
    message.pose.position.x = 0.0 ;
    message.pose.position.y = 0.0;
    message.pose.position.z = 0.0;

    message.pose.orientation.x = 0.0;
    message.pose.orientation.y = 0.0;
    message.pose.orientation.z = 0.0;
    message.pose.orientation.w = 1.0;

    message.scale.x = 0.1;
    message.scale.y = 0.1;
    message.scale.z = 0.1;

    //! configura a cor dos marcadores
    message.color.r = 1.0;
    message.color.g = 0.0;
    message.color.b = 0.0;
    message.color.a = 1.0;

    //! configura os marcadores para serem permanentes
    message.lifetime = ros::Duration(0.0);

    message.points.resize(boost::num_vertices(dead_reckoning_map_));
    int i = 0;
    foreach (ExperienceDescriptor e, boost::vertices(dead_reckoning_map_)) {
        if ( ROTATE_MAPS){
            message.points[i].x = dead_reckoning_map_[e].position_.x;
            message.points[i].y = -dead_reckoning_map_[e].position_.y;
            message.points[i].z = -dead_reckoning_map_[e].position_.z;
        }else{
            message.points[i].x = dead_reckoning_map_[e].position_.x;
            message.points[i].y = dead_reckoning_map_[e].position_.y;
            message.points[i].z = dead_reckoning_map_[e].position_.z;
        }
        i++;
    }

    //    graph_traits<Map>::edge_iterator ei, ei_end;
    //    message.points.resize(2*num_edges(map));
    //    int i=0;
    //    for (tie(ei, ei_end) = edges(map); ei != ei_end; ++ei,i+=2){
    //        Experience v1 = source(*ei, map);
    //        Experience v2 = target(*ei, map);
    //        tf::pointTFToMsg(map[v1].pose.getOrigin(),message.points[i]);
    //        tf::pointTFToMsg(map[v2].pose.getOrigin(),message.points[i+1]);
    //    }

}

/*!
 * \brief Function to publish map
 */
void ExperienceMap::publishExperienceMap()
{
    visualization_msgs::Marker message;
    createROSMessageMap(message);

    map_publisher_.publish(message);
}

/*!
 * \brief Function to publish ground truth
 */
void ExperienceMap::publishGroundTruth()
{
    visualization_msgs::Marker message;
    createROSMessageGroundTruth(message);

    ground_truth_publisher_.publish(message);
}

void ExperienceMap::publishError()
{
    dolphin_slam::Error message;
    createROSMessageError(message);

    error_publisher_.publish(message);
}


/*!
 * \brief Function to publish map
 */
void ExperienceMap::publishDeadReckoning()
{
    visualization_msgs::Marker message;
    createROSMessageDeadReckoning(message);

    dead_reckoning_publisher_.publish(message);
}


/*!
 * \brief Iterate map to minimize errors after loop closure events
 */
void ExperienceMap::iterateMap()
{

    cv::Point3f difference_i,difference_o;
    int ni,no;
    ExperienceDescriptor adjacent_experience;
    cv::Point3f first_experience_position;

    for(int n=0;n<10;n++){
        //! Para todos os vertices do mapa de experiencias
        foreach(ExperienceDescriptor experience, boost::vertices(map_))
        {

            //            if(index[experience] == 0){
            //                continue;
            //            }

            ni = no = 0;
            difference_i = cv::Point3f(0,0,0);
            difference_o = cv::Point3f(0,0,0);

            //! Seleciona todas as arestas que chegam no vertice escolhido
            foreach (LinkDescriptor link, boost::in_edges(experience,map_))
            {
                adjacent_experience = boost::source(link,map_);


                //! Calcula o erro baseado nas experiencias que chegam ao vertice
                difference_i += map_[adjacent_experience].position_ + map_[link].delta_position_ - map_[experience].position_;
                ni ++;
            }

            if(ni)
            {
                difference_i *= (1.0/ni);
            }

            //! Seleciona todas as arestas que saem do vertice escolhido
            foreach (LinkDescriptor link, boost::out_edges(experience,map_))
            {
                adjacent_experience = boost::target(link,map_);

                //! Calcula o erro baseado nas experiencias que saem ao vertice
                difference_o += map_[adjacent_experience].position_ - (map_[experience].position_ + map_[link].delta_position_ );
                no ++;
            }

            if(no)
            {
                difference_o *= (1.0/no);
            }

            map_[experience].position_ += 0.5*(difference_i + difference_o);

            if(map_[experience].id_ == 0)
            {
                first_experience_position = map_[experience].position_;
            }
        }



    }

    ROS_DEBUG_STREAM_NAMED("em","First experience position = " << first_experience_position);

    foreach(ExperienceDescriptor experience, boost::vertices(map_))
    {
        map_[experience].position_ = map_[experience].position_ - first_experience_position;
    }

}


} //namespace
