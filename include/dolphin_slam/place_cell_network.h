#ifndef POSE_CELL_NETWORK_H
#define POSE_CELL_NETWORK_H

//ROS
#include <ros/ros.h>

#include <numeric>

#include <robot_state.h>

// Messages
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <underwater_sensor_msgs/DVL.h>
#include <dolphin_slam/ExperienceEvent.h>
#include <dolphin_slam/LocalViewNetwork.h>
#include <dolphin_slam/RobotPose.h>
#include <dolphin_slam/ExecutionTime.h>

#include <local_view_module.h>

// OpenCV
#include <opencv2/opencv.hpp>

#include <boost/foreach.hpp>

#include <cmath>

#include <color.h>
#include <time_monitor/time_monitor.h>
#include <angles/angles.h>

#include <boost/math/special_functions/modf.hpp>


#define foreach BOOST_FOREACH
#define reverse_foreach BOOST_REVERSE_FOREACH

namespace dolphin_slam
{

struct PlaceCellParameters
{
    std::vector<int> number_of_neurons_;
    std::vector<double> distance_between_neurons_;
    double input_learning_rate_;

    double recurrent_connection_std_;
    std::string local_view_activation_;
    std::string weight_function_;

};


class PlaceCellNetwork
{
public:
    PlaceCellNetwork();

    void loadParameters();
    void createROSSubscribers();
    void createROSPublishers();

    void createROSServices();
    void createROSTimers();


    void rectifyIndeces(int a[]);
    int getWrapIndex(int index, int dimension);


    void callRobotStateServices();
private:
    float squaredDistance(int center[], int new_index[]);
    float euclideanDistance(int center[], int new_index[]);
    float calculateMaxDistance();

    //! CANN
    void updateNetwork(); //!< \todo Decidir como será o update da rede
    void exciteNetwork();
    void learnExternalConnections();
    void applyExternalInputOnNetwork();
    void applyPathIntegrationOnNetwork();
    void limitNetworkActivity();
    void normalizeNetworkActivity();
    void getActiveNeuron(std::vector<int> &active_neuron);

    void integrateX(double delta);
    void integrateY(double delta);
    void integrateZ(double delta);

    //! Função para alocar a matriz de neuronios
    void createNeurons();
    //! Função para alocar a matriz de pesos
    void createExcitatoryWeights();

    void normalizeRecurrentExcitatoryWeights();


    //! callbacks
    void viewTemplateCallback(const LocalViewNetworkConstPtr &message); //!< \todo trocar para a mensagem de view template

    void timerCallback(const ros::TimerEvent &event);

    void publishNetworkActivityMessage();
    void publishExperienceMapEvent();
    void publishExecutionTime();

    PlaceCellParameters parameters_;

    //! Local View
    bool has_new_local_view_cell_;
    int most_active_lv_cell_;
    float lambda_learning_;
    float max_energy_;
    std::vector <Cell> lv_cells_active_;
    std::vector<cv::Mat_<float> > local_view_synaptic_weights_;    //!< matriz de pesos das conexões de entrada
    int max_view_template_id_;
    int external_input_min_age_;


    //! CANN
    cv::Mat_<float> neurons_;   //!< 4D Array of neurons
    cv::Mat_<float> aux_neurons_;  //!< Matriz auxiliar para atualização da rede

    cv::Mat_<float> recurrent_excitatory_weights_; //!< Matriz dos pesos de excitação.
    std::vector<int> active_index_;
    float k_external_activation_;
    float learning_constant_;


    //RobotState robot_state_;    //! Stores the travelled distance between updates

    bool experience_event_;     //! variable to indicate a event to the experience map

    //ROS Related Variable

    //! ROS Node Handle
    ros::NodeHandle node_handle_;

    //! ROS Topics
    ros::Subscriber view_template_subscriber_;
    ros::Subscriber DVL_subscriber_;
    ros::Subscriber IMU_subscriber_;

    ros::Publisher net_activity_publisher_;
    ros::Publisher net_activity_yaw_publisher_;
    ros::Publisher experience_event_publisher_;
    ros::Publisher execution_time_publisher_;


    ros::Timer timer_;

    ros::ServiceClient robot_state_pc_service;
    ros::ServiceClient robot_state_em_service;

    dolphin_slam::RobotPose robot_pose_pc_;
    dolphin_slam::RobotPose robot_pose_em_;

    LocalViewNetworkPtr local_view_network_msg_;

    TimeMonitor time_monitor_;

    static const int DIMS = 3;

};





} //namespace

#endif // POSE_CELL_NETWORK_H
