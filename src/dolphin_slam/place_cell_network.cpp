#include "place_cell_network.h"

using std::cout;
using std::endl;

int sign(double delta)
{
    if (delta == 0)
        return 0;
    if (delta > 0)
        return 1;
    if (delta < 0)
        return -1;
}

namespace dolphin_slam
{

/*!
*   \brief Constructor
*
*/
PlaceCellNetwork::PlaceCellNetwork()
{
    max_view_template_id_ = 0;
    most_active_lv_cell_ = -1;
    robot_pose_pc_.request.reset = true;
    robot_pose_em_.request.reset = true;

    loadParameters();

    createROSPublishers();

    createROSSubscribers();

    createROSServices();

    createNeurons();
    createExcitatoryWeights();

}

/*!
*   \brief Load configuration parameters
*
*/
void PlaceCellNetwork::loadParameters()
{
    ros::NodeHandle private_nh("~");

    private_nh.getParam("number_of_neurons",parameters_.number_of_neurons_);

    private_nh.getParam("distance_between_neurons",parameters_.distance_between_neurons_);

    private_nh.param<double>("recurrent_connection_std",parameters_.recurrent_connection_std_,2);

    private_nh.param<double>("input_learning_rate",parameters_.input_learning_rate_,1);

    private_nh.param<std::string>("local_view_activation",parameters_.local_view_activation_,"multiple");

    private_nh.param<std::string>("weight_function",parameters_.weight_function_,"mexican_hat");

    private_nh.param<int>("min_input_age",parameters_.min_input_age_,5);

}

void PlaceCellNetwork::createROSSubscribers()
{
    view_template_subscriber_ = node_handle_.subscribe("local_view_cells", 1, &PlaceCellNetwork::localViewCallback, this);

}

void PlaceCellNetwork::createROSPublishers()
{
    net_activity_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("network_activity",1);

    execution_time_publisher_ = node_handle_.advertise<dolphin_slam::ExecutionTime>("execution_time",1,false);


    experience_event_publisher_ = node_handle_.advertise<dolphin_slam::ExperienceEvent>("experience_event",1);

}

void PlaceCellNetwork::createROSServices()
{
    robot_state_pc_service = node_handle_.serviceClient<dolphin_slam::RobotPose>("robot_state_pc",true);
    robot_state_em_service = node_handle_.serviceClient<dolphin_slam::RobotPose>("robot_state_em",true);

    robot_state_pc_service.waitForExistence();
    robot_state_em_service.waitForExistence();
}

/*!
 * \brief Function to alocate a matrix of neurons
 */
void PlaceCellNetwork::createNeurons()
{
    neurons_.create(DIMS,&parameters_.number_of_neurons_[0]);
    aux_neurons_.create(DIMS,&parameters_.number_of_neurons_[0]);

    std::fill(neurons_.begin(),neurons_.end(),0);

    //! Put all activity in the first neurons, representing the initial position (x,y,z,yaw)->(0,0,0,0)
    *neurons_.begin() = 1;

}

//! Função para alocar a matriz de pesos
void PlaceCellNetwork::createExcitatoryWeights()
{

    std::vector<int> weight_number(DIMS);  //!< armazena o tamanho de cada dimensão da matriz de excitação
    double variance;
    int windex[DIMS];

    for(int i=0;i<DIMS;i++)
    {
        weight_number[i] = (parameters_.number_of_neurons_[i]+2)/2;
    }
    recurrent_excitatory_weights_.create(DIMS,&weight_number[0]);

    for(int i=0;i<weight_number[0];i++)
    {
        for(int j=0;j<weight_number[1];j++)
        {
            for(int k=0;k<weight_number[2];k++)
            {
                windex[0]=i;    windex[1]=j;    windex[2]=k;
                variance = pow(parameters_.recurrent_connection_std_,2);

                if(parameters_.weight_function_ == "gaussian")
                {
                    recurrent_excitatory_weights_(windex) =
                            exp(-(i*i+j*j+k*k)/(2*variance));
                }
                else if (parameters_.weight_function_ == "mexican_hat")
                {
                    recurrent_excitatory_weights_(windex) =
                            (1-(i*i+j*j+k*k)/(variance))*
                            exp(-(i*i+j*j+k*k)/(2*variance));
                }
                else
                {
                    ROS_ERROR("Wrong weight function. Consider revise the weight_function parameter");
                }

            }
        }
    }

    //! I don't know what is the behaviour of this function with mexican hat weights. \todo Revise it
    // normalizeRecurrentExcitatoryWeights();

    //! Print a 2D weight function for debugging
    cout << "weights = " << endl;
    cout << "[";
    for(int i=0;i<weight_number[0];i++)
    {
        if (i)
            cout << ";" << endl;

        for(int j=0;j<weight_number[1];j++)
        {
            windex[0]=i;    windex[1]=j;    windex[2]=0;
            if (j)
                cout << " , ";

            cout << recurrent_excitatory_weights_(windex);
        }
    }
    cout << "]" << endl;


}


void PlaceCellNetwork::normalizeRecurrentExcitatoryWeights()
{
    int index[DIMS];
    int i,j,k,l;    //! indices da matriz de neuronios

    int distances[DIMS];

    float normalization_factor;


    //! fill the aux_neurons_ matrix with zeros
    std::fill(aux_neurons_.begin(),aux_neurons_.end(),0);

    //! para cada neuronio da matriz
    for(i=0;i<parameters_.number_of_neurons_[0];i++)
    {
        for(j=0;j<parameters_.number_of_neurons_[1];j++)
        {
            for(k=0;k<parameters_.number_of_neurons_[2];k++)
            {
                //! para cada elemento da matriz de pesos
                //! preenche os indices em um vetor
                index[0]=i;  index[1]=j;  index[2]=k;
                //! Compute the distance between neurons in every dimension
                for(int d=0;d<DIMS;d++)
                {
                    distances[d] = std::min(index[d],parameters_.number_of_neurons_[d]-index[d]);
                }
                aux_neurons_(index)+= recurrent_excitatory_weights_(distances);

            }
        }
    }

    normalization_factor = std::accumulate(aux_neurons_.begin(),aux_neurons_.end(),0.0f);

    ROS_DEBUG_STREAM_NAMED("pc","Excitatory Weights Normalization Factor = " << normalization_factor);

    recurrent_excitatory_weights_ /= normalization_factor;

}

void PlaceCellNetwork::timerCallback(const ros::TimerEvent& event)
{

    update();
}


void PlaceCellNetwork::createROSTimers()
{
    timer_ = node_handle_.createTimer(ros::Duration(1), &PlaceCellNetwork::timerCallback,this);

}


void PlaceCellNetwork::localViewCallback(const ActiveLocalViewCellsConstPtr &message)
{
    ROS_DEBUG_STREAM("ViewTemplate Message Received ");

//    cout << "lcv_active = ";

    lv_cells_active_.resize(message->cell_id_.size());
    for(int i=0;i< lv_cells_active_.size(); i++)
    {
        lv_cells_active_[i].id_ = message->cell_id_[i];
        lv_cells_active_[i].rate_ = message->cell_rate_[i];
        lv_cells_active_[i].active_ = true;

        //cout << lv_cells_active_[i].id_ << " " << lv_cells_active_[i].rate_ << " ";
    }
    //cout << endl;

    experience_event_ = (most_active_lv_cell_ != message->most_active_cell_);

    //! atualiza a local view mais ativa no momento
    most_active_lv_cell_ = message->most_active_cell_;

    //cout << "view template id = " << most_active_lv_cell_ << endl;

    max_view_template_id_ = std::max(max_view_template_id_,most_active_lv_cell_);

    //! aloca novas posições na matriz de conexões caso ainda não existam
    if((max_view_template_id_+1) > static_cast<int>(local_view_synaptic_weights_.size()))
    {
        int old_size = local_view_synaptic_weights_.size();
        local_view_synaptic_weights_.resize(max_view_template_id_+1);//!< cria novas posições na matriz de conexões
        //! aloca os novos vetores criados
        for(std::vector<cv::Mat_<double> >::iterator it = local_view_synaptic_weights_.begin() + old_size;
            it < local_view_synaptic_weights_.end();it++)
        {
            //! Create a four dimension excitatory matrix
            it->create(DIMS,&parameters_.number_of_neurons_[0]);
            std::fill(it->begin(),it->end(),0.0);
        }
    }

    std::cout << "max_view_template_id_ = " << max_view_template_id_ << "weight size = " << local_view_synaptic_weights_.size() << std::endl;

    update();


}

void PlaceCellNetwork::callRobotStateServices()
{
    if(!robot_state_pc_service.call(robot_pose_pc_))
    {
        ROS_WARN_STREAM_NAMED("pc","Nao recebeu resposta do servico robot_pose_pc");

        //! se nao houver reposta, cria conexao, e espera que ela exista
        robot_state_pc_service = node_handle_.serviceClient<dolphin_slam::RobotPose>("robot_state_pc",true);
        robot_state_pc_service.waitForExistence();
        robot_state_pc_service.call(robot_pose_pc_);
    }

    if(experience_event_)
    {
        if(!robot_state_em_service.call(robot_pose_em_))
        {
            ROS_WARN_STREAM_NAMED("pc","Nao recebeu resposta do servico robot_pose_em");

            //! se nao houver reposta, cria conexao, e espera que ela exista
            robot_state_em_service = node_handle_.serviceClient<dolphin_slam::RobotPose>("robot_state_em",true);
            robot_state_em_service.waitForExistence();
            robot_state_em_service.call(robot_pose_em_);
        }
    }
}

void PlaceCellNetwork::update()
{
    time_monitor_.start();

    //! chama os serviços de dead reckoning do robô
    callRobotStateServices();

    //! excita a rede com uma função de ativação do tipo chapéu mexicano
    excite();

    //! realiza a integração de caminho na cann
    pathIntegration();

    //! normaliza a atividade na rede
    normalizeNetworkActivity();

    externalInput();

//    //! normaliza a atividade na rede
    normalizeNetworkActivity();

    //! publica as mensagens de saída
    if(experience_event_)
        publishExperienceMapEvent();

    publishNetworkActivityMessage();

    time_monitor_.finish();

    ROS_DEBUG_STREAM_NAMED("pc","Duration of update function = " << time_monitor_.getDuration() << "s");
    publishExecutionTime();

}

void PlaceCellNetwork::publishExecutionTime()
{
    ExecutionTime msg;
    msg.header.stamp = ros::Time::now();

    msg.module = "pc";
    msg.iteration_time = time_monitor_.getDuration();

    execution_time_publisher_.publish(msg);

}

void PlaceCellNetwork::rectifyIndeces(int a[DIMS])
{

    for(int i=0;i<DIMS;i++)
    {
        a[i] = getWrapIndex(a[i],i);
    }
}


int PlaceCellNetwork::getWrapIndex(int index,int dimension)
{
    int new_index = index;
    //! testa se ficou fora dos limites
    while(new_index<0)
    {
        new_index += parameters_.number_of_neurons_[dimension];
    }

    while(new_index>=parameters_.number_of_neurons_[dimension])
    {
        new_index -= parameters_.number_of_neurons_[dimension];
    }

    return new_index;
}


/*!
 * \brief Lateral Excitation of the network
 *  \todo Otimizar a função, pois o que é feito em um lado,
 * é feito de forma similar no lado oposto da matriz
 */
void PlaceCellNetwork::excite()
{
    time_monitor_.start();

    //! fill the aux_neurons_ matrix with zeros
    std::fill(aux_neurons_.begin(),aux_neurons_.end(),0);

    int index[DIMS],aindex[DIMS];
    int i,j,k;    //! indices da matriz de neuronios
    int ai,aj,ak;    //! indices da matriz de pesos

    int distances[4];
    int less,bigger;



    //! para cada neuronio da matriz
    for(i=0;i<parameters_.number_of_neurons_[0];i++)
    {
        for(j=0;j<parameters_.number_of_neurons_[1];j++)
        {
            for(k=0;k<parameters_.number_of_neurons_[2];k++)
            {
                //! para cada elemento da matriz de pesos
                //! preenche os indices em um vetor
                index[0]=i;  index[1]=j;  index[2]=k;
                //                    if(neurons_(index) > 1e-4)
                //                    {
                //! para cada neuronio da matriz
                for(ai=0;ai<parameters_.number_of_neurons_[0];ai++)
                {
                    for(aj=0;aj<parameters_.number_of_neurons_[1];aj++)
                    {
                        for(ak=0;ak<parameters_.number_of_neurons_[2];ak++)
                        {
                            aindex[0]=ai;  aindex[1]=aj;  aindex[2]=ak;

                            //! Compute the distance between neurons in every dimension
                            for(int d=0;d<DIMS;d++)
                            {
                                if(index[d]>aindex[d])
                                {
                                    bigger = index[d];
                                    less = aindex[d];
                                }
                                else
                                {
                                    less = index[d];
                                    bigger = aindex[d];
                                }
                                distances[d] = std::min(bigger-less,less+parameters_.number_of_neurons_[d]-bigger);
                            }
                            aux_neurons_(aindex)+= neurons_(index)*recurrent_excitatory_weights_(distances);

                        }
                    }
                }
                //                    }

            }
        }
    }

    //! adiciona a ativação atual dos neurônios com a excitação recorrente
    //! para cada neuronio da matriz
    for(i=0;i<parameters_.number_of_neurons_[0];i++)
    {
        for(j=0;j<parameters_.number_of_neurons_[1];j++)
        {
            for(k=0;k<parameters_.number_of_neurons_[2];k++)
            {
                //! para cada elemento da matriz de pesos
                //! preenche os indices em um vetor
                index[0]=i;  index[1]=j;  index[2]=k;
                //! \todo descobrir pq ue diminui o neuronio atual
                //neurons_(index) = std::max(aux_neurons_(index) - neurons_(index),0.0);
                neurons_(index) = std::max(aux_neurons_(index),0.0);
            }
        }
    }


    time_monitor_.finish();
    ROS_DEBUG_STREAM("Excite duration " << time_monitor_.getDuration() << "s");
}

void PlaceCellNetwork::externalInput()
{

    int local_view_age;

    //! Hebbian Learning
    learnExternalConnections();


    if(parameters_.local_view_activation_ == "multiple")
    {
        for(int lvc=0;lvc<lv_cells_active_.size();lvc++)
        {
            local_view_age = max_view_template_id_ - lv_cells_active_[lvc].id_;

            if(local_view_age >= parameters_.min_input_age_){
                //! apply external inputs
                neurons_ += lv_cells_active_[lvc].rate_ *
                        local_view_synaptic_weights_[lv_cells_active_[lvc].id_];
            }

        }
    }
    else if (parameters_.local_view_activation_ == "single")
    {

        local_view_age = max_view_template_id_ - most_active_lv_cell_;

        if(local_view_age >= parameters_.min_input_age_){
            //! apply external inputs
            neurons_ += local_view_synaptic_weights_[most_active_lv_cell_];
        }
    }
    else
    {
        ROS_ERROR("Wrong local view activation method. Revise local_view_activation_ parameter");
    }
}

float PlaceCellNetwork::squaredDistance(int center[], int new_index[])
{
    float distance = 0;
    for(int i=0;i<DIMS;i++)
    {
        distance += pow(center[i]-new_index[i],2);
    }
    return distance;
}

/*!
 * \brief Calculate the Euclidean distance between points
 */
float PlaceCellNetwork::euclideanDistance(int center[], int new_index[])
{
    float distance = 0;
    for(int i=0;i<DIMS;i++)
    {
        distance += pow(center[i]-new_index[i],2);
    }
    return sqrt(distance);
}

/*!
 * \brief Calculate max distance that a point can have
 */
float PlaceCellNetwork::calculateMaxDistance()
{
    float distance;

    for(int i=0;i<DIMS;i++)
    {
        distance += pow(parameters_.distance_between_neurons_[i],2);
    }
    return sqrt(distance);
}

void PlaceCellNetwork::integrateX(double delta)
{
    int i,j,k;    //! indices da matriz de neuronios
    int index[DIMS], aindex[DIMS];
    double num_cells;
    int integer_cells;
    double decimal_cells;
    int direction;
    int cell_min,cell_max;

    std::vector<double> weights(parameters_.number_of_neurons_[0],0);

    std::fill(aux_neurons_.begin(),aux_neurons_.end(),0.0f);


    //! calcula o numero de celulas para deslocar e a direção do deslocamento
    num_cells = delta/parameters_.distance_between_neurons_[0];

    //! split the number into integer and decimal part
    decimal_cells = fabs(boost::math::modf(num_cells,&integer_cells));
    direction = sign(delta);


    for(j=0;j<parameters_.number_of_neurons_[1];j++)
    {
        for(k=0;k<parameters_.number_of_neurons_[2];k++)
        {
            //! zera o vetor de pesos
            std::fill(weights.begin(),weights.end(),0.0);
            for(i=0;i<parameters_.number_of_neurons_[0];i++)
            {
                //! calcula as duas celulas que serao influenciadas pela atividade da celula k
                cell_min = i + integer_cells;               cell_min = getWrapIndex(cell_min,0);
                cell_max = cell_min + direction;            cell_max = getWrapIndex(cell_max,0);

                index[0]=i;         index[1]=j;     index[2]=k;
                aindex[0]=cell_min; aindex[1]=j;    aindex[2]=k;

                aux_neurons_(aindex) +=(1-decimal_cells)*neurons_(index);
                weights[cell_min] += 1-decimal_cells;

                aindex[0]=cell_max; aindex[1]=j;    aindex[2]=k;

                aux_neurons_(aindex) +=(decimal_cells)*neurons_(index);
                weights[cell_max] += decimal_cells;

            }
            //! Agora deve-se dividir cada neuronio atualizado pelo pelo associado `a ele
            for(i=0;i<parameters_.number_of_neurons_[0];i++)
            {
                aindex[0]=i;    aindex[1]=j;   aindex[2]=k;
                aux_neurons_(aindex) /= weights[i];
            }
        }
    }


    neurons_ = aux_neurons_.clone();

}

void PlaceCellNetwork::integrateY(double delta)
{
    int i,j,k;    //! indices da matriz de neuronios
    int index[DIMS], aindex[DIMS];
    double num_cells;
    int integer_cells;
    double decimal_cells;
    int direction;
    int cell_min,cell_max;

    std::vector<double> weights(parameters_.number_of_neurons_[1],0);

    std::fill(aux_neurons_.begin(),aux_neurons_.end(),0.0f);


    //! calcula o numero de celulas para deslocar e a direção do deslocamento
    num_cells = delta/parameters_.distance_between_neurons_[1];

    //! split the number into integer and decimal part
    decimal_cells = fabs(boost::math::modf(num_cells,&integer_cells));
    direction = sign(delta);

    for(i=0;i<parameters_.number_of_neurons_[0];i++)
    {
        for(k=0;k<parameters_.number_of_neurons_[2];k++)
        {
            //! zera o vetor de pesos
            std::fill(weights.begin(),weights.end(),0.0);
            for(j=0;j<parameters_.number_of_neurons_[1];j++)
            {
                //! calcula as duas celulas que serao influenciadas pela atividade da celula k
                cell_min = j + integer_cells;               cell_min = getWrapIndex(cell_min,1);
                cell_max = cell_min + direction;            cell_max = getWrapIndex(cell_max,1);


                index[0]=i;     index[1]=j;         index[2]=k;
                aindex[0]=i;    aindex[1]=cell_min; aindex[2]=k;

                aux_neurons_(aindex) +=(1-decimal_cells)*neurons_(index);
                weights[cell_min] += 1-decimal_cells;

                aindex[0]=i;    aindex[1]=cell_max; aindex[2]=k;

                aux_neurons_(aindex) +=(decimal_cells)*neurons_(index);
                weights[cell_max] += decimal_cells;

            }
            //! Agora deve-se dividir cada neuronio atualizado pelo pelo associado `a ele
            for(j=0;j<parameters_.number_of_neurons_[1];j++)
            {
                aindex[0]=i;    aindex[1]=j;   aindex[2]=k;
                aux_neurons_(aindex) /= weights[j];
            }
        }
    }


    neurons_ = aux_neurons_.clone();

}

void PlaceCellNetwork::integrateZ(double  delta)
{
    int i,j,k;    //! indices da matriz de neuronios
    int index[DIMS], aindex[DIMS];
    double num_cells;
    int integer_cells;
    double decimal_cells;
    int direction;
    int cell_min,cell_max;

    std::vector<double> weights(parameters_.number_of_neurons_[2],0);

    std::fill(aux_neurons_.begin(),aux_neurons_.end(),0.0f);

    //! calcula o numero de celulas para deslocar e a direção do deslocamento
    num_cells = delta/parameters_.distance_between_neurons_[2];

    //! split the number into integer and decimal part
    decimal_cells = fabs(boost::math::modf(num_cells,&integer_cells));
    direction = sign(delta);


    for(i=0;i<parameters_.number_of_neurons_[0];i++)
    {
        for(j=0;j<parameters_.number_of_neurons_[1];j++)
        {
            //! zera o vetor de pesos
            std::fill(weights.begin(),weights.end(),0.0);
            for(k=0;k<parameters_.number_of_neurons_[2];k++)
            {
                //! calcula as duas celulas que serao influenciadas pela atividade da celula k
                cell_min = k + integer_cells;       cell_min = getWrapIndex(cell_min,2);
                cell_max = cell_min + direction;    cell_max = getWrapIndex(cell_max,2);

                index[0]=i;     index[1]=j;     index[2]=k;
                aindex[0]=i;    aindex[1]=j;    aindex[2]=cell_min;

                aux_neurons_(aindex) +=(1-decimal_cells)*neurons_(index);
                weights[cell_min] += 1-decimal_cells;

                aindex[0]=i;    aindex[1]=j;    aindex[2]=cell_max;

                aux_neurons_(aindex) +=(decimal_cells)*neurons_(index);
                weights[cell_max] += decimal_cells;

            }
            //! Agora deve-se dividir cada neuronio atualizado pelo pelo associado `a ele
            for(k=0;k<parameters_.number_of_neurons_[2];k++)
            {
                aindex[0]=i;    aindex[1]=j;   aindex[2]=k;     ;
                aux_neurons_(aindex) /= weights[k];
            }

        }
    }

    neurons_ = aux_neurons_.clone();
}

/*!
 * \brief Path integration function
 *
 *Copia a atividade neuronal de uma celula para outra(s), respeitando a orientaçao da celula
 *
 */
void PlaceCellNetwork::pathIntegration()
{

    float delta_x,delta_y,delta_z;

    //! adquire a distancia percorrida em x,y,z e yaw
    delta_x = robot_pose_pc_.response.traveled_distance_.x;
    delta_y = robot_pose_pc_.response.traveled_distance_.y;
    delta_z = robot_pose_pc_.response.traveled_distance_.z;

    ROS_DEBUG_STREAM("Path Integration: [" << std::setw(6) << delta_x << ", " << std::setw(6) << delta_y << ", "
                     << std::setw(6) << delta_z << " ]" );

    integrateX(delta_x);
    integrateY(delta_y);
    integrateZ(delta_z);

}

//! Função responsável por fazer um aprendizado hebbiano
void PlaceCellNetwork::learnExternalConnections()
{
    int i,j,k;    //! indices da matriz de neuronios
    int index[3];

    std::cout << "parameters_.input_learning_rate_  "<< parameters_.input_learning_rate_ << std::endl;

    if(parameters_.local_view_activation_ == "multiple"){
        for(int lvc=0;lvc<lv_cells_active_.size();lvc++)
        {
            //! para cada neuronio da matriz
            for(i=0;i<parameters_.number_of_neurons_[0];i++)
            {
                for(j=0;j<parameters_.number_of_neurons_[1];j++)
                {
                    for(k=0;k<parameters_.number_of_neurons_[2];k++)
                    {

                        index[0]=i;  index[1]=j;  index[2]=k;
                        local_view_synaptic_weights_[lv_cells_active_[lvc].id_](index) =
                                std::max(local_view_synaptic_weights_[lv_cells_active_[lvc].id_](index),
                                parameters_.input_learning_rate_ * lv_cells_active_[lvc].rate_ * neurons_(index));
                    }
                }
            }

        }
    }
    else
    {
        //! para cada neuronio da matriz
        for(i=0;i<parameters_.number_of_neurons_[0];i++)
        {
            for(j=0;j<parameters_.number_of_neurons_[1];j++)
            {
                for(k=0;k<parameters_.number_of_neurons_[2];k++)
                {

                    index[0]=i;  index[1]=j;  index[2]=k;
                    local_view_synaptic_weights_[most_active_lv_cell_](index) =
                            std::max(local_view_synaptic_weights_[most_active_lv_cell_](index),
                            parameters_.input_learning_rate_ * neurons_(index));

                }
            }
        }
    }

    //! Local View Weights normalization
    //    float sum = std::accumulate(local_view_synaptic_weights_[view_template_id_].begin(),
    //local_view_synaptic_weights_[view_template_id_].end(),0.0f);
    //    local_view_synaptic_weights_[view_template_id_] /= sum;


}

void PlaceCellNetwork::limitNetworkActivity()
{
    int bigger_activity = *std::max_element(neurons_.begin(),neurons_.end());

    neurons_ /= bigger_activity;
}

void PlaceCellNetwork::normalizeNetworkActivity()
{
    double max = *std::max_element(neurons_.begin(),neurons_.end());


    neurons_ /= max;

    std::cout << "max activity = " << max << std::endl;
    //    float sum = std::accumulate(neurons_.begin(),neurons_.end(),0.0f);
    //    neurons_ /= sum;
}


double PlaceCellNetwork::getActiveNeuron(std::vector<int> &active_neuron)
{
    double max_activity = 0;
    int i,j,k;
    int index[DIMS];

    //! \todo otimizar essa funçao

    //! para cada neuronio da matriz
    for(i=0;i<parameters_.number_of_neurons_[0];i++)
    {
        for(j=0;j<parameters_.number_of_neurons_[1];j++)
        {
            for(k=0;k<parameters_.number_of_neurons_[2];k++)
            {

                index[0]=i;  index[1]=j;  index[2]=k;

                if(neurons_(index) > max_activity)
                {
                    max_activity = neurons_(index);
                    std::copy(index,index+DIMS,active_neuron.begin());
                }

            }
        }


    }

    ROS_DEBUG_STREAM_NAMED("pc","max activity = " << max_activity << " or " << *std::max_element(neurons_.begin(),neurons_.end())
                           << "neuron = " << active_neuron[0] << " " <<  active_neuron[1] << " " << active_neuron[2]  );

    return max_activity;

}



//! ###################### ROS MESSAGES ################################
void PlaceCellNetwork::publishNetworkActivityMessage()
{
    visualization_msgs::Marker message;
    int i,j,k;    //! indices da matriz de neuronios
    int index[DIMS];

    float scale = 0.25;
    Color color;

    //! completa o cabeçalho da mensagem
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "/network";

    //! configura o tipo e a ação tomada pela mensagem
    message.type = visualization_msgs::Marker::POINTS;
    message.action = visualization_msgs::Marker::ADD;
    message.ns = "network";
    message.id = 0;

    //! configura a pose dos marcadores
    message.pose.position.x = -parameters_.number_of_neurons_[0]/2;
    message.pose.position.y = -parameters_.number_of_neurons_[1]/2;
    message.pose.position.z = 0;

    message.pose.orientation.x = 0.0;
    message.pose.orientation.y = 0.0;
    message.pose.orientation.z = 0.0;
    message.pose.orientation.w = 1.0;

    message.scale.x  = message.scale.y = message.scale.z = scale;

    //! configura a cor dos marcadores
    message.color.r = 0.0;
    message.color.g = 0.0;
    message.color.b = 1.0;
    message.color.a = 1.0;

    //! configura os marcadores para serem permanentes
    message.lifetime = ros::Duration(0.0);

    message.points.resize(neurons_.total());
    message.colors.resize(neurons_.total());

    float max = *std::max_element(neurons_.begin(),neurons_.end());

    int point_index = 0;
    //! para cada neuronio da matriz
    for(i=0;i<parameters_.number_of_neurons_[0];i++)
    {
        for(j=0;j<parameters_.number_of_neurons_[1];j++)
        {
            for(k=0;k<parameters_.number_of_neurons_[2];k++)
            {

                index[0]=i;  index[1]=j;  index[2]=k;

                //! set point position
                message.points[point_index].x = i;
                message.points[point_index].y = j;
                message.points[point_index].z = k;

                color.setColor(neurons_(index)/max);

                //! set point color, according to network activity;
                message.colors[point_index].r = color.getR();
                message.colors[point_index].g = color.getG();
                message.colors[point_index].b = color.getB();
                message.colors[point_index].a = 1.0;

                point_index++;


            }
        }
    }

    net_activity_publisher_.publish(message);

}

void PlaceCellNetwork::publishExperienceMapEvent()
{
    dolphin_slam::ExperienceEvent message;

    std::vector<int> active_index(DIMS);

    message.most_active_lv_cell_id_ = most_active_lv_cell_;
    message.lv_cell_id_.resize(lv_cells_active_.size());
    message.lv_cell_rate_.resize(lv_cells_active_.size());
    for(int i=0;i<lv_cells_active_.size();i++)
    {
        message.lv_cell_id_[i] =lv_cells_active_[i].id_;
        message.lv_cell_rate_[i] = lv_cells_active_[i].rate_;
    }

    message.traveled_distance_ = robot_pose_em_.response.traveled_distance_;

    message.ground_truth_ = robot_pose_em_.response.ground_truth_;

    //! set active index


    message.pc_rate_ = getActiveNeuron(active_index);
    message.pc_index_.resize(DIMS);
    std::copy(active_index.begin(),active_index.end(),message.pc_index_.begin());

    message.pc_activity_.number_of_neurons_.resize(DIMS);
    std::copy(parameters_.number_of_neurons_.begin(),parameters_.number_of_neurons_.end(),message.pc_activity_.number_of_neurons_.begin());

    message.pc_activity_.activity_.resize(neurons_.total());
    std::copy(neurons_.begin(),neurons_.end(),message.pc_activity_.activity_.begin());

    experience_event_publisher_.publish(message);


}

}  //namespace
