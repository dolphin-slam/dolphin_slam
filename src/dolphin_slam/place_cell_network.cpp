#include "place_cell_network.h"

using std::cout;
using std::endl;

namespace dolphin_slam
{

/*!
*   \brief Constructor
*
*/
PlaceCellNetwork::PlaceCellNetwork():
    number_of_recurrent_excitatory_weights_(4,1)
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

    private_nh.getParam("distance_between_neurons_",parameters_.distance_between_neurons_);

    private_nh.getParam("excitatory_variance",parameters_.excitatory_variance_);

    private_nh.param<double>("input_learning_rate",parameters_.input_learning_rate_,1);

    private_nh.param<bool>("multiple_local_view_active",parameters_.multiple_local_view_active_,true);

    private_nh.param<bool>("use_gaussian_weights",parameters_.use_gaussian_weights_,false);

}

void PlaceCellNetwork::createROSSubscribers()
{
    view_template_subscriber_ = node_handle_.subscribe("local_view_cells", 1, &PlaceCellNetwork::viewTemplateCallback, this);

}

void PlaceCellNetwork::createROSPublishers()
{
    net_activity_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("network_activity",1);
    net_activity_yaw_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("network_activity_yaw",1);

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

/*! \brief Function to alocate a matrix of neurons

*/
void PlaceCellNetwork::createNeurons()
{
    neurons_.create(4,&parameters_.number_of_neurons_[0]);
    aux_neurons_.create(4,&parameters_.number_of_neurons_[0]);

    initNeuronsActivity();
}

//! Função para alocar a matriz de pesos
void PlaceCellNetwork::createExcitatoryWeights()
{

    for(int i=0;i<number_of_recurrent_excitatory_weights_.size();i++)
    {
        number_of_recurrent_excitatory_weights_[i] = (parameters_.number_of_neurons_[i]+2)/2;
    }

    recurrent_excitatory_weights_.create(4,&number_of_recurrent_excitatory_weights_[0]);

    initRecurrentExcitatoryWeights();

}

void PlaceCellNetwork::initNeuronsActivity()
{
    std::fill(neurons_.begin(),neurons_.end(),0);

    //! Put all activity in the first neurons, representing the initial position (x,y,z,yaw)->(0,0,0,0)
    *neurons_.begin() = 1;

}

void PlaceCellNetwork::initRecurrentExcitatoryWeights()
{

    //int index[4];
    float dist;

    float factor = 0;
    std::vector<std::vector<float> >weights_per_dimension(4);

    for(int i=0;i<4;i++)
    {
        weights_per_dimension[i].resize(number_of_recurrent_excitatory_weights_[i]);
        cout << "Excitatory weigths dimension "<< i << " = [";
        for(int j=0;j<number_of_recurrent_excitatory_weights_[i];j++)
        {
            dist = j*parameters_.distance_between_neurons_[i];
            weights_per_dimension[i][j] = exp(-(dist*dist)/(2*parameters_.excitatory_variance_[i]));
            cout << weights_per_dimension[i][j] << ", ";
        }
        cout << "]" << endl;
    }




    int windex[4];

    if(parameters_.use_gaussian_weights_)
    {
        for(int i=0;i<number_of_recurrent_excitatory_weights_[0];i++)
        {
            for(int j=0;j<number_of_recurrent_excitatory_weights_[1];j++)
            {
                for(int k=0;k<number_of_recurrent_excitatory_weights_[2];k++)
                {
                    for(int l=0;l<number_of_recurrent_excitatory_weights_[3];l++)
                    {
                        windex[0]=i;    windex[1]=j;    windex[2]=k;    windex[3]=l;

                        recurrent_excitatory_weights_(windex) =
                                weights_per_dimension[0][i]*
                                weights_per_dimension[1][j]*
                                weights_per_dimension[2][k]*
                                weights_per_dimension[3][l];
                    }
                }
            }
        }
    }
    else
    {


        for(int i=0;i<number_of_recurrent_excitatory_weights_[0];i++)
        {
            for(int j=0;j<number_of_recurrent_excitatory_weights_[1];j++)
            {
                for(int k=0;k<number_of_recurrent_excitatory_weights_[2];k++)
                {
                    for(int l=0;l<number_of_recurrent_excitatory_weights_[3];l++)
                    {
                        windex[0]=i;    windex[1]=j;    windex[2]=k;    windex[3]=l;
                        factor = pow(i*parameters_.distance_between_neurons_[0],2)/parameters_.excitatory_variance_[0] +
                                pow(j*parameters_.distance_between_neurons_[1],2)/parameters_.excitatory_variance_[1] +
                                pow(k*parameters_.distance_between_neurons_[2],2)/parameters_.excitatory_variance_[2] +
                                pow(l*parameters_.distance_between_neurons_[3],2)/parameters_.excitatory_variance_[3];

                        recurrent_excitatory_weights_(windex) = (1 - factor)*
                                weights_per_dimension[0][i]*
                                weights_per_dimension[1][j]*
                                weights_per_dimension[2][k]*
                                weights_per_dimension[3][l];
                    }
                }
            }
        }

    }

    normalizeRecurrentExcitatoryWeights();

    //    cout << "weights = " << endl;
    //    for(int i=0;i<number_of_recurrent_excitatory_weights_[0];i++)
    //    {
    //        for(int j=0;j<number_of_recurrent_excitatory_weights_[1];j++)
    //        {
    //            windex[0]=i;    windex[1]=j;    windex[2]=0;    windex[3]=0;
    //            cout << recurrent_excitatory_weights_(windex)<< " ";
    //        }
    //        cout << endl;
    //    }
    //    cout << endl;

}

void PlaceCellNetwork::normalizeRecurrentExcitatoryWeights()
{
    int index[4];
    int i,j,k,l;    //! indices da matriz de neuronios

    int distances[4];

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
                for(l=0;l<parameters_.number_of_neurons_[3];l++)
                {
                    //! para cada elemento da matriz de pesos
                    //! preenche os indices em um vetor
                    index[0]=i;  index[1]=j;  index[2]=k;  index[3]=l;

                    //! Compute the distance between neurons in every dimension
                    for(int d=0;d<4;d++)
                    {
                        distances[d] = std::min(index[d],parameters_.number_of_neurons_[d]-index[d]);
                    }
                    aux_neurons_(index)+= recurrent_excitatory_weights_(distances);

                }
            }
        }
    }

    normalization_factor = std::accumulate(aux_neurons_.begin(),aux_neurons_.end(),0.0f);

    ROS_DEBUG_STREAM_NAMED("pc","Excitatory Weights Normalization Factor = " << normalization_factor);

    recurrent_excitatory_weights_ /= normalization_factor;

}

void PlaceCellNetwork::timerCallback(const ros::TimerEvent& event)
{

    updateNetwork();
}


void PlaceCellNetwork::createROSTimers()
{
    timer_ = node_handle_.createTimer(ros::Duration(1), &PlaceCellNetwork::timerCallback,this);

}


void PlaceCellNetwork::viewTemplateCallback(const dolphin_slam::LocalViewNetworkConstPtr &message)
{
    ROS_DEBUG_STREAM("ViewTemplate Message Received ");

    //! \todo Decidir quando chamar o evento de nova experiência
    has_new_local_view_cell_ = message->has_new_cell_;

    lv_cells_active_.resize(message->active_cells_.size());
    for(int i=0;i< message->active_cells_.size(); i++)
    {
        lv_cells_active_[i].id_ = message->active_cells_[i].id_;
        lv_cells_active_[i].rate_ = message->active_cells_[i].rate_;
    }

    if(most_active_lv_cell_ != message->most_active_cell_)
    {
        experience_event_ = true;
    }
    else
    {
        experience_event_ = false;
    }

    //! atualiza a local view mais ativa no momento
    most_active_lv_cell_ = message->most_active_cell_;

    cout << "view template id = " << most_active_lv_cell_ << endl;


    max_view_template_id_ = message->number_of_cells_ -1;

    //! aloca novas posições na matriz de conexões caso ainda não existam
    if(message->number_of_cells_ > static_cast<int>(local_view_synaptic_weights_.size()))
    {
        int old_size = local_view_synaptic_weights_.size();
        local_view_synaptic_weights_.resize(message->number_of_cells_);//!< cria novas posições na matriz de conexões
        //! aloca os novos vetores criados
        for(std::vector<cv::Mat_<float> >::iterator it = local_view_synaptic_weights_.begin() + old_size;
            it < local_view_synaptic_weights_.end();it++)
        {
            //! Create a four dimension excitatory matrix
            it->create(4,&parameters_.number_of_neurons_[0]);
            std::fill(it->begin(),it->end(),0);
        }
    }

    updateNetwork();


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

void PlaceCellNetwork::updateNetwork()
{
    time_monitor_.start();

    //! chama os serviços de dead reckoning do robô
    callRobotStateServices();

    //! excita a rede com uma função de ativação do tipo chapéu mexicano
    exciteNetwork();

    //! realiza a integração de caminho na cann
    applyPathIntegrationOnNetwork();

    //applyExternalInputOnNetwork();

    //! normaliza a atividade na rede
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

void PlaceCellNetwork::rectifyIndeces(int a[4])
{

    for(int i=0;i<4;i++)
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
void PlaceCellNetwork::exciteNetwork()
{
    time_monitor_.start();

    //! fill the aux_neurons_ matrix with zeros
    std::fill(aux_neurons_.begin(),aux_neurons_.end(),0);

    int index[4],aindex[4];
    int i,j,k,l;    //! indices da matriz de neuronios
    int ai,aj,ak,al;    //! indices da matriz de pesos

    int distances[4];
    int less,bigger;



    //! para cada neuronio da matriz
    for(i=0;i<parameters_.number_of_neurons_[0];i++)
    {
        for(j=0;j<parameters_.number_of_neurons_[1];j++)
        {
            for(k=0;k<parameters_.number_of_neurons_[2];k++)
            {
                for(l=0;l<parameters_.number_of_neurons_[3];l++)
                {
                    //! para cada elemento da matriz de pesos
                    //! preenche os indices em um vetor
                    index[0]=i;  index[1]=j;  index[2]=k;  index[3]=l;
//                    if(neurons_(index) > 1e-4)
//                    {
                        //! para cada neuronio da matriz
                        for(ai=0;ai<parameters_.number_of_neurons_[0];ai++)
                        {
                            for(aj=0;aj<parameters_.number_of_neurons_[1];aj++)
                            {
                                for(ak=0;ak<parameters_.number_of_neurons_[2];ak++)
                                {
                                    for(al=0;al<parameters_.number_of_neurons_[3];al++)
                                    {
                                        aindex[0]=ai;  aindex[1]=aj;  aindex[2]=ak;  aindex[3]=al;

                                        //! Compute the distance between neurons in every dimension
                                        for(int d=0;d<4;d++)
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
                        }
//                    }
                }
            }
        }
    }

    //! adiciona a ativação atual dos neurônios com a excitação recorrente
    neurons_ += aux_neurons_;

    time_monitor_.finish();
    ROS_DEBUG_STREAM("Excite duration " << time_monitor_.getDuration() << "s");
}

void PlaceCellNetwork::applyExternalInputOnNetwork()
{

    int local_view_age;

    //! Hebbian Learning
    learnExternalConnections();


    if(parameters_.multiple_local_view_active_)
    {
        for(int lvc=0;lvc<lv_cells_active_.size();lvc++)
        {
            local_view_age = max_view_template_id_ - lv_cells_active_[lvc].id_;

            if(local_view_age >= external_input_min_age_){
                //! apply external inputs
                neurons_ += lv_cells_active_[lvc].rate_ *
                        local_view_synaptic_weights_[lv_cells_active_[lvc].id_];
            }

        }
    }
    else
    {

        local_view_age = max_view_template_id_ - most_active_lv_cell_;

        if(local_view_age >= external_input_min_age_){
            //! apply external inputs
            neurons_ += local_view_synaptic_weights_[most_active_lv_cell_];

            cout << "weight = ";
            foreach(float weight, local_view_synaptic_weights_[most_active_lv_cell_])
            {
                cout << weight << " ";
            }
            cout << endl;
        }


    }
}

float PlaceCellNetwork::squaredDistance(int center[], int new_index[])
{
    float distance = 0;
    for(int i=0;i<3;i++)
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
    for(int i=0;i<3;i++)
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

    for(int i=0;i<4;i++)
    {
        distance += pow(parameters_.distance_between_neurons_[i],2);
    }
    return sqrt(distance);
}

void PlaceCellNetwork::integrateX(float delta_x,float delta_y,float delta_z,float delta_o)
{
    int i,j,k,l;    //! indices da matriz de neuronios
    int index[4], aindex[4];
    float num_cells;
    int num_cells_int;
    float num_cells_float;
    int direction;
    int cell_min,cell_max;
    float delta;

    std::vector<float> weights(parameters_.number_of_neurons_[0],0);

    std::fill(aux_neurons_.begin(),aux_neurons_.end(),0.0f);

    //! fixa as dimensoes y,z e yaw para deslocar na dimensao x
    for(l=0;l<parameters_.number_of_neurons_[3];l++)
    {
        //! Calcula a distancia de acordo com a rotação
        delta = delta_x*cos(l*parameters_.number_of_neurons_[3]+delta_o/2.0) +
                delta_y*sin(l*parameters_.number_of_neurons_[3]+delta_o/2.0);

        //! calcula o numero de celulas para deslocar e a direção do deslocamento
        num_cells = delta/parameters_.distance_between_neurons_[0];
        get_integer_decimal_part(num_cells,num_cells_int,num_cells_float);
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
                    cell_min = i + num_cells_int;               cell_min = getWrapIndex(cell_min,0);
                    cell_max = cell_min + direction;            cell_max = getWrapIndex(cell_max,0);

                    index[0]=i;         index[1]=j;     index[2]=k;     index[3]=l;
                    aindex[0]=cell_min; aindex[1]=j;    aindex[2]=k;    aindex[3]=l;

                    aux_neurons_(aindex) +=(1-num_cells_float)*neurons_(index);
                    weights[cell_min] += 1-num_cells_float;

                    aindex[0]=cell_max; aindex[1]=j;    aindex[2]=k;    aindex[3]=l;

                    aux_neurons_(aindex) +=(num_cells_float)*neurons_(index);
                    weights[cell_max] += num_cells_float;

                }
                //! Agora deve-se dividir cada neuronio atualizado pelo pelo associado `a ele
                for(i=0;i<parameters_.number_of_neurons_[0];i++)
                {
                    aindex[0]=i;    aindex[1]=j;   aindex[2]=k;     aindex[3]=l;
                    aux_neurons_(aindex) /= weights[i];
                }
            }
        }
    }

    neurons_ = aux_neurons_.clone();

}

void PlaceCellNetwork::integrateY(float delta_x,float delta_y,float delta_z,float delta_o)
{
    int i,j,k,l;    //! indices da matriz de neuronios
    int index[4], aindex[4];
    float num_cells;
    int num_cells_int;
    float num_cells_float;
    int direction;
    int cell_min,cell_max;
    float delta;

    std::vector<float> weights(parameters_.number_of_neurons_[1],0);

    std::fill(aux_neurons_.begin(),aux_neurons_.end(),0.0f);

    //! fixa as dimensoes y,z e yaw para deslocar na dimensao x
    for(l=0;l<parameters_.number_of_neurons_[3];l++)
    {
        //! Calcula a distancia de acordo com a rotação
        delta = -delta_x*sin(l*parameters_.number_of_neurons_[3]+delta_o/2.0) +
                delta_y*cos(l*parameters_.number_of_neurons_[3]+delta_o/2.0);

        //! calcula o numero de celulas para deslocar e a direção do deslocamento
        num_cells = delta/parameters_.distance_between_neurons_[1];
        get_integer_decimal_part(num_cells,num_cells_int,num_cells_float);
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
                    cell_min = j + num_cells_int;           cell_min = getWrapIndex(cell_min,1);
                    cell_max = cell_min + direction;        cell_max = getWrapIndex(cell_max,1);

                    index[0]=i;     index[1]=j;         index[2]=k;     index[3]=l;
                    aindex[0]=i;    aindex[1]=cell_min; aindex[2]=k;    aindex[3]=l;

                    aux_neurons_(aindex) +=(1-num_cells_float)*neurons_(index);
                    weights[cell_min] += 1-num_cells_float;

                    aindex[0]=i;    aindex[1]=cell_max; aindex[2]=k;    aindex[3]=l;

                    aux_neurons_(aindex) +=(num_cells_float)*neurons_(index);
                    weights[cell_max] += num_cells_float;

                }
                //! Agora deve-se dividir cada neuronio atualizado pelo pelo associado `a ele
                for(j=0;j<parameters_.number_of_neurons_[1];j++)
                {
                    aindex[0]=i;    aindex[1]=j;   aindex[2]=k;     aindex[3]=l;
                    aux_neurons_(aindex) /= weights[j];
                }
            }
        }
    }

    neurons_ = aux_neurons_.clone();

}

void PlaceCellNetwork::integrateZ(float delta_x, float delta_y, float delta_z, float delta_o)
{
    int i,j,k,l;    //! indices da matriz de neuronios
    int index[4], aindex[4];
    float num_cells;
    int num_cells_int;
    float num_cells_float;
    int direction;
    int cell_min,cell_max;
    float delta;

    std::vector<float> weights(parameters_.number_of_neurons_[2],0);

    std::fill(aux_neurons_.begin(),aux_neurons_.end(),0.0f);

    delta = delta_z;
    //! calcula o numero de celulas para deslocar e a direção do deslocamento
    num_cells = delta/parameters_.distance_between_neurons_[2];
    get_integer_decimal_part(num_cells,num_cells_int,num_cells_float);
    direction = sign(delta);

    ROS_DEBUG_STREAM_NAMED("pc","delta_z = " << delta_z << " cells = " << num_cells_int << " " << num_cells_float);

    for(i=0;i<parameters_.number_of_neurons_[0];i++)
    {
        for(j=0;j<parameters_.number_of_neurons_[1];j++)
        {
            for(l=0;l<parameters_.number_of_neurons_[3];l++)
            {
                //! zera o vetor de pesos
                std::fill(weights.begin(),weights.end(),0.0);
                for(k=0;k<parameters_.number_of_neurons_[2];k++)
                {
                    //! calcula as duas celulas que serao influenciadas pela atividade da celula k
                    cell_min = k + num_cells_int;       cell_min = getWrapIndex(cell_min,2);
                    cell_max = cell_min + direction;    cell_max = getWrapIndex(cell_max,2);

                    index[0]=i;     index[1]=j;     index[2]=k;         index[3]=l;
                    aindex[0]=i;    aindex[1]=j;    aindex[2]=cell_min; aindex[3]=l;

                    aux_neurons_(aindex) +=(1-num_cells_float)*neurons_(index);
                    weights[cell_min] += 1-num_cells_float;

                    aindex[0]=i;    aindex[1]=j;    aindex[2]=cell_max; aindex[3]=l;

                    aux_neurons_(aindex) +=(num_cells_float)*neurons_(index);
                    weights[cell_max] += num_cells_float;

                }
                //! Agora deve-se dividir cada neuronio atualizado pelo pelo associado `a ele
                for(k=0;k<parameters_.number_of_neurons_[2];k++)
                {
                    aindex[0]=i;    aindex[1]=j;   aindex[2]=k;     aindex[3]=l;
                    aux_neurons_(aindex) /= weights[k];
                }
            }
        }
    }

    neurons_ = aux_neurons_.clone();
}

void PlaceCellNetwork::integrateYaw(float delta_x, float delta_y, float delta_z, float delta_o)
{
    int i,j,k,l;    //! indices da matriz de neuronios
    int index[4], aindex[4];
    float num_cells;
    int num_cells_int;
    float num_cells_float;
    int direction;
    int cell_min,cell_max;
    float delta;

    std::vector<float> weights(parameters_.number_of_neurons_[2],0);

    std::fill(aux_neurons_.begin(),aux_neurons_.end(),0.0f);

    delta = delta_o;
    //! calcula o numero de celulas para deslocar e a direção do deslocamento
    num_cells = delta/parameters_.distance_between_neurons_[3];
    get_integer_decimal_part(num_cells,num_cells_int,num_cells_float);
    direction = sign(delta);

    for(i=0;i<parameters_.number_of_neurons_[0];i++)
    {
        for(j=0;j<parameters_.number_of_neurons_[1];j++)
        {
            for(k=0;k<parameters_.number_of_neurons_[2];k++)
            {
                //! zera o vetor de pesos
                std::fill(weights.begin(),weights.end(),0.0);
                for(l=0;l<parameters_.number_of_neurons_[3];l++)
                {
                    //! calcula as duas celulas que serao influenciadas pela atividade da celula k
                    cell_min = l + num_cells_int;           cell_min = getWrapIndex(cell_min,3);
                    cell_max = cell_min + direction;        cell_max = getWrapIndex(cell_max,3);

                    index[0]=i;     index[1]=j;     index[2]=k;     index[3]=l;
                    aindex[0]=i;    aindex[1]=j;    aindex[2]=k;    aindex[3]=cell_min;

                    aux_neurons_(aindex) +=(1-num_cells_float)*neurons_(index);
                    weights[cell_min] += 1-num_cells_float;

                    aindex[0]=i;    aindex[1]=j;    aindex[2]=k;    aindex[3]=cell_max;

                    aux_neurons_(aindex) +=(num_cells_float)*neurons_(index);
                    weights[cell_max] += num_cells_float;

                }
                //! Agora deve-se dividir cada neuronio atualizado pelo pelo associado `a ele
                for(l=0;l<parameters_.number_of_neurons_[3];l++)
                {
                    aindex[0]=i;    aindex[1]=j;   aindex[2]=k;     aindex[3]=l;
                    aux_neurons_(aindex) /= weights[l];
                }
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
 *Calcula o shift necessário em z
 *Em cada camada yaw, calcula a rotação de acordo com a metade da rotação realizada + a orientação da célula
 *e a shift necessário na dimensão yaw
 *Em cada plano x, y  calcula o shift em cada célula
 */
void PlaceCellNetwork::applyPathIntegrationOnNetwork()
{

    float delta_x,delta_y,delta_z,delta_o;

    //! adquire a distancia percorrida em x,y,z e yaw
    delta_x = robot_pose_pc_.response.traveled_distance_.x;
    delta_y = robot_pose_pc_.response.traveled_distance_.y;
    delta_z = robot_pose_pc_.response.traveled_distance_.z;
    delta_o = 0;

    ROS_DEBUG_STREAM_NAMED("pc","Deltas = [" << std::setw(6) << delta_x << ", " << std::setw(6) << delta_y << ", " << std::setw(6) << delta_z << ", "<< std::setw(6) << delta_o << " ]" );

    integrateZ(delta_x,delta_y,delta_z,delta_o);
    integrateX(delta_x,delta_y,delta_z,delta_o);
    integrateY(delta_x,delta_y,delta_z,delta_o);
    //integrateYaw(delta_x,delta_y,delta_z,delta_o);

}

//! Função responsável por fazer um aprendizado hebbiano
void PlaceCellNetwork::learnExternalConnections()
{
    int i,j,k,l;    //! indices da matriz de neuronios
    int index[4];
    float weight;


    if(parameters_.multiple_local_view_active_){

        //! lvc = local view cell
        foreach (Cell& lvc, lv_cells_active_) {


            //! para cada neuronio da matriz
            for(i=0;i<parameters_.number_of_neurons_[0];i++)
            {
                for(j=0;j<parameters_.number_of_neurons_[1];j++)
                {
                    for(k=0;k<parameters_.number_of_neurons_[2];k++)
                    {
                        for(l=0;l<parameters_.number_of_neurons_[3];l++)
                        {
                            index[0]=i;  index[1]=j;  index[2]=k;  index[3]=l;
                            //                    delta_weight = learning_constant_* neurons_(index);
                            //                    local_view_synaptic_weights_[view_template_id_](index) += delta_weight;
                            local_view_synaptic_weights_[lvc.id_](index) =
                                    std::max(local_view_synaptic_weights_[lvc.id_](index),
                                    learning_constant_ * lvc.rate_ * neurons_(index));
                        }
                    }
                }
            }

        }
    }else
    {
        //! para cada neuronio da matriz
        for(i=0;i<parameters_.number_of_neurons_[0];i++)
        {
            for(j=0;j<parameters_.number_of_neurons_[1];j++)
            {
                for(k=0;k<parameters_.number_of_neurons_[2];k++)
                {
                    for(l=0;l<parameters_.number_of_neurons_[3];l++)
                    {
                        index[0]=i;  index[1]=j;  index[2]=k;  index[3]=l;
                        //                    delta_weight = learning_constant_* neurons_(index);
                        //                    local_view_synaptic_weights_[view_template_id_](index) += delta_weight;
                        local_view_synaptic_weights_[most_active_lv_cell_](index) =
                                std::max(local_view_synaptic_weights_[most_active_lv_cell_](index),
                                learning_constant_ * neurons_(index));
                    }
                }
            }
        }
    }

    //! Local View Weights normalization
    //    float sum = std::accumulate(local_view_synaptic_weights_[view_template_id_].begin(),local_view_synaptic_weights_[view_template_id_].end(),0.0f);
    //    local_view_synaptic_weights_[view_template_id_] /= sum;


}

void PlaceCellNetwork::limitNetworkActivity()
{
    int bigger_activity = *std::max_element(neurons_.begin(),neurons_.end());

    neurons_ /= bigger_activity;
}

void PlaceCellNetwork::normalizeNetworkActivity()
{
    float sum = std::accumulate(neurons_.begin(),neurons_.end(),0.0f);
    neurons_ /= sum;
}


void PlaceCellNetwork::getActiveNeuron(std::vector<int> &active_neuron)
{
    float max_activity = 0;
    int i,j,k,l;
    int index[4];

    //! \todo otimizar essa funçao

    //! para cada neuronio da matriz
    for(i=0;i<parameters_.number_of_neurons_[0];i++)
    {
        for(j=0;j<parameters_.number_of_neurons_[1];j++)
        {
            for(k=0;k<parameters_.number_of_neurons_[2];k++)
            {
                for(l=0;l<parameters_.number_of_neurons_[3];l++)
                {
                    index[0]=i;  index[1]=j;  index[2]=k;  index[3]=l;

                    if(neurons_(index) > max_activity)
                    {
                        max_activity = neurons_(index);
                        std::copy(index,index+4,active_neuron.begin());
                    }
                }
            }
        }


    }

    ROS_DEBUG_STREAM_NAMED("pc","max activity = " << max_activity << " or " << *std::max_element(neurons_.begin(),neurons_.end()) << "neuron = " << active_neuron[0] << " " <<  active_neuron[1] << " " << active_neuron[2] << " " << active_neuron[3] );

}



int sign(float value)
{
    return(value < 0 ? -1: 1);
}

void get_integer_decimal_part(float value, int &integer,float &decimal)
{
    decimal = static_cast<float>(fabs(boost::math::modf(value,&integer)));
}


//! ###################### ROS MESSAGES ################################
void PlaceCellNetwork::publishNetworkActivityMessage()
{
    visualization_msgs::Marker message;
    int i,j,k,l;    //! indices da matriz de neuronios
    int index[4];

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
                for(l=0;l<parameters_.number_of_neurons_[3];l++)
                {
                    index[0]=i;  index[1]=j;  index[2]=k;  index[3]=l;

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
    }

    net_activity_publisher_.publish(message);

}

void PlaceCellNetwork::publishExperienceMapEvent()
{
    dolphin_slam::ExperienceEvent message;

    std::vector<int> active_index(4);

    message.lv_cells_active_.resize(lv_cells_active_.size());
    for(int i=0;i<lv_cells_active_.size();i++)
    {
        message.lv_cells_active_[i].id_ =lv_cells_active_[i].id_;
        message.lv_cells_active_[i].rate_ = lv_cells_active_[i].rate_;
    }

    message.has_new_local_view_cell_ = has_new_local_view_cell_;

    message.most_active_lv_cell_ = most_active_lv_cell_;

    message.traveled_distance_ = robot_pose_em_.response.traveled_distance_;

    message.ground_truth_ = robot_pose_em_.response.ground_truth_;

    //! set active index
    getActiveNeuron(active_index);

    message.pc_activity_.active_neuron_.resize(parameters_.number_of_neurons_.size());
    std::copy(active_index.begin(),active_index.end(),message.pc_activity_.active_neuron_.begin());
    message.pc_activity_.activation_level_ = neurons_(&active_index[0]);

    experience_event_publisher_.publish(message);


}

}  //namespace
