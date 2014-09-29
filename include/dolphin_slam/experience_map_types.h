#ifndef EXPERIENCE_MAP_TYPES_H
#define EXPERIENCE_MAP_TYPES_H

#include <boost/graph/adjacency_list.hpp> //!< biblioteca para grafos
#include <boost/graph/graph_traits.hpp>
#include <opencv/cv.h>

#include <local_view_module.h>
#include <dolphin_slam/ExperienceEvent.h>

namespace dolphin_slam
{

/*!
\brief Data Types to be used in Experience Map (\class ExperienceMap)
*/

/*!
  Struct Experience, responsible to store the Experience variables
*/
struct Experience
{
    int id_;
    int pc_index_[3];    //!< Neuron active on creation of experience

    int lv_cell_id_;

    cv::Point3f position_;          //!< Estimated Pose of experience
    cv::Point3f ground_truth_;      //!< Real robot Pose on experience position

    double rate_lv_;
    double rate_pc_;
    double rate_total_;

    void computeActivity(const ExperienceEventConstPtr &event)
    {

        //! pega o indice da place cell pertencente a experiência
        int pc_index = pc_index_[0]*event->pc_activity_.number_of_neurons_[1]*event->pc_activity_.number_of_neurons_[2]+
                pc_index_[1]*event->pc_activity_.number_of_neurons_[2]+
                pc_index_[2];

        //! seta a taxa de ativação atual do neurônio
        rate_pc_ = event->pc_activity_.activity_[pc_index];

        //! procura pelo id da local view na mensagem recebida. se encontrar, teremos a taxa de ativação.
        rate_lv_ = 0;
        for(int i=0;i<event->lv_cell_id_.size();i++)
        {
            if(event->lv_cell_id_[i] == lv_cell_id_)
            {
                rate_lv_= event->lv_cell_rate_[i];
            }
        }

        rate_total_ = 0.5*rate_lv_ + 0.5*rate_pc_;
    }

};


/*!
* \brief Properties of a link
*/
struct Link
{
    cv::Point3f delta_position_;
};


/*!
  Topological Map using the boost Graph
  */
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,Experience, Link> Map;

//! Experience Descriptor
typedef boost::graph_traits<Map>::vertex_descriptor ExperienceDescriptor;

//! Link Descriptor
typedef boost::graph_traits<Map>::edge_descriptor LinkDescriptor;

//! Experience iterator
typedef boost::graph_traits<Map>::vertex_iterator ExperienceIterator;


typedef std::pair <LinkDescriptor,bool> ResultOfLinkCreation;

} // namespace

#endif // EXPERIENCE_MAP_TYPES_H
