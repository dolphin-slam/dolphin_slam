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
    int active_neuron_[4];    //!< \deprecated Neuron active on creation of experience
    float activation_level_;

    int most_active_lv_cell_;
    std::vector<Cell> lv_cells_;
    cv::Mat_<float> pc_activity_;   //!< 4D Array of neurons

    cv::Point3f position_;          //!< Estimated Pose of experience
    cv::Point3f ground_truth_;      //!< Real robot Pose on experience position

    float rate_lv_;
    float rate_pc_;
    float rate_total_;

    void computeActivity(const ExperienceEventConstPtr &event)
    {

        float distance_pc_index = 0;
        for(int i = 0;i<event->pc_activity_.active_neuron_.size();i++)
        {
            distance_pc_index += pow(event->pc_activity_.active_neuron_[i] - active_neuron_[i],2);
        }

        //! Calcula o nível de ativação de acordo com a pose cell
        //testa se a distancia entre as células é de no máximo uma em cada direção
        if(distance_pc_index < event->pc_activity_.active_neuron_.size()) //nota-se que a distancia foi calculada ao quadrado
        {
            rate_pc_ = 1 - (activation_level_ - event->pc_activity_.activation_level_);
        }
        else
        {
            rate_pc_ = 0;
        }

        //! Calcula o nível de ativação de acordo com a local view ativa
        if(most_active_lv_cell_ == event->most_active_lv_cell_)
        {
            rate_lv_ = 1;
        }
        else
        {
            rate_lv_ = 0;
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
