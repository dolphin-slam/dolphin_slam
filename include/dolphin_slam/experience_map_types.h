#ifndef EXPERIENCE_MAP_TYPES_H
#define EXPERIENCE_MAP_TYPES_H

#include <boost/graph/adjacency_list.hpp> //!< biblioteca para grafos
#include <boost/graph/graph_traits.hpp>

#include <local_view_module.h>
#include <dolphin_slam/ExperienceEvent.h>

#include <tf2/LinearMath/Transform.h>

#include <opencv2/core/mat.hpp>


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

    cv::Mat image_;

    tf2::Transform pose_;
    tf2::Transform gt_pose_;
    tf2::Transform dr_pose_;

    double rate_lv_;
    double rate_pc_;
    double rate_total_;

    double computeSimilarity(Experience &other){

        rate_pc_ = 1;
        for(int i;i<3;i++)
        {
            if(pc_index_ != other.pc_index_)
            {
                rate_pc_ = 0;
            }
        }

        //! procura pelo id da local view na mensagem recebida. se encontrar, teremos a taxa de ativação.
        if(lv_cell_id_ == other.lv_cell_id_)
        {
            rate_lv_ = 1;
        }
        else
        {
            rate_lv_ = 0;
        }

//        rate_total_ = 0.5*rate_lv_ + 0.5*rate_pc_;

        rate_total_ = rate_lv_;

        return rate_total_;
    }

};


/*!
* \brief Properties of a link
*/
struct Link
{
    tf2::Vector3 translation_; //! expressed in world frame
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
