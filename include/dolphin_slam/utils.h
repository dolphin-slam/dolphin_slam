#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>


namespace dolphin_slam
{

bool convert(std::vector<cv::Mat> &in, cv::Mat_<float> &out);

geometry_msgs::TransformStamped createTransformStamped(tf2::Transform &transform,ros::Time stamp, std::string frame_id, std::string child_frame_id);

tf2::Transform getTransform(geometry_msgs::TransformStamped &msg);

}




#endif // UTILS_H
