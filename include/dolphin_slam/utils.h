#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <opencv2/core/core.hpp>

namespace dolphin_slam
{

bool convert(std::vector<cv::Mat> &in, cv::Mat_<float> &out);

}


#endif // UTILS_H
