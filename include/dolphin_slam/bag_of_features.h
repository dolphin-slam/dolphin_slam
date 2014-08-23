#ifndef BOW_H
#define BOW_H

//! OpenCV Include
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <ros/ros.h>

//! Vector Include
#include <vector>

using namespace std;

namespace dolphin_slam
{

class BoF
{
public:
    BoF();
    ~BoF();

    void readDictionary(const char *path);
    cv::Mat createHistogram(const cv::Mat &image);

    void setThreshold(int threshold);
    int getThreshold();


    std::vector<cv::KeyPoint>  getKeypoints();
private:
    int threshold_;
    cv::Mat dictionary_;
    cv::Mat histogram_;

    vector<cv::KeyPoint> keypts_;

    cv::SurfFeatureDetector *detector_;
    cv::BOWImgDescriptorExtractor *bofDE_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;

};

} // namespace

#endif // BOW_H
