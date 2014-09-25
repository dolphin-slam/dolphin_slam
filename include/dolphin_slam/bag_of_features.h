#ifndef BOW_H
#define BOW_H

//! OpenCV Include
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
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
    BoF(int number_of_groups = 100, int hessian_threshold = 100);
    ~BoF();

    void readVocabulary(string path);
    cv::Mat createHistogram(const cv::Mat &image);

    void setThreshold(int threshold);
    int getThreshold();

    void addTrainingImage(cv::Mat image);

    void train();

    void write(cv::FileStorage &fs) const;
    void read(const cv::FileNode& node);

    std::vector<cv::KeyPoint>  getKeypoints();
    int getGroups();
    void setGroups(int groups);
    void sortVocabulary();

private:
    int hessian_threshold_;
    int groups_;
    cv::Mat vocabulary_;
    cv::Mat new_vocabulary_;
    cv::Mat histogram_;

    vector<cv::KeyPoint> keypts_;

    cv::Ptr<cv::SURF> surf_;

    cv::Ptr<cv::BOWImgDescriptorExtractor> descriptor_extractor_;
    cv::Ptr<cv::BOWKMeansTrainer> trainer_;

};

//These write and read functions must be defined for the serialization in FileStorage to work
void write(cv::FileStorage& fs, const std::string&, const BoF& bof);
void read(const cv::FileNode& node, BoF& bof, const BoF& default_value = BoF());


} // namespace

#endif // BOW_H
