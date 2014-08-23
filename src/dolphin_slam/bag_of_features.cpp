#include "bag_of_features.h"

namespace dolphin_slam
{

/*!
 * \brief Constructor
 */
BoF::BoF()
{

    threshold_ = 100;
    //! create FlannBased matcher
    matcher_ = cv::DescriptorMatcher::create("FlannBased");

    //! create SURF extractor
    extractor_ = cv::DescriptorExtractor::create("SURF");

    //! create SURF detector
    detector_ = new cv::SurfFeatureDetector(threshold_);

    //! create BoW Image Descriptor Extractor
    bofDE_ = new cv::BOWImgDescriptorExtractor(extractor_,matcher_);
}

/*!
 * \brief Destructor
 */
BoF::~BoF()
{
    //! Realease Pointers
    delete detector_;
    delete bofDE_;
}

/*!
 * \brief Set the SURF's threshold
 * \param Threshold : int
 */
void BoF::setThreshold(int threshold)
{
    //! Set threshold value
    threshold_ = threshold;


    delete detector_;
    //! Set threshold value at the SURF detector
    detector_ = new cv::SurfFeatureDetector(threshold_);
}

/*!
 * \brief Get the SURF's threshold
 * \return Threshold : int
 */
int BoF::getThreshold()
{
    //! Return threshold value
    return threshold_;
}

std::vector<cv::KeyPoint> BoF::getKeypoints()
{
    return keypts_;
}

/*!
 * \brief Create Histogram from Mat
 * \param Image : Mat
 * \return Histogram : Mat
 */
cv::Mat BoF::createHistogram(const cv::Mat &image)
{
    //! Detect SURF Features in the image
    detector_->detect(image, keypts_);
    //! Create the Image's Histogram
    bofDE_->compute(image,keypts_,histogram_);

    //! Return histogram
    return histogram_;
}

/*!
 * \brief Read the BoW's Dictionary
 * \param Path to dictionary : const char*
 */
void BoF::readDictionary(const char* path)
{
    //! Create FileStorage
    cv::FileStorage fs;

    //! Open File especified in the path
    if(fs.open(path, cv::FileStorage::READ))
    {
        ROS_DEBUG_STREAM("File sucessful opened: " << path);
    }
    else
    {
        ROS_WARN_STREAM("Could not open the file: " << path);
    }

    //! Read the dictionary from File
    fs["Dicionario"] >> dictionary_;
    //! Release the File
    fs.release();

    //! Set BoW Vocabulary
    bofDE_->setVocabulary(dictionary_);
}

} // namespace
