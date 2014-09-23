#include "bag_of_features.h"

namespace dolphin_slam
{

/*!
 * \brief Constructor
 */
BoF::BoF(int number_of_groups, int surf_threshold)
{

    groups_  = number_of_groups;
    surf_threshold_ = surf_threshold;

    //! create FlannBased matcher
    matcher_ = cv::DescriptorMatcher::create("FlannBased");

    //! create SURF detector
    detector_ = new cv::SurfFeatureDetector(surf_threshold_);

    //! create SURF extractor
    extractor_ = cv::DescriptorExtractor::create("SURF");


    //! create BoW Image Descriptor Extractor
    bofDE_ = new cv::BOWImgDescriptorExtractor(extractor_,matcher_);

    trainer_ = new cv::BOWKMeansTrainer(groups_,
                                              cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,10,1.0),
                                              1,
                                              cv::KMEANS_PP_CENTERS);
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
    surf_threshold_ = threshold;

    delete detector_;
    //! Set threshold value at the SURF detector
    detector_ = new cv::SurfFeatureDetector(surf_threshold_);
}

/*!
 * \brief Get the SURF's threshold
 * \return Threshold : int
 */
int BoF::getThreshold()
{
    //! Return threshold value
    return surf_threshold_;
}

void BoF::addTrainingImage(cv::Mat image)
{
    cv::Mat features;

    //! Detect SURF Features in the image
    detector_->detect(image, keypts_);

    //! Compute SURF descriptors
    extractor_->compute(image, keypts_, features);

    //! Add descriptors to training set
    trainer_->add(features);
}

void BoF::train()
{
    vocabulary_ = trainer_->cluster();
}

std::vector<cv::KeyPoint> BoF::getKeypoints()
{
    return keypts_;
}
int BoF::getGroups()
{
    return groups_;
}

void BoF::setGroups(int groups)
{
    groups_ = groups;

    delete trainer_;
    trainer_ = new cv::BOWKMeansTrainer(groups_,
                                              cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,10,1.0),
                                              1,
                                              cv::KMEANS_PP_CENTERS);
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

//! Write serialization for this class
void BoF::write(cv::FileStorage& fs) const{

    fs << "{";
    fs << "groups" << groups_;
    fs << "threshold" << surf_threshold_;
    fs << "vocabulary" << vocabulary_;
    fs << "}";

}

void BoF::read(const cv::FileNode &node)
{
    node["groups"] >> groups_;

    node["threshold"] >> surf_threshold_;
    setThreshold(surf_threshold_); //! Call to adjust surf parameters

    node["vocabulary"] >> vocabulary_;
    bofDE_->setVocabulary(vocabulary_);

}

/*!
 * \brief Read the BoW's Dictionary
 * \param Path to dictionary : const char*
 */
void BoF::readVocabulary(string path)
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
    fs["Dicionario"] >> vocabulary_;
    //! Release the File
    fs.release();

    //! Set BoW Vocabulary
    bofDE_->setVocabulary(vocabulary_);
}

void BoF::sortVocab()
{
	
	std::vector<double> ordenador;
	for (int i = 0; i<vocabulary_.rows; i++)
	{
		for (int j = 0; j < vocabulary_.cols; j++)
		{
			ordenador.push_back(vocabulary_.at<float>(i,j));
		}
	}
	
	std::sort(ordenador.begin(), ordenador.end());
	
	int iterations = 0;
	for (int i = 0; i<vocabulary_.rows; i++)
	{
		for (int j = 0; j < vocabulary_.cols; j++)
		{
			vocabulary_.at<float>(i,j) = ordenador[iterations];
			iterations++;
		}
	}
	
  
}

//These write and read functions must be defined for the serialization in FileStorage to work
void write(cv::FileStorage& fs, const std::string&, const BoF& bof)
{
    bof.write(fs);
}

void read(const cv::FileNode& node, BoF& bof, const BoF& default_value)
{
    bof.read(node);
}

} // namespace

