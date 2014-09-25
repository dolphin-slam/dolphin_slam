#include "bag_of_features.h"

namespace dolphin_slam
{

/*!
 * \brief Constructor
 */
BoF::BoF(int number_of_groups, int hessian_threshold)
{

    groups_  = number_of_groups;
    hessian_threshold_ = hessian_threshold;

    surf_ = new cv::SURF(hessian_threshold_);

    //! create BoW Image Descriptor Extractor
    descriptor_extractor_ = new cv::BOWImgDescriptorExtractor(surf_,cv::DescriptorMatcher::create("FlannBased"));

    trainer_ = new cv::BOWKMeansTrainer(groups_,
                                        cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,50,0.001),
                                        5,
                                        cv::KMEANS_PP_CENTERS);
}

/*!
 * \brief Destructor
 */
BoF::~BoF()
{
    //! The cv::Ptr release the memory automatically.

}

/*!
 * \brief Set the SURF's threshold
 * \param Threshold : int
 */
void BoF::setThreshold(int threshold)
{
    //! Set threshold value
    hessian_threshold_ = threshold;

    surf_ = new cv::SURF(hessian_threshold_);

}

/*!
 * \brief Get the SURF's threshold
 * \return Threshold : int
 */
int BoF::getThreshold()
{
    //! Return threshold value
    return hessian_threshold_;
}

void BoF::addTrainingImage(cv::Mat image)
{
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;

    //! Detect SURF keypoints in the image
    surf_->detect(image,keypoints);
    //! Compute SURF descriptors
    surf_->compute(image,keypoints,descriptors);
    //! Add descriptors to training set

    trainer_->add(descriptors);

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

    trainer_ = new cv::BOWKMeansTrainer(groups_,
                                              cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,50,0.001),
                                              5,
                                              cv::KMEANS_PP_CENTERS);
}


/*!
 * \brief Create Histogram from Mat
 * \param Image : Mat
 * \return Histogram : Mat
 */
cv::Mat BoF::createHistogram(const cv::Mat &image)
{
    cv::Mat histogram;
    std::vector<cv::KeyPoint> keypoints;

    //! Detect SURF Features in the image
    surf_->detect(image, keypoints);

    //! Create the Image's Histogram
    descriptor_extractor_->compute(image,keypoints,histogram);

    //! Return histogram
    return histogram;
}

//! Write serialization for this class
void BoF::write(cv::FileStorage& fs) const{

    fs << "{";
    fs << "groups" << groups_;
    fs << "hessian_threshold" << hessian_threshold_;
    fs << "vocabulary" << vocabulary_;
    fs << "}";

}

void BoF::read(const cv::FileNode &node)
{
    node["groups"] >> groups_;
    setGroups(groups_);

    node["hessian_threshold"] >> hessian_threshold_;
    setThreshold(hessian_threshold_); //! Call to adjust surf parameters

    node["vocabulary"] >> vocabulary_;
    descriptor_extractor_->setVocabulary(vocabulary_);

}


void BoF::sortVocabulary()
{
	
    new_vocabulary_.create(vocabulary_.size(),vocabulary_.type());
 
    int line1, line2;
 
    float dist, max_dist = 0;
 
    for(int i=0;i<vocabulary_.rows;i++)
    {
        for(int j=i+1;j<vocabulary_.rows;j++)
        {
            dist = cv::norm(vocabulary_.row(i),vocabulary_.row(j));
            if(dist > max_dist)
            {
                line1 = i;
                line2 = j;
                max_dist = dist;
            }
        }
    }
  
    //! ja tenho os dois descritores mais distantes.
    std::vector<double> distance(vocabulary_.rows);
    std::vector<double> new_distance(vocabulary_.rows);
    std::vector<bool> alread_on_list(vocabulary_.rows,false);
 
    for(int i=0;i<distance.size();i++)
    {
        if(i == line1)
        {
            distance[i] = 0;
            alread_on_list[i] = true;
        }
        else if(i == line2)
        {
            distance[i] = max_dist;
            alread_on_list[i] = true;
        }
        else
        {
            distance[i] = cv::norm(vocabulary_.row(line1),vocabulary_.row(i));
        }
     }
 
 
    float menor_distancia;

    vocabulary_.row(line1).copyTo(new_vocabulary_.row(0));
    new_distance[0] = distance[line1];
    int line;
    for(int i=1;i<new_vocabulary_.rows-1;i++)
    {
        menor_distancia = numeric_limits<float>::infinity();
        for(int j=0;j<vocabulary_.rows;j++)
        {
            if(!alread_on_list[j])
            {
                if(menor_distancia > distance[j])
                {
                    menor_distancia = distance[j];
                    line = j;
                }
            }
        }
        vocabulary_.row(line).copyTo(new_vocabulary_.row(i));
        new_distance[i] = distance[line];
        alread_on_list[line] = true;
    }
    vocabulary_.row(line2).copyTo(new_vocabulary_.row(new_distance.size()-1));
    new_distance[new_distance.size()-1] = distance[line2];
 
 	vocabulary_ = new_vocabulary_;
	
  
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

