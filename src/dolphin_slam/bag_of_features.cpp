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
	
	/*std::vector<double> ordenador;
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
	}*/
	    //cout << "Number of keypoints = " << dictionary.rows << endl;
 
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
        //cout << endl;
    }
 
//    cout << "max_dist = " << max_dist << endl;
//    cout << "line1 = " << line1 << endl;
//    cout << "line2 = " << line2 << endl;
 
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
 
        //cout << distance[i] << endl;
    }
 
 
    float menor_distancia;
//    new_dictionary.row(0) = dictionary.row(line1);
    //cout << "copy line " << line1 << " to 0"  << endl;
    //copyLine(dictionary,line1,new_dictionary,0);
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
        //cout << "copy line " << line << " to " << i << endl;
        vocabulary_.row(line).copyTo(new_vocabulary_.row(i));
        //copyLine(dictionary,line,new_dictionary,i);
        new_distance[i] = distance[line];
        alread_on_list[line] = true;
    }
    //cout << "copy line " << line2 << " to " << new_dictionary.rows-1 << endl;
    //copyLine(dictionary,line2,new_dictionary,new_dictionary.rows-1);
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

