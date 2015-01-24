#include "bow_descriptor_extractor.h"

BOWImgDescriptorExtractor::BOWImgDescriptorExtractor( const cv::Ptr<cv::DescriptorMatcher>& _dmatcher ) :
    dmatcher(_dmatcher)
{}

BOWImgDescriptorExtractor::~BOWImgDescriptorExtractor()
{}

void BOWImgDescriptorExtractor::setVocabulary( const cv::Mat& _vocabulary )
{
    dmatcher->clear();
    vocabulary = _vocabulary;
    dmatcher->add( vector<cv::Mat>(1, vocabulary) );
}

const cv::Mat& BOWImgDescriptorExtractor::getVocabulary() const
{
    return vocabulary;
}

void BOWImgDescriptorExtractor::compute( const cv::Mat& descriptors, cv::Mat& imgDescriptor,
                                         vector<vector<int> >* pointIdxsOfClusters)
{
    imgDescriptor.release();


    int clusterCount = descriptorSize(); // = vocabulary.rows

    // Match keypoint descriptors to cluster center (to vocabulary)
    vector<cv::DMatch> matches;
    dmatcher->match( descriptors, matches );

    // Compute image descriptor
    if( pointIdxsOfClusters )
    {
        pointIdxsOfClusters->clear();
        pointIdxsOfClusters->resize(clusterCount);
    }

    imgDescriptor = cv::Mat( 1, clusterCount, descriptorType(), cv::Scalar::all(0.0) );
    float *dptr = (float*)imgDescriptor.data;
    for( size_t i = 0; i < matches.size(); i++ )
    {
        int queryIdx = matches[i].queryIdx;
        int trainIdx = matches[i].trainIdx; // cluster index
        CV_Assert( queryIdx == (int)i );

        dptr[trainIdx] = dptr[trainIdx] + 1.f;
        if( pointIdxsOfClusters )
            (*pointIdxsOfClusters)[trainIdx].push_back( queryIdx );
    }

    // Normalize image descriptor.
    imgDescriptor /= descriptors.rows;
}

void BOWImgDescriptorExtractor::computeBowInteger( const cv::Mat& descriptors, cv::Mat& imgDescriptor,
                                         vector<vector<int> >* pointIdxsOfClusters)
{
    imgDescriptor.release();

    int clusterCount = descriptorSize(); // = vocabulary.rows

    // Match keypoint descriptors to cluster center (to vocabulary)
    vector<cv::DMatch> matches;
    dmatcher->match( descriptors, matches );

    // Compute image descriptor
    if( pointIdxsOfClusters )
    {
        pointIdxsOfClusters->clear();
        pointIdxsOfClusters->resize(clusterCount);
    }

    imgDescriptor = cv::Mat( 1, clusterCount, cv::DataType<int>::type);

    std::fill(imgDescriptor.begin<int>(),imgDescriptor.end<int>(),0);

    int *dptr = (int*)imgDescriptor.data;
    for( size_t i = 0; i < matches.size(); i++ )
    {
        int queryIdx = matches[i].queryIdx;
        int trainIdx = matches[i].trainIdx; // cluster index
        CV_Assert( queryIdx == (int)i );

        dptr[trainIdx] = dptr[trainIdx] + 1;
        if( pointIdxsOfClusters )
            (*pointIdxsOfClusters)[trainIdx].push_back( queryIdx );
    }
}


int BOWImgDescriptorExtractor::descriptorSize() const
{
    return vocabulary.empty() ? 0 : vocabulary.rows;
}

int BOWImgDescriptorExtractor::descriptorType() const
{
    return CV_32FC1;
}
