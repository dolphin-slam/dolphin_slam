#ifndef BOW_DESCRIPTOR_EXTRACTOR_H
#define BOW_DESCRIPTOR_EXTRACTOR_H

 //! \todo Usar a versao do opencv ao atualizar para o opencv 3.0

/*
 * Class to compute image descriptor using bag of visual words.
 */

#include <opencv2/core/core.hpp>
#include <opencv2/flann/miniflann.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <limits>
#include <vector>

using std::vector;

/*
 * Class to compute image descriptor using bag of visual words.
 */
class CV_EXPORTS BOWImgDescriptorExtractor
{
public:
    BOWImgDescriptorExtractor( const cv::Ptr<cv::DescriptorMatcher>& dmatcher );
    virtual ~BOWImgDescriptorExtractor();

    void setVocabulary( const cv::Mat& vocabulary);
    const cv::Mat& getVocabulary() const;
    void compute( const cv::Mat& descriptors, cv::Mat& imgDescriptor,
                  vector<vector<int> >* pointIdxsOfClusters=0);
    void computeBowInteger(const cv::Mat &descriptors, cv::Mat &imgDescriptor, vector<vector<int> > *pointIdxsOfClusters = 0);
    // compute() is not constant because DescriptorMatcher::match is not constant

    int descriptorSize() const;
    int descriptorType() const;

protected:
    cv::Mat vocabulary;
    cv::Ptr<cv::DescriptorMatcher> dmatcher;
};

#endif // BOW_DESCRIPTOR_EXTRACTOR_H
