#include "utils.h"

namespace dolphin_slam
{

bool convert(std::vector<cv::Mat> &in, cv::Mat_<float> &out)

{
    int rows = in.size();
    int cols = in[0].cols;

    out.create(rows,cols);
    for(int i=0;i<rows;i++)
    {
        for(int j=0;j<cols;j++)
        {
            out[i][j] = in[i].at<float>(0,j);
        }
    }
    return true;
}

//bool convert(cv::Mat_<double> &in, std::vector<cv::Mat> &out)
//{
//    int rows = in.rows;
//    int cols = in.cols;


//    out.resize(rows);
//    for(int i=0;i<rows;i++)
//    {
//        out[i].create(0,cols,CV_64F);
//        for(int j=0;j<cols;j++)
//        {
//            out[i].at<double>(0,j) = in[i][j];
//        }
//    }
//    return true;
//}


}
