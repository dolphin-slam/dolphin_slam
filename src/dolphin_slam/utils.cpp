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

geometry_msgs::TransformStamped createTransformStamped(tf2::Transform &transform,ros::Time stamp, std::string frame_id, std::string child_frame_id)
{
    geometry_msgs::TransformStamped msg;

    msg.header.frame_id = frame_id;
    msg.header.seq = 0;
    msg.header.stamp = stamp;

    msg.child_frame_id = child_frame_id;

    tf2::Vector3 translation = transform.getOrigin();
    tf2::Quaternion quaternion = transform.getRotation();

    msg.transform.translation.x = translation.x();
    msg.transform.translation.y = translation.y();
    msg.transform.translation.z = translation.z();

    msg.transform.rotation.x = quaternion.x();
    msg.transform.rotation.y = quaternion.y();
    msg.transform.rotation.z = quaternion.z();
    msg.transform.rotation.w = quaternion.w();

    return msg;
}


tf2::Transform getTransform(geometry_msgs::TransformStamped &msg)
{

    tf2::Vector3 translation(msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z);
    tf2::Quaternion quaternion(msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w);
    tf2::Transform transform(quaternion,translation);

    return transform;
}



}
