#include "image_processing.h"

namespace dolphin_slam {



ImageProcessing::ImageProcessing() : it_(node_handle_)
{

    loadParameters();

    init();

    createROSSubscribers();

    createROSPublishers();

    createROSServices();

    log_file_.open("image_processing.log");

}

ImageProcessing::~ImageProcessing()
{
    log_file_.close();
}


void ImageProcessing::loadParameters()
{
    ros::NodeHandle private_nh_("~");

    //! int surf_threshold;
    private_nh_.param<int>("surf_threshold",parameters_.surf_threshold_,100);

    //! string image_topic;
    private_nh_.param<string>("image_topic",parameters_.image_topic_,"/image_raw");

    private_nh_.param<string>("sonar_topic",parameters_.sonar_topic_,"/sonar");

    //! string image_transport;
    private_nh_.param<string>("image_transport",parameters_.image_transport_,"raw");

    //! string image_transport;
    private_nh_.param<string>("sonar_transport",parameters_.sonar_transport_,"raw");

    //! string output_topic;
    private_nh_.param<string>("descriptors_topic",parameters_.descriptors_topic_,"/descriptors");

    //! string output_topic;
    private_nh_.param<string>("keypoints_topic",parameters_.image_keypoints_topic_,"/image_keypoints");

    //! int frames_to_jump;
    private_nh_.param<int>("frames_to_jump",parameters_.frames_to_jump_,0);

    private_nh_.param<string>("source",parameters_.source_,"camera");

    private_nh_.param<string>("sonar_mask",parameters_.sonar_mask_,"sonar_mask.jpg");


    private_nh_.param<string>("image_detector",parameters_.image_detector_,"surf");

    private_nh_.param<bool>("apply_roi",parameters_.apply_roi_,false);

    private_nh_.param<bool>("use_selected_images",parameters_.use_selected_images_,false);

    private_nh_.param<string>("selected_images_file",parameters_.selected_images_file_,"good_images.txt");

}


void ImageProcessing::createROSSubscribers()
{


    if(parameters_.source_ == "camera")
    {

        //! hint to modify the image_transport. Here I use raw transport
        image_transport::TransportHints hints(parameters_.image_transport_,ros::TransportHints(),node_handle_);

        //! image subscription
        image_subscriber_ = it_.subscribe(parameters_.image_topic_,1,&ImageProcessing::imageCallback,this,hints);

    }
    else if (parameters_.source_ == "sonar")
    {

        //! hint to modify the image_transport. Here I use raw transport
        image_transport::TransportHints hints(parameters_.sonar_transport_,ros::TransportHints(),node_handle_);

        //! image subscription
        image_subscriber_ = it_.subscribe(parameters_.sonar_topic_,1,&ImageProcessing::imageCallback,this,hints);

    }


}


void ImageProcessing::createROSPublishers()
{

    image_publisher_ = it_.advertise(parameters_.image_keypoints_topic_, 1);

    descriptors_publisher_ = node_handle_.advertise<dolphin_slam::Descriptors>(parameters_.descriptors_topic_, 100);

}

void ImageProcessing::createROSServices()
{
    image_service = node_handle_.advertiseService("image_request",&ImageProcessing::imageRequest,this);
}



bool ImageProcessing::init()
{


    if(parameters_.source_ == "camera")
    {
        surf_ = new cv::SURF(parameters_.surf_threshold_);
        gftt_ = new cv::GFTTDetector(800,0.01,15,3,true);
        sift_ = new cv::SIFT();

    }
    else if(parameters_.source_ == "sonar")
    {
        surf_ = new cv::SURF(parameters_.surf_threshold_);

        //        surf_ = new cv::SURF(parameters_.surf_threshold_,1,1);

        sonar_mask_ = cv::imread(parameters_.sonar_mask_,CV_LOAD_IMAGE_GRAYSCALE);

        cv::threshold(sonar_mask_,sonar_mask_,127,255,CV_THRESH_BINARY);

        cv::imwrite("sonar_mask.jpg",sonar_mask_);
    }

    clahe = cv::createCLAHE(2,cv::Size(16,16));

    if(parameters_.use_selected_images_){
        std::ifstream file(parameters_.selected_images_file_.c_str());
        cout << "filename = " << parameters_.selected_images_file_ << endl;

        int id;

        file >> id;

        cout << "ids = ";
        while(file.good())
        {
            selected_images.push_back(id);

            cout << id << " ";
            file >> id;
        }
        cout << endl;

        file.close();




    }


}


void ImageProcessing::publishImageKeypoints()
{
    cv_bridge::CvImage image_keypoints;

    image_keypoints.header.stamp = image_->header.stamp;
    image_keypoints.header.seq = image_->header.seq;
    image_keypoints.header.frame_id = image_->header.frame_id;

    image_keypoints.encoding = sensor_msgs::image_encodings::BGR8;

    cv::drawKeypoints(image_->image,keypoints_,image_keypoints.image);

    image_publisher_.publish(image_keypoints.toImageMsg());

}


void ImageProcessing::publishDescriptors()
{
    Descriptors msg;

    msg.header.stamp = image_->header.stamp;
    msg.header.seq = image_->header.seq;
    msg.header.frame_id = image_->header.frame_id;

    msg.image_seq_ = image_->header.seq;
    msg.image_stamp_ = image_->header.stamp;

    msg.descriptor_count_ = descriptors_.rows;
    msg.descriptor_length_ = descriptors_.cols;
    msg.data_.resize(descriptors_.rows*descriptors_.cols);

    std::copy(descriptors_.begin<float>(),descriptors_.end<float>(),msg.data_.begin());
    descriptors_publisher_.publish(msg);

}


bool ImageProcessing::is_selected_image(int id)
{
    for(int i=0;i<selected_images.size();i++)
    {
        if(id < selected_images[i])
            return false;
        else if (id == selected_images[i])
            return true;
    }

    return false;
}

void ImageProcessing::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    static int count = 0;
    unsigned short int thresh1 = 150;
    unsigned short int thresh2 = 1100;
    unsigned short int pixel;
    float pixel_f;

    cv::Mat sonar_gray;


    if(parameters_.use_selected_images_)
    {
        if(is_selected_image(msg->header.seq))
            count = 0;
        else
            return;
    }

    if (count == 0){
        if(parameters_.source_ == "camera")
        {

            ROS_DEBUG_STREAM("Image received. seq = " << msg->header.seq);
            image_ = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);




            if(parameters_.image_detector_ == "surf")
            {

                //clahe->apply(image_->image,image_->image);

                //! Detect SURF keypoints in the image
                surf_->detect(image_->image,keypoints_);
                //! Compute SURF descriptors
                surf_->compute(image_->image,keypoints_,descriptors_);



            }
            else if (parameters_.image_detector_ == "gftt")
            {

               // clahe->apply(image_->image,image_->image);


                gftt_->detect(image_->image,keypoints_);

                sift_->compute(image_->image,keypoints_,descriptors_);
            }

            ROS_DEBUG_STREAM("Number of SURF keypoints: " << keypoints_.size());

            image_buffer_.push(make_pair(msg->header.seq,image_->image));

            publishDescriptors();

            publishImageKeypoints();

        }
        else if(parameters_.source_ == "sonar")
        {

            image_ = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO16);


            cv::blur( image_->image, image_->image, cv::Size(5,5) );
            cv::medianBlur ( image_->image, image_->image, 5 );
            //cv::GaussianBlur(image_->image,image_->image,cv::Size(15,15),4,4);

            sonar_gray.create(image_->image.rows,image_->image.cols,CV_8U);

            //! threshold and normalize to CV_8U image
            for(int i=0;i<image_->image.rows;i++)
            {
                for(int j=0;j<image_->image.cols;j++)
                {
                    pixel = image_->image.at<unsigned short int>(i,j);
                    if(pixel < thresh1)
                        pixel = thresh1;
                    if(pixel > thresh2)
                        pixel = thresh2;

                    pixel_f = static_cast<float>(pixel - thresh1)/static_cast<float>(thresh2 - thresh1);

                    //! mask
                    if(sonar_mask_.at<unsigned char>(i,j) == 0)
                    {
                        sonar_gray.at<unsigned char>(i,j) = 0;
                    }
                    else
                    {
                        sonar_gray.at<unsigned char>(i,j) = static_cast<unsigned char>(pixel_f*255.0);
                    }
                }
            }

            cv::threshold(sonar_gray,sonar_gray,1,255,CV_THRESH_BINARY);

            image_->image = sonar_gray;
            image_->encoding = sensor_msgs::image_encodings::MONO8;

            computeShapeDescriptors(sonar_gray);

            //            //! Detect SURF keypoints in the image
            //            surf_->detect(image_->image,keypoints_);
            //            //! Compute SURF descriptors
            //            surf_->compute(image_->image,keypoints_,descriptors_);

            //            ROS_DEBUG_STREAM("Number of SURF keypoints" << keypoints_.size());

            //            image_buffer_.push(make_pair(msg->header.seq,image_->image));


            publishImageKeypoints();

        }
    }

    if(parameters_.frames_to_jump_)
        count = (count + 1)%parameters_.frames_to_jump_;
}

bool ImageProcessing::computeShapeDescriptors(cv::Mat &image)
{
    vector<vector<cv::Point> > contour;
    vector<cv::Vec4i> hierarchy;
    cv::Moments mm;
    double hu[7];

    std::vector <std::vector<float> > hu_descriptors;

    int valid_contours = 0;

    cv::Mat image_contour;
    image_contour = image.clone();

    cv::findContours(image_contour, contour,hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    hu_descriptors.resize(contour.size(),std::vector<float>(7));

    keypoints_.resize(contour.size());

    for( int i = 0; i< contour.size(); i++ )
    {
        mm = cv::moments(contour[i]);
        //        float w1 =0 ;
        //        float w2 = 0;
        if ( mm.m00 > 10){
            //           w1= mm.mu20/mm.m00;
            //           w2 = mm.mu02/mm.m00;




            //           keypoints_[i] = cv::KeyPoint(cv::Point(mm.m10/mm.m00,mm.m01/mm.m00),20);
            cv::HuMoments(mm,hu);

            //std::cout << "M = " << mm.m00 << " " << mm.m01 << " " << mm.m10 <<  std::endl;
            //std::cout << "NU = " << mm.nu02 << " " << mm.nu03 << " " << mm.nu11 << " " << mm.nu12 << " " << mm.nu20 << " " << mm.nu21 << " " << mm.nu30 << std::endl;
            //std::cout << "hu = " ;
            for( int j = 0; j < 7; j++ )
            {
                hu_descriptors[valid_contours][j] = boost::numeric_cast<float>(hu[j]);
            }
            valid_contours++;
        }
    }

    descriptors_.create(valid_contours,7,CV_32F);
    for(int i=0;i<descriptors_.rows;i++)
    {
        for(int j=0;j<descriptors_.cols;j++)
        {
            descriptors_.at<float>(i,j) = hu_descriptors[i][j];
        }
    }
    if(descriptors_.rows > 0)
        publishDescriptors();

}

bool ImageProcessing::imageRequest(dolphin_slam::ImageRequest::Request  &req,
                                   dolphin_slam::ImageRequest::Response &res)
{

    cv_bridge::CvImage image_response;
    std::pair <int,cv::Mat> next_element;

    while(!image_buffer_.empty())
    {
        next_element = image_buffer_.front();
        image_buffer_.pop();
        if(next_element.first == req.seq)
        {
            image_response.header.seq = req.seq;
            image_response.encoding = sensor_msgs::image_encodings::MONO8;
            image_response.image = next_element.second;
            image_response.toImageMsg(res.image);
        }
    }

    return true;
}

} //dolphin_slam
