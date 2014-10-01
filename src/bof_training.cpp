
#include<bag_of_features.h>
#include<ros/ros.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <opencv/highgui.h>
#include <opencv/cv.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include <boost/lexical_cast.hpp>



struct Parameters
{
    int surf_threshold_;
    int bof_group_;
    int frames_to_jump_;
    string image_topic_;
    string bof_vocabulary_path_;
    string bag_file;
};

void load_parameters(Parameters &parameters)
{
    ros::NodeHandle private_nh_("~");

    //! int surf_threshold;
    private_nh_.param<int>("surf_threshold",parameters.surf_threshold_,100);

    //! int bof_groups;
    private_nh_.param<int>("bof_groups",parameters.bof_group_,100);

    //! implementar isto para treinar varios grupos
//    private_nh.getParam("bof_groups",parameters.bof_groups_);

//    private_nh.getParam("hessian_thresholds",parameters.hessian_thresholds_);

    //! int bof_groups;
    private_nh_.param<int>("frames_to_jump",parameters.frames_to_jump_,10);

    //! string image_topic;
    private_nh_.param<string>("image_topic",parameters.image_topic_,"/image");

    //! string bof_dictionary_path
    private_nh_.param<string>("bof_vocabulary_path",parameters.bof_vocabulary_path_,"/home/lsilveira/Codigos/indigo_ws/src/dolphin_slam/bof_vocabularies/");

    //! string bag_file;
    private_nh_.param<string>("bag_file",parameters.bag_file,"/mnt/Dados/Dropbox/Mestrado/Datasets/Ballester/girona_rotated.bag");

}

int main(int argc, char **argv){

    Parameters parameters;
    ros::init(argc, argv, "bof_training");

    cv::FileStorage fs;
    int count = 0;
    string filename;


    load_parameters(parameters);

    dolphin_slam::BoF bag_of_features(parameters.bof_group_,parameters.surf_threshold_);

    cv_bridge::CvImageConstPtr cv_image_ptr;

    rosbag::Bag bag;

    try {
        cout << "Opening bag file " << parameters.bag_file << endl;
        bag.open(parameters.bag_file, rosbag::bagmode::Read);
        cout << "File succesfully opened." << endl;
    } catch (rosbag::BagIOException exception) {
        cout << "Bag IO exception: " << exception.what() << endl;
        exit(1);
    }


    std::vector<std::string> topics;
    topics.push_back(parameters.image_topic_);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    cout << "Start Bagfile reading" << endl;

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        if(count == 0){

            sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();

            cv_image_ptr = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::MONO8);

            bag_of_features.addTrainingImage(cv_image_ptr->image);

        }

        count = (count+1) % parameters.frames_to_jump_;

    }

    bag.close();

    cout << "Finish Bagfile reading" << endl;
    cout << "Start training" << endl;
    cout << "Features = " << bag_of_features.getThreshold() << " Groups = " << bag_of_features.getGroups() << endl;

    bag_of_features.train();

    cout << "Trainning complete" << endl;

    //! Save vocabulary
    filename = parameters.bof_vocabulary_path_ +
            string("vocabulary_s") + boost::lexical_cast<string>(parameters.surf_threshold_) +
            string("_g") + boost::lexical_cast<string>(parameters.bof_group_) + string(".xml");

    cout << "Vocabulary: " << filename << endl;

    fs.open(filename,cv::FileStorage::WRITE);
    fs << "BoF" << bag_of_features;
    fs.release();

    //! Sort vocabulary
    bag_of_features.sortVocabulary();

    //! Save sorted vocabulary
    filename = parameters.bof_vocabulary_path_ +
            string("sorted/vocabulary_s") + boost::lexical_cast<string>(parameters.surf_threshold_) +
            string("_g") + boost::lexical_cast<string>(parameters.bof_group_) + string(".xml");

    cout << "Sorted vocabulary: " << filename << endl;

    fs.open(filename,cv::FileStorage::WRITE);
    fs << "BoF" << bag_of_features;
    fs.release();


}
