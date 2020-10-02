// Node for experiments. It publishes experimental images to the specific topic (useful when working without camera)
// Test image with ones
// Test image with zeros
// Test aquired image from multispectral camera (see data folder)

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "experiments");
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_(nh_);
    image_transport::Publisher pub_1_ = it_.advertise("/camera/image_raw", 10);
    image_transport::Publisher pub_2_ = it_.advertise("/kinect2/hd/image_color_rect", 10);
    image_transport::Publisher pub_3_ = it_.advertise("/kinect2/hd/image_depth_rect", 10);
    ros::Rate loop_rate(10);
    // Paths to images data
    const std::string pPath = "multispectral_processing";
    const std::string sPath = "/data/simulation/";
    // Input arguments
    short argument = 0;
    std::string prefix;
    if (argc > 1)
    {
        argument = (short)*argv[1] - 48;
        prefix = argv[2];
    }
    // Basic images full of ones or zeros, args options 0, 1
    cv::Mat expImg1 = cv::Mat::ones(cv::Size(1280, 1024), CV_8UC1);
    cv::Mat expImg2 = cv::Mat::zeros(cv::Size(1280, 1024), CV_8UC1);
    sensor_msgs::ImagePtr imgCVB1 = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, expImg1).toImageMsg();
    sensor_msgs::ImagePtr imgCVB2 = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, expImg2).toImageMsg();
    // Kinect and multispectral simultation images, args options 2, 3, 4, 5
    sensor_msgs::ImagePtr muImgCVB;
    sensor_msgs::ImagePtr krImgCVB;
    sensor_msgs::ImagePtr kdImgCVB;
    if ((argument > 1) && (argument < 6))
    {
        std::stringstream s1, s2, s3;
        s1 << (argument - 1) << "/" << prefix << "_multispectral_camera.png";
        s2 << (argument - 1) << "/" << prefix << "_kinect_hd_rgb.png";
        s3 << (argument - 1) << "/" << prefix << "_kinect_hd_depth.png";
        std::string mup = ros::package::getPath(pPath) + sPath + s1.str();
        std::string krp = ros::package::getPath(pPath) + sPath + s2.str();
        std::string kdp = ros::package::getPath(pPath) + sPath + s3.str();
        cv::Mat muImg = cv::imread(mup);
        cv::Mat krImg = cv::imread(krp);
        cv::Mat kdImg = cv::imread(kdp);
        muImgCVB = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, muImg).toImageMsg();
        krImgCVB = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, krImg).toImageMsg();
        kdImgCVB = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, kdImg).toImageMsg();
    }
    else
    {
        argument = 0;
    }
    // Dispaly message
    ROS_INFO("\n\n******** Experiments node started ***********\n");
    std::cout << "Experiment case id: " << argument << std::endl;
    while (ros::ok())
    {
        if (argument == 0)
        {
            pub_1_.publish(imgCVB1);
        }
        else if (argument == 1)
        {
            pub_1_.publish(imgCVB2);
        }
        else
        {
            pub_1_.publish(muImgCVB);
            pub_2_.publish(krImgCVB);
            pub_3_.publish(kdImgCVB);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("\n\n******** Experiments node has terminated ***********\n");
    return 0;
}