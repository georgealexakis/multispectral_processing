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
    image_transport::Publisher pub_1_ = it_.advertise("/camera/image_raw", 1);
    image_transport::Publisher pub_2_ = it_.advertise("/kinect2/hd/image_color_rect", 1);
    image_transport::Publisher pub_3_ = it_.advertise("/kinect2/hd/image_depth_rect", 1);
    ros::Rate loop_rate(10);
    // Paths to images data
    const std::string pPath = "multispectral_processing";
    const std::string oPath = "/data/others/";
    const std::string sPath = "/data/simulation/";
    // Input arguments
    short argument = 0;
    if (argc > 1)
    {
        argument = (short)*argv[1] - 48;
        std::cout << argument << std::endl;
    }
    // Basic images full of ones or zeros, args options 0, 1
    cv::Mat expImg1 = cv::Mat::ones(cv::Size(1280, 1024), CV_8UC1);
    cv::Mat expImg2 = cv::Mat::zeros(cv::Size(1280, 1024), CV_8UC1);
    sensor_msgs::ImagePtr imgCVB1 = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, expImg1).toImageMsg();
    sensor_msgs::ImagePtr imgCVB2 = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, expImg2).toImageMsg();
    // Multiple images captured from multispectral camera, args options 2, 3, 4, 5, 6
    std::string p3 = ros::package::getPath(pPath) + oPath + "b1.png";
    cv::Mat expImg3 = cv::imread(p3);
    std::string p4 = ros::package::getPath(pPath) + oPath + "b2.png";
    cv::Mat expImg4 = cv::imread(p4);
    std::string p5 = ros::package::getPath(pPath) + oPath + "b3.png";
    cv::Mat expImg5 = cv::imread(p5);
    std::string p6 = ros::package::getPath(pPath) + oPath + "b4.png";
    cv::Mat expImg6 = cv::imread(p6);
    std::string p7 = ros::package::getPath(pPath) + oPath + "b5.bmp";
    cv::Mat expImg7 = cv::imread(p7);
    sensor_msgs::ImagePtr imgCVB3 = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, expImg3).toImageMsg();
    sensor_msgs::ImagePtr imgCVB4 = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, expImg4).toImageMsg();
    sensor_msgs::ImagePtr imgCVB5 = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, expImg5).toImageMsg();
    sensor_msgs::ImagePtr imgCVB6 = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, expImg6).toImageMsg();
    sensor_msgs::ImagePtr imgCVB7 = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, expImg7).toImageMsg();
    // Kinect and multispectral simultation images, args options 7, 8, 9, 10
    sensor_msgs::ImagePtr muImgCVB;
    sensor_msgs::ImagePtr krImgCVB;
    sensor_msgs::ImagePtr kdImgCVB;
    if ((argument > 6) && (argument < 11))
    {
        std::stringstream s1, s2, s3;
        s1 << (argument - 6) << "/2020511_multispectral_camera.png";
        s2 << (argument - 6) << "/2020511_kinect_hd_rgb.png";
        s3 << (argument - 6) << "/2020511_kinect_hd_depth.png";
        std::string mup = ros::package::getPath(pPath) + sPath + s1.str();
        std::string krp = ros::package::getPath(pPath) + sPath + s2.str();
        std::string kdp = ros::package::getPath(pPath) + sPath + s3.str();
        cv::Mat muImg = cv::imread(mup);
        cv::Mat krImg = cv::imread(krp);
        cv::Mat kdImg = cv::imread(kdp);
        muImgCVB = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, muImg).toImageMsg();
        krImgCVB = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, krImg).toImageMsg();
        kdImgCVB = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, kdImg).toImageMsg();
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
        else if (argument == 2)
        {
            pub_1_.publish(imgCVB3);
        }
        else if (argument == 3)
        {
            pub_1_.publish(imgCVB4);
        }
        else if (argument == 4)
        {
            pub_1_.publish(imgCVB5);
        }
        else if (argument == 5)
        {
            pub_1_.publish(imgCVB6);
        }
        else if (argument == 6)
        {
            pub_1_.publish(imgCVB7);
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