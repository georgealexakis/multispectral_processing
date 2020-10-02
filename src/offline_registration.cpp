// Experiments node for publishing images of offline image registration

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offline_registration");
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_(nh_);
    image_transport::Publisher pub_1_ = it_.advertise("/band/3/interpolated/image_raw", 1);
    image_transport::Publisher pub_2_ = it_.advertise("/kinect2/hd/image_color_rect", 1);
    image_transport::Publisher pub_3_ = it_.advertise("/kinect2/hd/image_depth_rect", 1);
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
    // Kinect and multispectral simultation images
    sensor_msgs::ImagePtr muImgCVB;
    sensor_msgs::ImagePtr krImgCVB;
    sensor_msgs::ImagePtr kdImgCVB;
    if ((argument >= 0) && (argument < 5))
    {
        std::stringstream s1, s2, s3;
        s1 << argument << "/" << prefix << "_band3_interpolated.png";
        s2 << argument << "/" << prefix << "_kinect_hd_rgb.png";
        s3 << argument << "/" << prefix << "_kinect_hd_depth.png";
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
    ROS_INFO("\n\n******** Offline registration node started ***********\n");
    std::cout << "Experiment case id: " << argument << std::endl;
    while (ros::ok())
    {
        pub_1_.publish(muImgCVB);
        pub_2_.publish(krImgCVB);
        pub_3_.publish(kdImgCVB);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("\n\n******** Offline registration node has terminated ***********\n");
    return 0;
}