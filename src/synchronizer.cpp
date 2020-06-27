// Experimental node for synchronization and stereo image processing

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class Synchronizer
{
private:
    // Declare node handler, subscriber and publishers
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> sub_1_, sub_2_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_1_i, sub_2_i;
    ros::Publisher pub_1_, pub_2_, pub_1_i, pub_2_i;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    // Declare camera info and header messages
    sensor_msgs::CameraInfo M_CAMERA_INFO, K_CAMERA_INFO;
    // Global variables
    static const short BAND_WIDTH = 960;
    static const short BAND_HEIGHT = 540;
    static constexpr const char *D_MODEL = "plumb_bob";
    static constexpr const char *FRAME_ID = "stereo_frame";
    static constexpr const char *IME_B = "mono8";

public:
    Synchronizer()
    {
        // Subscribers
        sub_1_.subscribe(nh_, "/band/1/image_raw", 1);
        sub_2_.subscribe(nh_, "/kinect2/qhd/image_color_rect", 1);
        sub_1_i.subscribe(nh_, "/band/camera_info", 1);
        sub_2_i.subscribe(nh_, "/kinect2/qhd/camera_info", 1);
        // Publishers
        pub_1_ = nh_.advertise<sensor_msgs::Image>("/stereo/right/image_raw", 1);
        pub_2_ = nh_.advertise<sensor_msgs::Image>("/stereo/left/image_raw", 1);
        pub_1_i = nh_.advertise<sensor_msgs::CameraInfo>("/stereo/right/camera_info", 1);
        pub_2_i = nh_.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info", 1);
        // Synchronizer policy
        sync_.reset(new Sync(MySyncPolicy(10), sub_1_, sub_2_, sub_1_i, sub_2_i));
        sync_->registerCallback(boost::bind(&Synchronizer::callback, this, _1, _2, _3, _4));
        // Display info
        ROS_INFO("Synchronizer node started");
        // Init camera info
        M_CAMERA_INFO.header.seq = 0;
        M_CAMERA_INFO.header.frame_id = FRAME_ID;
        M_CAMERA_INFO.width = BAND_WIDTH;
        M_CAMERA_INFO.height = BAND_HEIGHT;
        M_CAMERA_INFO.distortion_model = D_MODEL;

        K_CAMERA_INFO.header.frame_id = FRAME_ID;
        K_CAMERA_INFO.header.seq = 0;
        K_CAMERA_INFO.width = BAND_WIDTH;
        K_CAMERA_INFO.height = BAND_HEIGHT;
        K_CAMERA_INFO.distortion_model = D_MODEL;
    }

    void callback(const sensor_msgs::ImageConstPtr &in1, const sensor_msgs::ImageConstPtr &in2, const sensor_msgs::CameraInfoConstPtr &in3, const sensor_msgs::CameraInfoConstPtr &in4)
    {
        // Set to the camera info the time stamp
        M_CAMERA_INFO.header.stamp = ros::Time::now();
        K_CAMERA_INFO.header.stamp = ros::Time::now();

        // Convert from ROS to OpenCV
        cv_bridge::CvImagePtr im_ptr1, im_ptr2;
        try
        {
            im_ptr1 = cv_bridge::toCvCopy(in1, sensor_msgs::image_encodings::MONO8);
            im_ptr2 = cv_bridge::toCvCopy(in2, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // Image message to opencv image
        cv::Mat img_m = im_ptr1->image;
        cv::Mat img_k = im_ptr2->image;

        // Set camera info
        M_CAMERA_INFO.D = in3->D;
        M_CAMERA_INFO.K = in3->K;
        M_CAMERA_INFO.R = in3->R;
        M_CAMERA_INFO.P = in3->P;

        K_CAMERA_INFO.D = in4->D;
        K_CAMERA_INFO.K = in4->K;
        K_CAMERA_INFO.R = in4->R;
        K_CAMERA_INFO.P = in4->P;

        // Resize image
        cv::Mat img_rm;
        resize(img_m, img_rm);

        // Convert to image message
        sensor_msgs::ImagePtr imgCVM = cv_bridge::CvImage(M_CAMERA_INFO.header, IME_B, img_rm).toImageMsg();
        sensor_msgs::ImagePtr imgCVK = cv_bridge::CvImage(K_CAMERA_INFO.header, IME_B, img_k).toImageMsg();

        // Publish images
        pub_1_.publish(imgCVM);
        pub_2_.publish(imgCVK);
        pub_1_i.publish(M_CAMERA_INFO);
        pub_2_i.publish(K_CAMERA_INFO);
    }

    // Resize multispectral image to fit
    void resize(cv::Mat &img_m, cv::Mat &result)
    {
        // Bicubic Interpolation for reseizing
        cv::Mat img_r = cv::Mat(764, 960, CV_8UC1);
        cv::resize(img_m, img_r, img_r.size(), 0, 0, cv::INTER_CUBIC);
        // Crop images to fit with depth images 960x540
        img_r(cv::Rect(cv::Point(0, 652), cv::Point(960, 112))).copyTo(result);
    }

    // Shut down everything
    void stop()
    {
        pub_1_.shutdown();
        pub_2_.shutdown();
        pub_1_i.shutdown();
        pub_2_i.shutdown();
        ros::shutdown();
        std::cout << "Synchronizer node shutted down" << std::endl;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "synchronizer");
    Synchronizer synchronizer;
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    synchronizer.stop();
    return 0;
}