#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>
#include <ctime>

class Backup
{
private:
    // Declare node handler and subscribers
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> sub_1_, sub_2_, sub_3_, sub_4_, sub_5_, sub_6_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    // Package path and backup folder
    static constexpr const char *package = "multispectral_processing";
    static constexpr const char *file_backup = "/data/backup/single_frame/";
    static constexpr const char *file_stream = "/data/backup/multi_frames/";
    std::string path_backup, path_stream;
    bool OPERATION = false;
    int counter = 0;

public:
    Backup()
    {
        // Subscribers
        sub_1_.subscribe(nh_, "/camera/image_raw", 1);
        sub_2_.subscribe(nh_, "/band/3/interpolated/image_raw", 1);
        sub_3_.subscribe(nh_, "/band/ndvi/image_raw", 1);
        sub_4_.subscribe(nh_, "/band/ndvi_colored/image_raw", 1);
        sub_5_.subscribe(nh_, "/kinect2/hd/image_color_rect", 1);
        sub_6_.subscribe(nh_, "/kinect2/hd/image_depth_rect", 1);
        // Synchronizer policy
        sync_.reset(new Sync(MySyncPolicy(10), sub_1_, sub_2_, sub_3_, sub_4_, sub_5_, sub_6_));
        sync_->registerCallback(boost::bind(&Backup::callback, this, _1, _2, _3, _4, _5, _6));
        // Get package path
        path_backup = ros::package::getPath(package) + file_backup;
        path_stream = ros::package::getPath(package) + file_stream;
        // Display info
        ROS_INFO("Backup node (cpp) started");
        std::cout << "Press esc to save single frame, S or s to save stream of frames." << std::endl;
        std::cout << "Files are saved in " + path_backup + " and " + path_stream + " folders." << std::endl;
    }

    void callback(const sensor_msgs::ImageConstPtr &in1, const sensor_msgs::ImageConstPtr &in2, const sensor_msgs::ImageConstPtr &in3, const sensor_msgs::ImageConstPtr &in4, const sensor_msgs::ImageConstPtr &in5, const sensor_msgs::ImageConstPtr &in6)
    {
        // Convert from ROS to OpenCV
        cv_bridge::CvImagePtr im_ptr1, im_ptr2, im_ptr3, im_ptr4, im_ptr5, im_ptr6;
        try
        {
            // If encoding is the empty string (the default), the returned CvImage has the same encoding as source.
            im_ptr1 = cv_bridge::toCvCopy(in1);
            im_ptr2 = cv_bridge::toCvCopy(in2);
            im_ptr3 = cv_bridge::toCvCopy(in3);
            im_ptr4 = cv_bridge::toCvCopy(in4);
            im_ptr5 = cv_bridge::toCvCopy(in5);
            im_ptr6 = cv_bridge::toCvCopy(in6);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // Image message to opencv image
        cv::Mat img_1 = im_ptr1->image;
        cv::Mat img_2 = im_ptr2->image;
        cv::Mat img_3 = im_ptr3->image;
        cv::Mat img_4 = im_ptr4->image;
        cv::Mat img_5 = im_ptr5->image;
        cv::Mat img_6 = im_ptr6->image;

        // Display images
        cv::namedWindow("Image 1", cv::WINDOW_NORMAL);
        cv::resizeWindow("Image 1", 600, 600);
        cv::imshow("Image 1", img_1);

        cv::namedWindow("Image 2", cv::WINDOW_NORMAL);
        cv::resizeWindow("Image 2", 600, 600);
        cv::imshow("Image 2", img_2);

        cv::namedWindow("Image 3", cv::WINDOW_NORMAL);
        cv::resizeWindow("Image 3", 600, 600);
        cv::imshow("Image 3", img_3);

        cv::namedWindow("Image 4", cv::WINDOW_NORMAL);
        cv::resizeWindow("Image 4", 600, 600);
        cv::imshow("Image 4", img_4);

        cv::namedWindow("Image 5", cv::WINDOW_NORMAL);
        cv::resizeWindow("Image 5", 600, 600);
        cv::imshow("Image 5", img_5);

        cv::namedWindow("Image 6", cv::WINDOW_NORMAL);
        cv::resizeWindow("Image 6", 600, 600);
        cv::imshow("Image 6", img_6);

        // Key listener
        // Save single frame with <esc> key, save stream of frames with <s, S> key
        int key = cv::waitKey(3);
        if (key == 27)
        {
            OPERATION = false;
            backupImages(img_1, img_2, img_3, img_4, img_5, img_6, path_backup);
            ROS_INFO_STREAM("Single frame saved.");
        }
        else if (key == 83 || key == 115)
        {
            // Enable or disable the stream saving procedure
            if (OPERATION)
            {
                ROS_INFO_STREAM("Saving stream of frames disabled.");
                OPERATION = false;
            }
            else
            {
                ROS_INFO_STREAM("Saving stream of frames enabled.");
                OPERATION = true;
            }
        }
        if (OPERATION)
        {
            backupImages(img_1, img_2, img_3, img_4, img_5, img_6, path_stream);
        }
    }

    // Save the backup of the images
    void backupImages(cv::Mat &img_1, cv::Mat &img_2, cv::Mat &img_3, cv::Mat &img_4, cv::Mat &img_5, cv::Mat &img_6, std::string path)
    {
        // String streams init
        std::stringstream s1, s2, s3, s4, s5, s6;
        // Current date/time based on current system
        std::time_t now = std::time(0);
        std::tm *ltm = std::localtime(&now);
        // Titles of images
        if (OPERATION)
        {
            s1 << path << counter << "_multispectral_camera.png";
            s2 << path << counter << "_band3_interpolated.png";
            s3 << path << counter << "_multispectral_ndvi.png";
            s4 << path << counter << "_multispectral_ndvi_colored.png";
            s5 << path << counter << "_kinect_hd_rgb.png";
            s6 << path << counter << "_kinect_hd_depth.png";
            counter++;
        }
        else
        {
            s1 << path << 1900 + ltm->tm_year << 1 + ltm->tm_mon << ltm->tm_mday << "_multispectral_camera.png";
            s2 << path << 1900 + ltm->tm_year << 1 + ltm->tm_mon << ltm->tm_mday << "_band3_interpolated.png";
            s3 << path << 1900 + ltm->tm_year << 1 + ltm->tm_mon << ltm->tm_mday << "_multispectral_ndvi.png";
            s4 << path << 1900 + ltm->tm_year << 1 + ltm->tm_mon << ltm->tm_mday << "_multispectral_ndvi_colored.png";
            s5 << path << 1900 + ltm->tm_year << 1 + ltm->tm_mon << ltm->tm_mday << "_kinect_hd_rgb.png";
            s6 << path << 1900 + ltm->tm_year << 1 + ltm->tm_mon << ltm->tm_mday << "_kinect_hd_depth.png";
        }
        // Save images
        try
        {
            cv::imwrite(s1.str(), img_1);
            cv::imwrite(s2.str(), img_2);
            cv::imwrite(s3.str(), img_3);
            cv::imwrite(s4.str(), img_4);
            cv::imwrite(s5.str(), img_5);
            cv::imwrite(s6.str(), img_6);
        }
        catch (cv::Exception &e)
        {
            ROS_ERROR("Unable to save the images. The node will be shutted down.");
            ros::shutdown();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "backup");
    Backup backup;
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    ros::shutdown();
    std::cout << "Backup node shutted down" << std::endl;
    return 0;
}