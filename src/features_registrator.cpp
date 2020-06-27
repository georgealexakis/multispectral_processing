#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <ros/package.h>

class FeaturesRegistrator
{
private:
    // Declare node handler, subscribers and publishers
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> sub_1_, sub_2_, sub_3_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_1_, pub_2_, pub_3_;
    ros::Publisher pub_4_;
    // Declare camera info and header messages
    sensor_msgs::CameraInfo M_CAMERA_INFO;
    // Approximate time sync policy
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    // Image parameters
    static const short RAW_WIDTH = 1278;
    static const short RAW_HEIGHT = 1017;
    static constexpr const char *D_MODEL = "plumb_bob";
    static constexpr const char *FRAME_ID = "multispectral_frame";
    static constexpr const char *IME_B = "mono8";
    static constexpr const char *IME_C = "bgr8";
    static constexpr const char *package = "multispectral_processing";
    static constexpr const char *file = "/resources/homography1.yaml";
    cv::Mat HOMOGRAPHY_MATRIX;
    // Number of matches
    const short MAX_FEATURES = 1000;
    // Lower is more accurate. Higher take more features
    const float GOOD_MATCH_PERCENT = 0.15f;
    // Node operation capturing or not
    bool OPERATION = false;
    std::string path;
    // Capture best homography matrix
    cv::Mat BEST_HOMOGRAPHY;
    short MAX_MATCHES = 0;
    double MIN_DIFF = 255.0;

public:
    FeaturesRegistrator(int argc, char **argv) : it_(nh_)
    {
        // Subcribers
        sub_1_.subscribe(nh_, "/band/3/interpolated/image_raw", 1);
        sub_2_.subscribe(nh_, "/kinect2/hd/image_color_rect", 1);
        sub_3_.subscribe(nh_, "/kinect2/hd/image_depth_rect", 1);
        // Publishers
        pub_1_ = it_.advertise("/multispectral/image_color", 1);
        pub_2_ = it_.advertise("/multispectral/image_mono", 1);
        pub_3_ = it_.advertise("/multispectral/image_depth", 1);
        pub_4_ = nh_.advertise<sensor_msgs::CameraInfo>("/multispectral/camera_info", 1);
        sync_.reset(new Sync(MySyncPolicy(10), sub_1_, sub_2_, sub_3_));
        sync_->registerCallback(boost::bind(&FeaturesRegistrator::callback, this, _1, _2, _3));
        // Get package path
        path = ros::package::getPath(package) + file;
        // Read camera parameters
        readCameraParameters(nh_);
        // Display info
        ROS_INFO("Feature matcher node started");
        if ((argc > 1) && (((std::string("capture")).compare(argv[1])) == 0))
        {
            OPERATION = true;
            ROS_INFO("Perspective transformation matrix capturing mode started.");
        }
        else
        {
            ROS_INFO("Publishing mode started.");
        }
    }

    ~FeaturesRegistrator()
    {
    }

    void callback(const sensor_msgs::ImageConstPtr &im1, const sensor_msgs::ImageConstPtr &im2, const sensor_msgs::ImageConstPtr &im3)
    {
        // Modify ROS image to OpenCV image for further processing
        cv_bridge::CvImagePtr im1_ptr, im2_ptr, im3_ptr;
        try
        {
            im1_ptr = cv_bridge::toCvCopy(im1, IME_B);
            im2_ptr = cv_bridge::toCvCopy(im2, IME_C);
            im3_ptr = cv_bridge::toCvCopy(im3, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat img1 = im1_ptr->image;
        cv::Mat img2BGR = im2_ptr->image;
        cv::Mat img3 = im3_ptr->image;
        // BGR channel splitting and acquire only RED channel
        cv::Mat channel[3];
        split(img2BGR, channel);
        cv::Mat img2;
        channel[2].copyTo(img2);
        // Key listener
        short key = cv::waitKey(3);
        // Operations
        if (OPERATION)
        {
            // Reset captures with r or R
            if ((key == 114) || (key == 82))
            {
                BEST_HOMOGRAPHY = cv::Mat();
                MAX_MATCHES = 0;
                MIN_DIFF = 255;
            }
            // The estimated homography will be calculated
            cv::Mat h, r1, r2, r3, r4;
            cv::Mat vis1 = cv::Mat::zeros(RAW_HEIGHT, 2 * RAW_WIDTH, CV_8UC3);
            cv::Mat vis2 = cv::Mat::zeros(RAW_HEIGHT, 2 * RAW_WIDTH, CV_16UC1);
            computeHomographyFeatures(img1, img2, h);
            // Check if homography matrix exists
            if (h.empty())
            {
                r1 = cv::Mat::zeros(RAW_HEIGHT, RAW_WIDTH, CV_8UC3);
                r2 = cv::Mat::zeros(RAW_HEIGHT, RAW_WIDTH, CV_8UC1);
            }
            else
            {
                cv::warpPerspective(img2BGR, r1, h, img1.size());
                cv::warpPerspective(img3, r2, h, img1.size());
            }
            if (BEST_HOMOGRAPHY.empty())
            {
                r3 = cv::Mat::zeros(RAW_HEIGHT, RAW_WIDTH, CV_8UC3);
                r4 = cv::Mat::zeros(RAW_HEIGHT, RAW_WIDTH, CV_8UC1);
            }
            else
            {
                cv::warpPerspective(img2BGR, r3, BEST_HOMOGRAPHY, img1.size());
                cv::warpPerspective(img3, r4, BEST_HOMOGRAPHY, img1.size());
                // Press esc to save matrix
                if (key == 27)
                {
                    saveHomography();
                }
            }
            r1.copyTo(vis1(cv::Rect(0, 0, RAW_WIDTH, RAW_HEIGHT)));
            r3.copyTo(vis1(cv::Rect(RAW_WIDTH, 0, RAW_WIDTH, RAW_HEIGHT)));
            r2.copyTo(vis2(cv::Rect(0, 0, RAW_WIDTH, RAW_HEIGHT)));
            r4.copyTo(vis2(cv::Rect(RAW_WIDTH, 0, RAW_WIDTH, RAW_HEIGHT)));
            // Display aligned images
            cv::namedWindow("Aligned kinect RGB image (Real time - Best result)", cv::WINDOW_NORMAL);
            cv::resizeWindow("Aligned kinect RGB image (Real time - Best result)", 600, 600);
            cv::imshow("Aligned kinect RGB image (Real time - Best result)", vis1);

            cv::namedWindow("Aligned kinect depth image (Real time - Best result)", cv::WINDOW_NORMAL);
            cv::resizeWindow("Aligned kinect depth image (Real time - Best result)", 600, 600);
            cv::imshow("Aligned kinect depth image (Real time - Best result)", vis2);
        }
        else
        {
            // Applies perspective transformation to the images
            cv::Mat depth, rgb;
            cv::warpPerspective(img3, depth, HOMOGRAPHY_MATRIX, img1.size());
            cv::warpPerspective(img2BGR, rgb, HOMOGRAPHY_MATRIX, img1.size());
            // Publish topics
            publishImages(img1, depth, rgb);
        }
    }

    // Find homography matrix with feature matching. More: https://docs.opencv.org/master/dc/dc3/tutorial_py_matcher.html
    void computeHomographyFeatures(cv::Mat &img1, cv::Mat &img2, cv::Mat &h)
    {
        // Variables to store keypoints and descriptors
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        // Detect ORB features and compute descriptors
        cv::Ptr<cv::Feature2D> orb = cv::ORB::create(MAX_FEATURES);
        orb->detectAndCompute(img1, cv::Mat(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::Mat(), keypoints2, descriptors2);

        // Match features with Brute-Force Matcher
        std::vector<cv::DMatch> matches;
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
        matcher->match(descriptors1, descriptors2, matches, cv::Mat());

        // Sort matches by DMatch.distance (Distance between descriptors. The lower, the better it is.)
        std::sort(matches.begin(), matches.end());

        // Remove not so good matches
        const short numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
        matches.erase(matches.begin() + numGoodMatches, matches.end());

        // Draw top matches 1/2
        std::stringstream kp1, kp2, nm, mm, md, df;
        kp1 << "Detected features (left image): " << keypoints1.size();
        kp2 << "Detected features (right image): " << keypoints2.size();
        mm << "Matches with min difference: " << MAX_MATCHES;
        md << "Min captured difference: " << MIN_DIFF;
        cv::Mat imMatches;
        cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, imMatches);
        cv::putText(imMatches, kp1.str(), cv::Point2f(10, 800), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2, CV_AA);
        cv::putText(imMatches, kp2.str(), cv::Point2f(10, 850), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2, CV_AA);
        // Set a threshold of acceptable number of features
        if (matches.size() >= (MAX_FEATURES * 0.04))
        {
            // Draw top matches 2/2
            nm << "Matches between the images: " << matches.size();
            cv::putText(imMatches, nm.str(), cv::Point2f(10, 900), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2, CV_AA);
            cv::putText(imMatches, mm.str(), cv::Point2f(10, 950), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2, CV_AA);
            cv::putText(imMatches, md.str(), cv::Point2f(10, 1000), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2, CV_AA);
            cv::namedWindow("Matches", cv::WINDOW_NORMAL);
            cv::resizeWindow("Matches", 600, 600);
            cv::imshow("Matches", imMatches);

            // Extract location of good matches
            std::vector<cv::Point2f> points1, points2;
            for (size_t i = 0; i < matches.size(); ++i)
            {
                points1.push_back(keypoints1[matches[i].queryIdx].pt);
                points2.push_back(keypoints2[matches[i].trainIdx].pt);
            }

            // Find homography with RANSAC method (finds a perspective transformation between two planes)
            if (!points1.empty() && !points2.empty())
            {
                cv::Mat tempH = cv::findHomography(points2, points1, cv::RANSAC);
                // Compute difference between images and the mean of pixels
                cv::Mat img2New;
                cv::warpPerspective(img2, img2New, tempH, img1.size());
                cv::Mat subResult;
                cv::subtract(img1, img2New, subResult);
                cv::Scalar tempVal = cv::mean(subResult);
                double meanResult = tempVal.val[0];
                // Keep the homography matrix with the most matches and minimun difference
                if (meanResult < MIN_DIFF)
                {
                    MAX_MATCHES = numGoodMatches;
                    MIN_DIFF = meanResult;
                    tempH.copyTo(BEST_HOMOGRAPHY);
                }
                df << "Difference: " << meanResult;
                cv::putText(subResult, df.str(), cv::Point2f(10, 1000), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2, CV_AA);
                cv::namedWindow("Difference between multispectral image and kinect image", cv::WINDOW_NORMAL);
                cv::resizeWindow("Difference between multispectral image and kinect image", 600, 600);
                cv::imshow("Difference between multispectral image and kinect image", subResult);
                if (meanResult < 20)
                {
                    tempH.copyTo(h);
                }
            }
            else
            {
                // Clear differences display
                cv::Mat subResult = cv::Mat::zeros(RAW_HEIGHT, RAW_WIDTH, CV_8UC1);
                subResult.setTo(cv::Scalar(255, 255, 255));
                cv::namedWindow("Difference between multispectral image and kinect image", cv::WINDOW_NORMAL);
                cv::resizeWindow("Difference between multispectral image and kinect image", 600, 600);
                cv::imshow("Difference between multispectral image and kinect image", subResult);
            }
        }
        else
        {
            // Draw top matches 2/2
            nm << "Matches between the images: " << matches.size() << " (not enough matches)";
            cv::putText(imMatches, nm.str(), cv::Point2f(10, 900), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2, CV_AA);
            cv::putText(imMatches, mm.str(), cv::Point2f(10, 950), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2, CV_AA);
            cv::putText(imMatches, md.str(), cv::Point2f(10, 1000), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2, CV_AA);
            cv::namedWindow("Matches", cv::WINDOW_NORMAL);
            cv::resizeWindow("Matches", 600, 600);
            cv::imshow("Matches", imMatches);
            // Clear differences display
            cv::Mat subResult = cv::Mat::zeros(RAW_HEIGHT, RAW_WIDTH, CV_8UC1);
            subResult.setTo(cv::Scalar(255, 255, 255));
            cv::namedWindow("Difference between multispectral image and kinect image", cv::WINDOW_NORMAL);
            cv::resizeWindow("Difference between multispectral image and kinect image", 600, 600);
            cv::imshow("Difference between multispectral image and kinect image", subResult);
        }
    }

    // Print and save matrix
    void saveHomography()
    {
        if (!BEST_HOMOGRAPHY.empty())
        {
            std::cout << "-------------Homography matrix saved to homography1.yaml-------------" << std::endl;
            std::cout << BEST_HOMOGRAPHY << std::endl;
            cv::FileStorage fs(path, cv::FileStorage::WRITE);
            fs << "homographyMatrix" << BEST_HOMOGRAPHY;
            fs.release();
            std::cout << "----------------------End---------------------------" << std::endl;
        }
        else
        {
            std::cout << "No homography matrix is available." << std::endl;
        }
    }

    void publishImages(cv::Mat img1, cv::Mat img2, cv::Mat img3)
    {
        // Transform images from OpenCV format to ROS
        M_CAMERA_INFO.header.stamp = ros::Time::now();
        // Header for multispectral image message
        // Color
        sensor_msgs::ImagePtr pim_c = cv_bridge::CvImage(M_CAMERA_INFO.header, IME_C, img3).toImageMsg();
        // Mono
        sensor_msgs::ImagePtr pim_m = cv_bridge::CvImage(M_CAMERA_INFO.header, IME_B, img1).toImageMsg();
        // Depth
        sensor_msgs::ImagePtr pim_d = cv_bridge::CvImage(M_CAMERA_INFO.header, sensor_msgs::image_encodings::TYPE_16UC1, img2).toImageMsg();
        // Publish image topics and camera info topic
        pub_1_.publish(pim_c);
        pub_2_.publish(pim_m);
        pub_3_.publish(pim_d);
        pub_4_.publish(M_CAMERA_INFO);
    }

    // Read camera parameters and create parameters for camera_info
    void readCameraParameters(ros::NodeHandle nh_)
    {
        // Multispectral camera parameters
        M_CAMERA_INFO.header.frame_id = FRAME_ID;
        M_CAMERA_INFO.width = RAW_WIDTH;
        M_CAMERA_INFO.height = RAW_HEIGHT;
        M_CAMERA_INFO.distortion_model = D_MODEL;
        M_CAMERA_INFO.header.seq = 0;

        std::vector<double> dataValues1;
        std::vector<double> dataValues2;
        std::vector<double> dataValues3;
        std::vector<double> dataValues4;

        try
        {
            nh_.getParam("/camera_matrix/data", dataValues1);
            nh_.getParam("/distortion_coefficients/data", dataValues2);
            nh_.getParam("/rectification_matrix/data", dataValues3);
            nh_.getParam("/projection_matrix/data", dataValues4);

            for (std::size_t i = 0; i < 9; ++i)
            {
                M_CAMERA_INFO.K[i] = dataValues1[i];
                M_CAMERA_INFO.R[i] = dataValues3[i];
            }
            for (std::size_t i = 0; i < 12; ++i)
            {
                M_CAMERA_INFO.P[i] = dataValues4[i];
            }
            M_CAMERA_INFO.D.resize(dataValues2.size());
            for (std::size_t i = 0; i < 5; ++i)
            {
                M_CAMERA_INFO.D[i] = dataValues2[i];
            }
        }
        catch (ros::Exception &e)
        {
            ROS_ERROR("Could not retrieve camera parameters: %s", e.what());
        }

        // Read homography matrix parameters
        cv::FileStorage fs(path, cv::FileStorage::READ);
        fs["homographyMatrix"] >> HOMOGRAPHY_MATRIX;
        fs.release();
        if (HOMOGRAPHY_MATRIX.empty())
        {
            ROS_INFO("Could not retrieve homography matrix. It will cause problem to the publishing mode.");
        }
    }

    // Shut down everything
    void stop()
    {
        pub_1_.shutdown();
        pub_2_.shutdown();
        pub_3_.shutdown();
        pub_4_.shutdown();
        cv::destroyAllWindows();
        ros::shutdown();
        std::cout << "Feature matcher node shutted down" << std::endl;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "features_registrator");
    FeaturesRegistrator featuresRegistrator(argc, argv);
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    featuresRegistrator.stop();
    return 0;
}