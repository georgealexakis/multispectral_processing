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

class CornersRegistrator
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
    static constexpr const char *file = "/resources/homography2.yaml";
    cv::Mat HOMOGRAPHY_MATRIX;
    // Termination criteria for point detection and chessboard pattern size
    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 50, 0.001);
    cv::Size patternsize = cv::Size(7, 5);
    // Node operation capturing or not
    bool OPERATION = false;
    std::string path;
    // Capture best homography matrix
    cv::Mat BEST_HOMOGRAPHY;
    double MIN_DIFF = 255.0;

public:
    CornersRegistrator(int argc, char **argv) : it_(nh_)
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
        sync_->registerCallback(boost::bind(&CornersRegistrator::callback, this, _1, _2, _3));
        // Get package path
        path = ros::package::getPath(package) + file;
        // Read camera parameters
        readCameraParameters(nh_);
        // Display info
        ROS_INFO("Corners registrator node started");
        if ((argc > 1) && (((std::string("capture")).compare(argv[1])) == 0))
        {
            OPERATION = true;
            ROS_INFO("Perspective transformation capturing mode started.");
        }
        else
        {
            ROS_INFO("Publishing mode started.");
        }
    }

    ~CornersRegistrator()
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
                MIN_DIFF = 255;
            }
            // The estimated homography will be calculated
            cv::Mat h, r1, r2, r3, r4;
            cv::Mat vis1 = cv::Mat::zeros(RAW_HEIGHT, 2 * RAW_WIDTH, CV_8UC3);
            cv::Mat vis2 = cv::Mat::zeros(RAW_HEIGHT, 2 * RAW_WIDTH, CV_16UC1);
            computeHomographyCorners(img1, img2, h);
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

    // Find homography matrix with chessboard corners. More: https://docs.opencv.org/master/dc/dc3/tutorial_py_matcher.html
    void computeHomographyCorners(cv::Mat &im1, cv::Mat &im2, cv::Mat &h)
    {
        bool ret1, ret2;
        std::vector<cv::Point2f> c1, c2;
        // Find chessboard corners
        // The function requires white space (like a square-thick border,
        // the wider the better) around the board to make the detection more robust in various environments.
        // Otherwise, if there is no border and the background is dark,
        // the outer black squares cannot be segmented properly and so the square grouping and ordering algorithm fails.
        ret1 = cv::findChessboardCorners(im1, patternsize, c1, cv::CALIB_CB_FAST_CHECK);
        ret2 = cv::findChessboardCorners(im2, patternsize, c2, cv::CALIB_CB_FAST_CHECK);
        // Init messages
        std::stringstream md, cd1, cd2, df;
        md << "Min captured difference: " << MIN_DIFF;
        cd1 << "Chessboard detected: " << ret1;
        cd2 << "Chessboard detected: " << ret2;
        if (ret1 && ret2)
        {
            // Determine corners positions more accurately
            cv::cornerSubPix(im1, c1, cv::Size(11, 11),
                             cv::Size(-1, -1), criteria);
            cv::cornerSubPix(im2, c2, cv::Size(11, 11),
                             cv::Size(-1, -1), criteria);

            // Draw the corners
            cv::Mat img1, img2;
            im1.copyTo(img1);
            im2.copyTo(img2);
            cv::drawChessboardCorners(img1, patternsize, cv::Mat(c1), ret1);
            cv::drawChessboardCorners(img2, patternsize, cv::Mat(c2), ret2);

            // Resize multispectral image to fit
            cv::Mat img1New = cv::Mat::zeros(1080, 1920, CV_8UC1);
            img1.copyTo(img1New(cv::Rect(320, 28, RAW_WIDTH, RAW_HEIGHT)));
            // Put text messages multispectral image and kinect image with chessboard
            cv::putText(img1New, md.str(), cv::Point2f(10, 950), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2, CV_AA);
            cv::putText(img1New, cd1.str(), cv::Point2f(10, 1000), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2, CV_AA);
            cv::putText(img2, cd2.str(), cv::Point2f(10, 1000), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2, CV_AA);
            // Merge multispectral image and kinect image with chessboard
            cv::Mat vis1 = cv::Mat::zeros(1080, 2 * 1920, CV_8UC1);
            img1New.copyTo(vis1(cv::Rect(0, 0, 1920, 1080)));
            img2.copyTo(vis1(cv::Rect(1920, 0, 1920, 1080)));
            // Dispaly multispectral image and kinect image with chessboard
            cv::namedWindow("Multispectral image - Kinect image", cv::WINDOW_NORMAL);
            cv::resizeWindow("Multispectral image - Kinect image", 600, 600);
            cv::imshow("Multispectral image - Kinect image", vis1);

            // Find homography with RANSAC method (finds a perspective transformation between two planes)
            cv::Mat tempH = cv::findHomography(c2, c1, cv::RANSAC);
            if (!tempH.empty())
            {
                // Compute difference between images and the mean of pixels
                cv::Mat im2New;
                cv::warpPerspective(im2, im2New, tempH, img1.size());
                cv::Mat subResult;
                cv::subtract(im1, im2New, subResult);
                cv::Scalar tempVal = cv::mean(subResult);
                double meanResult = tempVal.val[0];
                // Keep the homography matrix with the most matches and minimun difference
                if (meanResult < MIN_DIFF)
                {
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
            // Resize multispectral image to fit
            cv::Mat im1New = cv::Mat::zeros(1080, 1920, CV_8UC1);
            im1.copyTo(im1New(cv::Rect(320, 28, RAW_WIDTH, RAW_HEIGHT)));
            // Put text messages multispectral image and kinect image with chessboard
            cv::putText(im1New, md.str(), cv::Point2f(10, 950), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2, CV_AA);
            cv::putText(im1New, cd1.str(), cv::Point2f(10, 1000), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2, CV_AA);
            cv::putText(im2, cd2.str(), cv::Point2f(10, 1000), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2, CV_AA);
            // Merge multispectral image and kinect image with chessboard
            cv::Mat vis1 = cv::Mat::zeros(1080, 2 * 1920, CV_8UC1);
            im1New.copyTo(vis1(cv::Rect(0, 0, 1920, 1080)));
            im2.copyTo(vis1(cv::Rect(1920, 0, 1920, 1080)));
            // Dispaly multispectral image and kinect image with chessboard
            cv::namedWindow("Multispectral image - Kinect image", cv::WINDOW_NORMAL);
            cv::resizeWindow("Multispectral image - Kinect image", 600, 600);
            cv::imshow("Multispectral image - Kinect image", vis1);
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
            std::cout << "-------------Homography matrix saved to homography2.yaml-------------" << std::endl;
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
        std::cout << "Corners registrator node shutted down" << std::endl;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "corners_registrator");
    CornersRegistrator cornersRegistrator(argc, argv);
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    cornersRegistrator.stop();
    return 0;
}