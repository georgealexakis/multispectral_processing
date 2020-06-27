#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <math.h>
#include <time.h>
#include <limits.h>

class BandSeparator
{
private:
  // Declare node handler, subscriber and publishers
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_m;
  std::vector<image_transport::Publisher> pub_b;
  image_transport::Publisher pub_3i, pub_n, pub_nc, pub_bg;
  ros::Publisher pub_c;
  ros::Subscriber sub_c;

  // Declare camera info and header messages
  sensor_msgs::CameraInfo CAMERA_INFO, CI3;

  // Camera information, coefficients variables
  std::string cameraManufacturerSN;
  std::string cameraSN;
  std::string cameraReference;
  std::vector<double> crosstalkCorrectionCoefficients;
  std::vector<double> whiteReferenceCoefficients;

  // Image encoding, CV_8UC1 for grayscale images. Unsigned 8bits uchar 0~255
  // +--------+----+----+----+----+------+------+------+------+
  // |        | C1 | C2 | C3 | C4 | C(5) | C(6) | C(7) | C(8) |
  // +--------+----+----+----+----+------+------+------+------+
  // | CV_8U  |  0 |  8 | 16 | 24 |   32 |   40 |   48 |   56 |
  // | CV_8S  |  1 |  9 | 17 | 25 |   33 |   41 |   49 |   57 |
  // | CV_16U |  2 | 10 | 18 | 26 |   34 |   42 |   50 |   58 |
  // | CV_16S |  3 | 11 | 19 | 27 |   35 |   43 |   51 |   59 |
  // | CV_32S |  4 | 12 | 20 | 28 |   36 |   44 |   52 |   60 |
  // | CV_32F |  5 | 13 | 21 | 29 |   37 |   45 |   53 |   61 |
  // | CV_64F |  6 | 14 | 22 | 30 |   38 |   46 |   54 |   62 |
  // +--------+----+----+----+----+------+------+------+------+
  static const short IME = CV_8UC1;
  static const short IMEC = CV_8UC3;
  // cv::Mat with doubles
  static const short IMEF = CV_64FC1;

  // 8 bit image type for publishing
  static constexpr const char *IME_B = "mono8";
  static constexpr const char *IME_C = "bgr8";

  // Camera info
  static const short RAW_WIDTH = 1280;
  static const short RAW_HEIGHT = 1024;
  static const short CRAW_WIDTH = 1278;
  static const short CRAW_HEIGHT = 1017;
  static const short BAND_WIDTH = 426;
  static const short BAND_HEIGHT = 339;
  static constexpr const char *D_MODEL = "plumb_bob";
  static constexpr const char *FRAME_ID = "multispectral_band_frame";

  // Crosstalk correction coefficients from manifacturer resources
  double wCoefCrossTalk[9][9];

  // Reading order of every super pixel starts from (2, 0)
  const short bandsOrder[9] = {4, 8, 0, 3, 2, 1, 5, 6, 7};

  // Reading order of every super pixel starts from (0, 0)
  const short bandsOrderCo[9] = {6, 7, 8, 0, 1, 2, 3, 4, 5};

  // Set offset to begin from (2, 0)
  static const short offsetX = 2;
  static const short offsetY = 0;

  // Flat-field and dark-field correction images
  cv::Mat F = cv::Mat::ones(RAW_HEIGHT, RAW_WIDTH, IME);
  cv::Mat D = cv::Mat::ones(RAW_HEIGHT, RAW_WIDTH, IME);
  static constexpr const char *FF_PATH = "/data/flat-field-correction/flat-field.png";
  static constexpr const char *DF_PATH = "/data/flat-field-correction/dark-field.png";

  // Keyboard buttons triggers
  bool buttonTriggers[7] = {false, false, false, false, false, false, false};

  // White balance selected area positions and status. positions[4] is the status (selected/unselected)
  int positions[5] = {-1, -1, -1, -1, -1};

  // Enable/disable debug mode
  bool OPERATION = false;

  // FPS counter
  time_t start, end;
  int counter = 0;
  double sec;
  double fps;
  std::string FPS_PATH, WR_PATH;
  static constexpr const char *PACKAGE = "multispectral_processing";
  static constexpr const char *FPS_LOG = "/resources/fps_log.yaml";
  static constexpr const char *WR_CO = "/resources/wr_coefficients.yaml";

public:
  BandSeparator(int argc, char **argv)
      : it_(nh_)
  {
    // Initialize subscriber and publishers
    sub_m = it_.subscribe("/camera/image_raw", 1, &BandSeparator::imageCallback, this);
    sub_c = nh_.subscribe("/camera_settings", 1, &BandSeparator::controllerCallback, this);
    // Initialize publishers
    pub_b.resize(9);
    for (size_t i = 0; i < 9; ++i)
    {
      std::stringstream bandTopic;
      bandTopic << "/band/" << (i + 1) << "/image_raw";
      pub_b[i] = it_.advertise(bandTopic.str(), 1);
    }
    pub_3i = it_.advertise("/band/3/interpolated/image_raw", 1);
    pub_n = it_.advertise("/band/ndvi/image_raw", 1);
    pub_nc = it_.advertise("/band/ndvi_colored/image_raw", 1);
    pub_bg = it_.advertise("/bands/grid/image_raw", 1);
    pub_c = nh_.advertise<sensor_msgs::CameraInfo>("/band/camera_info", 1);

    // Load CMS-V camera manufacturer parameters
    nh_.getParam("/manufacturerSN", cameraManufacturerSN);
    nh_.getParam("/siliosSN", cameraSN);
    nh_.getParam("/reference", cameraReference);
    nh_.getParam("/crosstalkCorrectionCoefficients", crosstalkCorrectionCoefficients);
    whiteReferenceCoefficients.resize(9);
    for (std::size_t i = 0; i < 9; ++i)
    {
      whiteReferenceCoefficients.at(i) = 1.0;
    }
    readCameraParameters(nh_);

    // Coefficients from 1d to 2d
    short step = 0;
    for (std::size_t i = 0; i < 9; ++i)
    {
      for (std::size_t j = 0; j < 9; ++j)
      {
        wCoefCrossTalk[i][j] = crosstalkCorrectionCoefficients.at(step++);
      }
    }

    // Display camera information and initial parameters
    ROS_INFO("\n\n******** Band separator node started ***********\n");
    std::cout << "Camera: " << cameraReference << std::endl
              << "M.S.N.: " << cameraManufacturerSN << std::endl
              << "C.S.N.: " << cameraSN << std::endl
              << "Crosstalk Coefficients" << std::endl;
    printCrosstalkCoefficients();
    std::cout << "Crosstalk Correction is off." << std::endl
              << "Flat-field Correction is off." << std::endl
              << "Black-field Correction is off." << std::endl
              << "White Reference is off." << std::endl;
    std::cout << "Use keyboard buttons c - Crosstalk, e - Flat-field, f - Flat-field capture, d - Dark-field capture, w - White reference, n - Display Indexes, b - Display bands, r - Reset values.";

    // Check operation mode
    if ((argc > 1) && (((std::string("debug")).compare(argv[1])) == 0))
    {
      OPERATION = true;
      std::cout << std::endl
                << "Debug mode is on." << std::endl;
    }
    std::cout << std::endl
              << "***************************************" << std::endl
              << std::endl;

    // Initialize the images for flat-field correction (flat-field image and dark-field image)
    std::string fPath = ros::package::getPath(PACKAGE) + FF_PATH;
    std::string dPath = ros::package::getPath(PACKAGE) + DF_PATH;
    F = cv::imread(fPath, 0);
    D = cv::imread(dPath, 0);

    // Initialize raw image window and set mouse listener
    cv::namedWindow("Raw Image", cv::WINDOW_NORMAL);
    cv::setMouseCallback("Raw Image", BandSeparator::onMouse, positions);

    // Get package path
    FPS_PATH = ros::package::getPath(PACKAGE) + FPS_LOG;
    WR_PATH = ros::package::getPath(PACKAGE) + WR_CO;
    // Load white reference coefficients
    loadWhiteReference();
  }

  ~BandSeparator()
  {
  }

  // Receive commands from controller
  void controllerCallback(const std_msgs::Int8::ConstPtr &msg)
  {
    short choice = msg->data;
    if (choice == 0)
    {
      buttonTriggers[0] = false;
    }
    else if (choice == 1)
    {
      buttonTriggers[0] = true;
    }
    else if (choice == 10)
    {
      buttonTriggers[1] = false;
    }
    else if (choice == 11)
    {
      buttonTriggers[1] = true;
    }
    else if (choice == 20)
    {
      buttonTriggers[2] = false;
    }
    else if (choice == 21)
    {
      buttonTriggers[2] = true;
    }
    else if (choice == 30)
    {
      buttonTriggers[3] = false;
    }
    else if (choice == 31)
    {
      buttonTriggers[3] = true;
    }
    else if (choice == 40)
    {
      buttonTriggers[4] = false;
    }
    else if (choice == 41)
    {
      buttonTriggers[4] = true;
    }
    else if (choice == 50)
    {
      buttonTriggers[5] = false;
    }
    else if (choice == 51)
    {
      buttonTriggers[5] = true;
    }
    else if (choice == 60)
    {
      buttonTriggers[6] = false;
    }
    else if (choice == 61)
    {
      buttonTriggers[6] = true;
    }
  }

  // Receive images from multispectral camera for further processing
  void imageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    // Modify ROS image to OpenCV image for further processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      // Raw image (image encoding to 8 bit)
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat rawImage = cv_ptr->image;

    // Separate every row of the band matrix.
    // It removes the last (2 - offsetY) = 2 pixels of the columns and the last (7 - offsetX) = 5
    // pixels from the rows resuling in image size equals to 1278x1017,
    // due to the precise band seperation to 426x339 for every band.
    // Band seperation follows the matrix below:
    //  +----------+----------+----------+
    //  |  Band 5  |  Band 9  |  Band 1  |
    //  |----------+----------+----------|
    //  |  Band 4  |  Band 3  |  Band 2  |
    //  |----------+----------+----------|
    //  |  Band 6  |  Band 7  |  Band 8  |
    //  +----------+----------+----------+

    // Check raw image size if it matches the right dimensions 1024x1280
    if (rawImage.rows == RAW_HEIGHT & rawImage.cols == RAW_WIDTH)
    {
      // Start FPS counter
      if (counter == 0)
      {
        time(&start);
      }

      // Generate image arrays for the 9 bands
      cv::Mat images[9];
      cv::Mat ctImages[9];
      cv::Mat wrImages[9];
      cv::Mat wrnImages[9];
      for (std::size_t im = 0; im < 9; ++im)
      {
        images[im] = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
      }

      // Flat-field correction
      if (buttonTriggers[1])
      {
        cv::Mat ffImage = cv::Mat(RAW_HEIGHT, RAW_WIDTH, IME);
        flatFieldCorrection(rawImage, ffImage);
        ffImage.copyTo(rawImage);
      }

      // Reading order is line by line for bands [5 9 1; 4 3 2; 6 7 8]
      for (std::size_t i = 0; i < BAND_HEIGHT; ++i)
      {
        for (std::size_t j = 0; j < BAND_WIDTH; ++j)
        {
          images[0].at<uchar>(i, j) = rawImage.at<uchar>(i * 3 + offsetX + 0, j * 3 + offsetY + 0);
          images[1].at<uchar>(i, j) = rawImage.at<uchar>(i * 3 + offsetX + 0, j * 3 + offsetY + 1);
          images[2].at<uchar>(i, j) = rawImage.at<uchar>(i * 3 + offsetX + 0, j * 3 + offsetY + 2);
          images[3].at<uchar>(i, j) = rawImage.at<uchar>(i * 3 + offsetX + 1, j * 3 + offsetY + 0);
          images[4].at<uchar>(i, j) = rawImage.at<uchar>(i * 3 + offsetX + 1, j * 3 + offsetY + 1);
          images[5].at<uchar>(i, j) = rawImage.at<uchar>(i * 3 + offsetX + 1, j * 3 + offsetY + 2);
          images[6].at<uchar>(i, j) = rawImage.at<uchar>(i * 3 + offsetX + 2, j * 3 + offsetY + 0);
          images[7].at<uchar>(i, j) = rawImage.at<uchar>(i * 3 + offsetX + 2, j * 3 + offsetY + 1);
          images[8].at<uchar>(i, j) = rawImage.at<uchar>(i * 3 + offsetX + 2, j * 3 + offsetY + 2);
        }
      }

      // Normalization (white reference process)
      for (std::size_t i = 0; i < 9; ++i)
      {
        cv::Mat tempWr;
        images[i].copyTo(tempWr);
        tempWr.convertTo(tempWr, IMEF);
        wrImages[i] = tempWr * whiteReferenceCoefficients.at(bandsOrderCo[i]);
        // Remove values higher than 255 and values lower than 0
        wrImages[i].copyTo(wrnImages[i]);
        wrnImages[i].setTo(255, wrnImages[i] > 255);
        wrnImages[i].setTo(0, wrnImages[i] < 0);
        wrnImages[i].convertTo(wrnImages[i], IME);
      }

      // Crosstalk correction with super-pixels to increase the contrast
      computeCrosstalkCorrection(wrImages, ctImages);

      // Display raw image. It is necessary to enable the options
      if (OPERATION)
      {
        displayRawImage(rawImage);
      }

      // Set white reference values
      setWhiteReference(rawImage, buttonTriggers[4]);

      // Choose the images to publish and copy them to the final array
      if (buttonTriggers[0])
      {
        for (int i = 0; i < 9; ++i)
        {
          ctImages[i].copyTo(images[i]);
        }
      }
      else
      {
        for (int i = 0; i < 9; ++i)
        {
          wrnImages[i].copyTo(images[i]);
        }
      }

      // Build the grid of bands
      cv::Mat bandsGrid = cv::Mat(CRAW_HEIGHT, CRAW_WIDTH, IME);
      mergeBands(images, bandsGrid);

      // NDVI calculation using band 3 and band 6
      cv::Mat ndvi = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
      cv::Mat ndviColor = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IMEC);
      ndviCalculator(images[4], images[6], ndvi, ndviColor);
      cv::Mat erdImageNDVI = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
      cv::Mat segImageNDVI = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
      segmentation(ndvi, erdImageNDVI, segImageNDVI);

      // Vegetation indexes
      if (buttonTriggers[5])
      {
        // GNDVI calculation using band 1 and band 6
        cv::Mat gndvi = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
        cv::Mat gndviColor = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IMEC);
        gndviCalculator(images[2], images[6], gndvi, gndviColor);
        cv::Mat erdImageGNDVI = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
        cv::Mat segImageGNDVI = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
        segmentation(gndvi, erdImageGNDVI, segImageGNDVI);

        // SAVI calculation using band 3 and band 6
        cv::Mat savi = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
        cv::Mat saviColor = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IMEC);
        saviCalculator(images[4], images[6], savi, saviColor);
        cv::Mat erdImageSAVI = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
        cv::Mat segImageSAVI = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
        segmentation(savi, erdImageSAVI, segImageSAVI);

        // GSAVI calculation using band 1 and band 6
        cv::Mat gsavi = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
        cv::Mat gsaviColor = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IMEC);
        gsaviCalculator(images[2], images[6], gsavi, gsaviColor);
        cv::Mat erdImageGSAVI = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
        cv::Mat segImageGSAVI = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
        segmentation(gsavi, erdImageGSAVI, segImageGSAVI);

        // MCARI calculation using band 1, band 4 and band 5
        cv::Mat mcari = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
        mcariCalculator(images[2], images[3], images[0], mcari);

        // MSR calculation using band 3 and band 6
        cv::Mat msr = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
        cv::Mat msrColor = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IMEC);
        msrCalculator(images[4], images[6], msr, msrColor);
        cv::Mat erdImageMSR = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
        cv::Mat segImageMSR = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
        segmentation(msr, erdImageMSR, segImageMSR);

        // TVI, MTVI1, MTVI2 calculation using band 1, band 3, band 6, band 4 and band 7
        cv::Mat tvi = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
        cv::Mat mtvi1 = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
        cv::Mat mtvi2 = cv::Mat(BAND_HEIGHT, BAND_WIDTH, IME);
        tviCalculator(images[2], images[4], images[6], images[3], images[7], tvi, mtvi1, mtvi2);

        // Display the images of the vegetation indexes
        displayImage(ndvi, "NDVI");
        displayImage(ndviColor, "NDVI Colormap");
        displayImage(gndvi, "GNDVI");
        displayImage(gndviColor, "GNDVI Colormap");
        displayImage(savi, "SAVI");
        displayImage(saviColor, "SAVI Colormap");
        displayImage(gsavi, "GSAVI");
        displayImage(gsaviColor, "GSAVI Colormap");
        displayImage(mcari, "MCARI");
        displayImage(msr, "MSR");
        displayImage(msrColor, "MSR Colormap");
        displayImage(tvi, "TVI");
        displayImage(mtvi1, "MTVI1");
        displayImage(mtvi2, "MTVI2");
        displayImage(segImageNDVI, "Segmented Image NDVI");
        displayImage(erdImageNDVI, "Eroded & Dilated Image NDVI");
        displayImage(segImageGNDVI, "Segmented Image GNDVI");
        displayImage(erdImageGNDVI, "Eroded & Dilated Image GNDVI");
        displayImage(segImageSAVI, "Segmented Image SAVI");
        displayImage(erdImageSAVI, "Eroded & Dilated Image SAVI");
        displayImage(segImageGSAVI, "Segmented Image GSAVI");
        displayImage(erdImageGSAVI, "Eroded & Dilated Image GSAVI");
        displayImage(segImageMSR, "Segmented Image MSR");
        displayImage(erdImageMSR, "Eroded & Dilated Image MSR");
      }
      else
      {
        cv::destroyWindow("NDVI");
        cv::destroyWindow("NDVI Colormap");
        cv::destroyWindow("GNDVI");
        cv::destroyWindow("GNDVI Colormap");
        cv::destroyWindow("SAVI");
        cv::destroyWindow("SAVI Colormap");
        cv::destroyWindow("GSAVI");
        cv::destroyWindow("GSAVI Colormap");
        cv::destroyWindow("MCARI");
        cv::destroyWindow("MSR");
        cv::destroyWindow("MSR Colormap");
        cv::destroyWindow("TVI");
        cv::destroyWindow("MTVI1");
        cv::destroyWindow("MTVI2");
        cv::destroyWindow("Segmented Image NDVI");
        cv::destroyWindow("Eroded & Dilated Image NDVI");
        cv::destroyWindow("Segmented Image GNDVI");
        cv::destroyWindow("Eroded & Dilated Image GNDVI");
        cv::destroyWindow("Segmented Image SAVI");
        cv::destroyWindow("Eroded & Dilated Image SAVI");
        cv::destroyWindow("Segmented Image GSAVI");
        cv::destroyWindow("Eroded & Dilated Image GSAVI");
        cv::destroyWindow("Segmented Image MSR");
        cv::destroyWindow("Eroded & Dilated Image MSR");
      }

      // Display or not final bands after pre-processing
      if (buttonTriggers[6])
      {
        displayImage(bandsGrid, "Band 8(828nm), Band 1(560nm), Band 2(595nm); Band 7(791nm), Band 9(Panchromatic filter), Band 3(634nm); Band 6(752nm), Band 5(713nm), Band 4(673nm)");
      }
      else
      {
        cv::destroyWindow("Band 8(828nm), Band 1(560nm), Band 2(595nm); Band 7(791nm), Band 9(Panchromatic filter), Band 3(634nm); Band 6(752nm), Band 5(713nm), Band 4(673nm)");
      }

      // Keyboard listener
      if (OPERATION)
      {
        setOperation(cv::waitKey(1), rawImage);
      }

      // Transform and publish image bands topics
      publisher(images, ndvi, ndviColor, bandsGrid);

      // Log FPS to fps_log.yaml
      time(&end);
      counter++;
      sec = difftime(end, start);
      fps = counter / sec;
      if (counter > 30)
      {
        cv::FileStorage fs(FPS_PATH, cv::FileStorage::WRITE);
        fs << "FPS" << fps;
        fs.release();
      }
      if (counter == (INT_MAX - 1000))
      {
        counter = 0;
      }
    }
    else
    {
      ROS_INFO("Wrong input image dimensions.");
    }
  }

  // Calculate cross-talk correction for every single pixel
  void computeCrosstalkCorrection(cv::Mat wrImages[], cv::Mat ctImages[])
  {
    // Crosstalk correction multiply and sum process as it is in the formula
    for (std::size_t i = 0; i < 9; ++i)
    {
      cv::Mat result = cv::Mat::zeros(BAND_HEIGHT, BAND_WIDTH, IMEF);
      for (std::size_t j = 0; j < 9; ++j)
      {
        cv::add(wrImages[j] * wCoefCrossTalk[bandsOrder[j]][bandsOrder[i]], result, result);
      }
      // Remove values higher than 255 and values lower than 0 and convert to CV_8UC1
      result.setTo(255, result > 255);
      result.setTo(0, result < 0);
      result.convertTo(result, IME);
      result.copyTo(ctImages[i]);
    }
  }

  // Display image
  void displayImage(cv::Mat &img, std::string title)
  {
    cv::namedWindow(title, cv::WINDOW_NORMAL);
    cv::resizeWindow(title, 600, 600);
    cv::imshow(title, img);
  }

  // Read camera parameters and create parameters for sub-bands /band/camera_info
  void readCameraParameters(ros::NodeHandle nh_)
  {
    CAMERA_INFO.header.frame_id = FRAME_ID;
    CAMERA_INFO.width = BAND_WIDTH;
    CAMERA_INFO.height = BAND_HEIGHT;
    CAMERA_INFO.distortion_model = D_MODEL;
    CAMERA_INFO.header.seq = 0;
    CAMERA_INFO.header.stamp = ros::Time(0, 0);

    std::vector<double> dataValues1;
    std::vector<double> dataValues2;
    std::vector<double> dataValues3;
    std::vector<double> dataValues4;

    nh_.getParam("/camera_matrix/data", dataValues1);
    nh_.getParam("/distortion_coefficients/data", dataValues2);
    nh_.getParam("/rectification_matrix/data", dataValues3);
    nh_.getParam("/projection_matrix/data", dataValues4);

    for (std::size_t i = 0; i < 9; ++i)
    {
      CAMERA_INFO.K[i] = dataValues1[i];
      CAMERA_INFO.R[i] = dataValues3[i];
    }
    for (std::size_t i = 0; i < 12; ++i)
    {
      CAMERA_INFO.P[i] = dataValues4[i];
    }
    CAMERA_INFO.D.resize(dataValues2.size());
    for (std::size_t i = 0; i < 5; ++i)
    {
      CAMERA_INFO.D[i] = dataValues2[i];
    }
  }

  // Display raw image and info messages
  void displayRawImage(cv::Mat &img)
  {
    cv::Mat rawImage;
    img.copyTo(rawImage);
    cv::putText(rawImage, "Use keyboard buttons c, e, f, d, w.", cv::Point2f(10, 915), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, CV_AA);
    if (buttonTriggers[0])
    {
      cv::putText(rawImage, "Crosstalk Correction is on.", cv::Point2f(10, 930), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, CV_AA);
    }
    else
    {
      cv::putText(rawImage, "Crosstalk Correction is off.", cv::Point2f(10, 930), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, CV_AA);
    }
    if (buttonTriggers[1])
    {
      cv::putText(rawImage, "Flat-field Correction is on.", cv::Point2f(10, 945), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, CV_AA);
    }
    else
    {
      cv::putText(rawImage, "Flat-field Correction is off.", cv::Point2f(10, 945), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, CV_AA);
    }
    if (buttonTriggers[2])
    {
      cv::putText(rawImage, "Flat-field image capture is on.", cv::Point2f(10, 960), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, CV_AA);
    }
    else
    {
      cv::putText(rawImage, "Flat-field image capture is off.", cv::Point2f(10, 960), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, CV_AA);
    }
    if (buttonTriggers[3])
    {
      cv::putText(rawImage, "Dark-field image capture is on.", cv::Point2f(10, 975), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, CV_AA);
    }
    else
    {
      cv::putText(rawImage, "Dark-field image capture is off.", cv::Point2f(10, 975), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, CV_AA);
    }
    if (buttonTriggers[4])
    {
      cv::putText(rawImage, "White Reference is on.", cv::Point2f(10, 990), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, CV_AA);
    }
    else
    {
      cv::putText(rawImage, "White Reference is off.", cv::Point2f(10, 990), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, CV_AA);
    }
    if (buttonTriggers[5])
    {
      cv::putText(rawImage, "Display Indexes is on.", cv::Point2f(10, 1005), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, CV_AA);
    }
    else
    {
      cv::putText(rawImage, "Display Indexes is off.", cv::Point2f(10, 1005), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, CV_AA);
    }
    displayImage(rawImage, "Raw Image");
  }

  // Keyboard listener for predefined cases
  void setOperation(int key, cv::Mat &rawImage)
  {
    switch (key)
    {
    case 99: // Keyboard button <c>
    case 67: // Keyboard button <C>
      if (!buttonTriggers[0])
      {
        std::cout << "Crosstalk Correction is on." << std::endl;
        buttonTriggers[0] = true;
      }
      else
      {
        std::cout << "Crosstalk Correction is off." << std::endl;
        buttonTriggers[0] = false;
      }
      break;
    case 101: // Keyboard button <e>
    case 69:  // Keyboard button <E>
      if (!buttonTriggers[1])
      {
        std::cout << "Flat-field Correction is on." << std::endl;
        buttonTriggers[1] = true;
      }
      else
      {
        std::cout << "Flat-field Correction is off." << std::endl;
        buttonTriggers[1] = false;
      }
      break;
    case 102: // Keyboard button <f>
    case 70:  // Keyboard button <F>
      if (!buttonTriggers[2])
      {
        std::cout << "Flat-field image capture is on." << std::endl;
        buttonTriggers[2] = true;
        F = rawImage;
      }
      else
      {
        std::cout << "Flat-field image capture is off." << std::endl;
        buttonTriggers[2] = false;
      }
      break;
    case 100: // Keyboard button <d>
    case 68:  // Keyboard button <D>
      if (!buttonTriggers[3])
      {
        std::cout << "Dark-field image capture is on." << std::endl;
        buttonTriggers[3] = true;
        D = rawImage;
      }
      else
      {
        std::cout << "Dark-field image capture is off." << std::endl;
        buttonTriggers[3] = false;
      }
      break;
    case 119: // Keyboard button <w>
    case 87:  // Keyboard button <W>
      if (!buttonTriggers[4])
      {
        std::cout << "White Reference is on." << std::endl;
        resetPositions();
        buttonTriggers[4] = true;
      }
      else
      {
        std::cout << "White Reference is off." << std::endl;
        buttonTriggers[4] = false;
      }
      break;
    case 110: // Keyboard button <n>
    case 78:  // Keyboard button <N>
      if (!buttonTriggers[5])
      {
        std::cout << "Display Indexes is on." << std::endl;
        buttonTriggers[5] = true;
      }
      else
      {
        std::cout << "Display Indexes is off." << std::endl;
        buttonTriggers[5] = false;
      }
      break;
    case 98: // Keyboard button <b>
    case 66: // Keyboard button <B>
      if (!buttonTriggers[6])
      {
        std::cout << "Display bands is on." << std::endl;
        buttonTriggers[6] = true;
      }
      else
      {
        std::cout << "Display bands is off." << std::endl;
        buttonTriggers[6] = false;
      }
      break;
    case 114: // Keyboard button <r>
    case 82:  // Keyboard button <R>
      std::cout << "White reference values have reseted." << std::endl;
      for (std::size_t i = 0; i < 9; ++i)
      {
        whiteReferenceCoefficients.at(i) = 1.0;
      }
      saveWhiteReference();
      break;
    default:
      break;
    }
  }

  // Reset white reference area positions
  void resetPositions()
  {
    for (std::size_t i = 0; i < 5; ++i)
    {
      positions[i] = -1;
    }
  }

  // Compute flat-field correction that allows you to homogenize the background
  void flatFieldCorrection(cv::Mat &rawImage, cv::Mat &P)
  {
    // Flat-fied correction equation [P i,j = Raw i,j − D i,j / F i,j − D i,j * 1 / m∗n ∗ ∑ ∑ (F x,y − D x,y)]
    short m = 1024;
    short n = 1280;
    double md = 1024.0;
    double nd = 1280.0;
    double flatFieldSum = 0.0;
    double flatfieldPixel = 0.0;
    for (std::size_t i = 0; i < m; ++i)
    {
      for (std::size_t j = 0; j < n; ++j)
      {
        flatFieldSum += ((double)F.at<uchar>(i, j) - (double)D.at<uchar>(i, j));
      }
    }
    for (std::size_t i = 0; i < m; ++i)
    {
      for (std::size_t j = 0; j < n; ++j)
      {
        flatfieldPixel = (((double)rawImage.at<uchar>(i, j) - (double)D.at<uchar>(i, j)) / ((double)F.at<uchar>(i, j) - (double)D.at<uchar>(i, j))) * (1.0 / (md * nd)) * flatFieldSum;
        if (flatfieldPixel > 255)
          flatfieldPixel = 255;
        else if (flatfieldPixel < 0)
          flatfieldPixel = 0;
        P.at<uchar>(i, j) = flatfieldPixel;
      }
    }
  }

  // Enable/disable white reference and draw recthange for the selected area
  void setWhiteReference(cv::Mat &rawImage, bool enable)
  {
    if (enable)
    {
      cv::Mat rgb;
      rawImage.copyTo(rgb);
      cv::cvtColor(rgb, rgb, CV_GRAY2BGR);
      if (positions[0] >= 0 && positions[1] >= 0 && positions[3] >= 0 && positions[3] >= 0)
      {
        cv::rectangle(rgb, cv::Point(positions[0], positions[1]), cv::Point(positions[2], positions[3]), cv::Scalar(0, 0, 255), 3, 8, 0);
        whiteReferenceCalculator(rawImage);
      }
      // Display image with colored rectangle
      displayImage(rgb, "Raw Image");
    }
  }

  // Mouse listener for white balance area selection
  static void onMouse(int event, int x, int y, int flags, void *pixelPositions)
  {
    int *positions;
    positions = (int *)pixelPositions;
    if (event == cv::EVENT_LBUTTONDOWN)
    {
      if ((x >= 0) && (y >= 0) && (x <= 1280) && (y <= 1024))
      {
        if (x >= 1278)
        {
          positions[0] = 1278;
        }
        else
        {
          positions[0] = x;
        }
        if (y >= 1022)
        {
          positions[1] = 1022;
        }
        else
        {
          positions[1] = y;
        }
        positions[2] = -1;
        positions[3] = -1;
        positions[4] = 0;
        std::cout << "White Reference Point 1 -> P0: " << positions[0] << " / P1: " << positions[1] << " - (Input) x: " << x << " / y: " << y << std::endl;
      }
      else
      {
        positions[0] = -1;
        positions[1] = -1;
        positions[2] = -1;
        positions[3] = -1;
      }
    }
    else if (event == cv::EVENT_MOUSEMOVE)
    {
      if (positions[4] == 0)
      {
        positions[2] = x;
        positions[3] = y;
      }
    }
    else if (event == cv::EVENT_LBUTTONUP)
    {
      if ((x >= 0) && (y >= 0) && (x <= 1280) && (y <= 1024))
      {
        if (((positions[0] == x) && (positions[1] == y)) || ((positions[0] == x + 1) && (positions[1] == y + 1)) || ((positions[0] == x + 2) && (positions[1] == y + 2)))
        {
          positions[2] = positions[0] + 2;
          positions[3] = positions[1] + 2;
        }
        else if ((positions[0] == x) || (positions[0] == x + 1) || (positions[0] == x + 2))
        {
          positions[2] = positions[0] + 2;
          positions[3] = y;
        }
        else if ((positions[1] == y) || (positions[1] == y + 1) || (positions[1] == y + 2))
        {
          positions[2] = x;
          positions[3] = positions[1] + 2;
        }
        else
        {
          positions[2] = x;
          positions[3] = y;
        }
        positions[4] = 1;
        std::cout << "White Reference Point 2 -> P2: " << positions[2] << " / P3: " << positions[3] << " - (Input) x: " << x << " / y: " << y << std::endl;
      }
      else
      {
        positions[0] = -1;
        positions[1] = -1;
        positions[2] = -1;
        positions[3] = -1;
      }
    }
  }

  // Crop selected pixels and calculates the white reference coefficients
  void whiteReferenceCalculator(cv::Mat &rawImage)
  {
    // For white balance it has being followed the equation [whiteBalance = 255 / average of the pixels in the selected area for every band]
    std::size_t sizeR = positions[3] - positions[1] + 1; // Take the size of the selected area (rows)
    std::size_t startR = positions[1] - 1;               // Take the starting point of the selected area (rows)
    std::size_t endR = startR + sizeR;                   // Take the ending point of the selected area (rows)
    std::size_t sizeC = positions[2] - positions[0] + 1; // Take the size of the selected area (columns)
    std::size_t startC = positions[0] - 1;               // Take the starting point of the selected area (columns)
    std::size_t endC = startC + sizeC;                   // Take the ending point of the selected area (columns)
    if (positions[4] == 1 && (sizeR > 2 && sizeC > 2))
    {
      positions[4] = -1;
      if ((positions[0] < positions[2]) && (positions[1] < positions[3]))
      {
        double pixelSum[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};  // Initiallize sum array
        short pixelCount[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // Initiallize count array
        std::cout << "r: " << sizeR << " c: " << sizeC << std::endl;
        std::cout << "-------------------------------------------------" << std::endl;
        std::cout << "Row start: from " << startR << " to " << endR << std::endl;
        std::cout << "Column start: from " << startC << " to " << endC << std::endl;
        for (std::size_t i = startR; i < endR; ++i)
        {
          for (std::size_t j = startC; j < endC; ++j)
          {
            if (i % 3 == 0)
            {
              if (j % 3 == 0)
              {
                pixelSum[0] += (double)rawImage.at<uchar>(i, j);
                pixelCount[0]++;
                std::cout << "Pixel 1: " << (double)rawImage.at<uchar>(i, j) << std::endl;
              }
              else if (j % 3 == 1)
              {
                pixelSum[1] += (double)rawImage.at<uchar>(i, j);
                pixelCount[1]++;
                std::cout << "Pixel 2: " << (double)rawImage.at<uchar>(i, j) << std::endl;
              }
              else
              {
                pixelSum[2] += (double)rawImage.at<uchar>(i, j);
                pixelCount[2]++;
                std::cout << "Pixel 3: " << (double)rawImage.at<uchar>(i, j) << std::endl;
              }
            }
            else if (i % 3 == 1)
            {
              if (j % 3 == 0)
              {
                pixelSum[3] += (double)rawImage.at<uchar>(i, j);
                pixelCount[3]++;
                std::cout << "Pixel 4: " << (double)rawImage.at<uchar>(i, j) << std::endl;
              }
              else if (j % 3 == 1)
              {
                pixelSum[4] += (double)rawImage.at<uchar>(i, j);
                pixelCount[4]++;
                std::cout << "Pixel 5: " << (double)rawImage.at<uchar>(i, j) << std::endl;
              }
              else
              {
                pixelSum[5] += (double)rawImage.at<uchar>(i, j);
                pixelCount[5]++;
                std::cout << "Pixel 6: " << (double)rawImage.at<uchar>(i, j) << std::endl;
              }
            }
            else
            {
              if (j % 3 == 0)
              {
                pixelSum[6] += (double)rawImage.at<uchar>(i, j);
                pixelCount[6]++;
                std::cout << "Pixel 7: " << (double)rawImage.at<uchar>(i, j) << std::endl;
              }
              else if (j % 3 == 1)
              {
                pixelSum[7] += (double)rawImage.at<uchar>(i, j);
                pixelCount[7]++;
                std::cout << "Pixel 8: " << (double)rawImage.at<uchar>(i, j) << std::endl;
              }
              else
              {
                pixelSum[8] += (double)rawImage.at<uchar>(i, j);
                pixelCount[8]++;
                std::cout << "Pixel 9: " << (double)rawImage.at<uchar>(i, j) << std::endl;
              }
            }
          }
        }
        std::cout << "-------------------------------------------------" << std::endl;
        std::cout << "Sum: " << pixelSum[0] << " TP: " << pixelCount[0] << std::endl;
        std::cout << "Sum: " << pixelSum[1] << " TP: " << pixelCount[1] << std::endl;
        std::cout << "Sum: " << pixelSum[2] << " TP: " << pixelCount[2] << std::endl;
        std::cout << "Sum: " << pixelSum[3] << " TP: " << pixelCount[3] << std::endl;
        std::cout << "Sum: " << pixelSum[4] << " TP: " << pixelCount[4] << std::endl;
        std::cout << "Sum: " << pixelSum[5] << " TP: " << pixelCount[5] << std::endl;
        std::cout << "Sum: " << pixelSum[6] << " TP: " << pixelCount[6] << std::endl;
        std::cout << "Sum: " << pixelSum[7] << " TP: " << pixelCount[7] << std::endl;
        std::cout << "Sum: " << pixelSum[8] << " TP: " << pixelCount[8] << std::endl;
        std::cout << "-------------------------------------------------" << std::endl;

        for (std::size_t i = 0; i < 9; ++i)
        {
          double result = 255 / (pixelSum[i] / pixelCount[i]);
          if (!isinf(result) && !isnan(result))
            whiteReferenceCoefficients.at(i) = result;
          std::cout << "CO: " << whiteReferenceCoefficients.at(i) << std::endl;
        }
        std::cout << "-------------------------------------------------" << std::endl;
        saveWhiteReference();
      }
      else
      {
        std::cout << "Start from top left to bottom right." << std::endl;
      }
    }
  }

  // Load white reference coefficients
  void loadWhiteReference()
  {
    cv::Mat temp;
    cv::FileStorage fs(WR_PATH, cv::FileStorage::READ);
    fs["whiteReferenceCoefficients"] >> temp;
    fs.release();
    for (std::size_t i = 0; i < 9; ++i)
    {
      whiteReferenceCoefficients.at(i) = temp.at<double>(i);
    }
    if (whiteReferenceCoefficients.empty())
    {
      ROS_INFO("Could not retrieve white reference coefficients. Defaults are set.");
      for (std::size_t i = 0; i < 9; ++i)
      {
        whiteReferenceCoefficients.at(i) = 1.0;
      }
    }
  }

  // Save white reference coefficients
  void saveWhiteReference()
  {
    if (!whiteReferenceCoefficients.empty())
    {
      cv::Mat temp = cv::Mat::zeros(1, 9, CV_64F);
      for (std::size_t i = 0; i < 9; ++i)
      {
        temp.at<double>(i) = whiteReferenceCoefficients.at(i);
      }
      std::cout << "White reference coefficients saved." << std::endl;
      std::cout << temp << std::endl;
      cv::FileStorage fs(WR_PATH, cv::FileStorage::WRITE);
      fs << "whiteReferenceCoefficients" << temp;
      fs.release();
      std::cout << "-------------------------------------------------" << std::endl;
    }
    else
    {
      std::cout << "No white reference coefficients are available." << std::endl;
    }
  }

  // Display coefficients in the screen
  void printCrosstalkCoefficients()
  {
    for (std::size_t i = 0; i < 9; ++i)
    {
      for (std::size_t j = 0; j < 9; ++j)
      {
        std::cout << wCoefCrossTalk[i][j] << ", ";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }

  // Calculate and display NDVI values with custom colormap
  void ndviCalculator(cv::Mat &b3, cv::Mat &b6, cv::Mat &ndvi, cv::Mat &ndviColor)
  {
    // NDVI = (NIR - RED) / (NIR + RED)
    for (std::size_t i = 0; i < BAND_HEIGHT; ++i)
    {
      for (std::size_t j = 0; j < BAND_WIDTH; ++j)
      {
        // Perform the calculations of the formula (NDVI values from -1.0 to 1.0)
        double ndviValue = (((double)b6.at<uchar>(i, j) - (double)b3.at<uchar>(i, j)) / ((double)b6.at<uchar>(i, j) + (double)b3.at<uchar>(i, j)));
        // Normalized NDVI values from 0.0 to 1.0
        double ndviNorm1 = 0.5 + 0.5 * ndviValue;
        // Normalize pixels to have values from 0 to 255
        double ndviNorm2 = 255 * ndviNorm1;
        if (ndviNorm2 > 255)
          ndviNorm2 = 255;
        else if (ndviNorm2 < 0)
          ndviNorm2 = 0;
        ndvi.at<uchar>(i, j) = ndviNorm2;

        // Colors for color-mapping in RGB palette
        // NDVI value and the coresponding RGB color for vegetation (from 0.0 to 1.0)
        //  0.00-0.05: RGB(234, 233, 189)
        //  0.05-0.10: RGB(215, 211, 148)
        //  0.10-0.15: RGB(202, 189, 119)
        //  0.15-0.20: RGB(175, 175, 77)
        //  0.20-0.30: RGB(128, 169, 5)
        //  0.30-0.40: RGB(12, 127, 0)
        //  0.40-0.50: RGB(0, 94, 0)
        //  0.50-0.60: RGB(0, 59, 1)
        //  0.60-1.00: RGB(0, 9, 0)

        // NDVI value and the coresponding RGB color for other materials such as snow and ice, water, buildings (from 0.0 to -1.0)
        //   (0.00)-(-0.05): RGB(128, 128, 128)
        //  (-0.05)-(-0.25): RGB(96, 96, 96)
        //  (-0.25)-(-0.50): RGB(64, 64, 64)
        //  (-0.50)-(-1.00): RGB(32, 32, 32)

        // NDVI coloring for different materials
        // Soil, vegetation and other materials respectievly
        // The colors are BGR not RGB
        if ((ndviValue >= 0.00) && (ndviValue <= 0.05))
        {
          ndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(189, 233, 234);
        }
        else if ((ndviValue > 0.05) && (ndviValue <= 0.10))
        {
          ndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(148, 211, 215);
        }
        else if ((ndviValue > 0.10) && (ndviValue <= 0.15))
        {
          ndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(119, 189, 202);
        }
        else if ((ndviValue > 0.15) && (ndviValue <= 0.20))
        {
          ndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(77, 175, 175);
        }
        else if ((ndviValue > 0.20) && (ndviValue <= 0.30))
        {
          ndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(5, 169, 128);
        }
        else if ((ndviValue > 0.30) && (ndviValue <= 0.40))
        {
          ndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 127, 12);
        }
        else if ((ndviValue > 0.40) && (ndviValue <= 0.50))
        {
          ndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 94, 0);
        }
        else if ((ndviValue > 0.50) && (ndviValue <= 0.60))
        {
          ndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(1, 59, 0);
        }
        else if (ndviValue > 0.60)
        {
          ndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 9, 0);
        }
        else if ((ndviValue < 0.00) && (ndviValue >= -0.05))
        {
          ndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 128, 128);
        }
        else if ((ndviValue < -0.05) && (ndviValue >= -0.25))
        {
          ndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(96, 96, 96);
        }
        else if ((ndviValue < -0.25) && (ndviValue >= -0.50))
        {
          ndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(64, 64, 64);
        }
        else if (ndviValue < -0.50)
        {
          ndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(32, 32, 32);
        }
      }
    }
  }

  // Calculate GNDVI values with custom colormap
  void gndviCalculator(cv::Mat &b1, cv::Mat &b6, cv::Mat &gndvi, cv::Mat &gndviColor)
  {
    // GNDVI = (NIR - GREEN) / (NIR + GREEN)
    // GREEN (green band 1 (560nm) is not accurate needs to be approximately 510 nm for real green band)
    for (std::size_t i = 0; i < BAND_HEIGHT; ++i)
    {
      for (std::size_t j = 0; j < BAND_WIDTH; ++j)
      {
        // Perform the calculations of the formula (GNDVI values from -1.0 to 1.0)
        double gndviValue = (((double)b6.at<uchar>(i, j) - (double)b1.at<uchar>(i, j)) / ((double)b6.at<uchar>(i, j) + (double)b1.at<uchar>(i, j)));
        // Normalized GNDVI values from 0.0 to 1.0
        double gndviNorm1 = 0.5 + 0.5 * gndviValue;
        // Normalize pixels to have values from 0 to 255
        double gndviNorm2 = 255 * gndviNorm1;
        if (gndviNorm2 > 255)
          gndviNorm2 = 255;
        else if (gndviNorm2 < 0)
          gndviNorm2 = 0;
        gndvi.at<uchar>(i, j) = gndviNorm2;

        // Colors for color-mapping in RGB palette
        // GNDVI coloring for different materials
        // Soil, vegetation and other materials respectievly
        // The colors are BGR not RGB
        if ((gndviValue >= 0.00) && (gndviValue <= 0.05))
        {
          gndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(189, 233, 234);
        }
        else if ((gndviValue > 0.05) && (gndviValue <= 0.10))
        {
          gndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(148, 211, 215);
        }
        else if ((gndviValue > 0.10) && (gndviValue <= 0.15))
        {
          gndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(119, 189, 202);
        }
        else if ((gndviValue > 0.15) && (gndviValue <= 0.20))
        {
          gndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(77, 175, 175);
        }
        else if ((gndviValue > 0.20) && (gndviValue <= 0.30))
        {
          gndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(5, 169, 128);
        }
        else if ((gndviValue > 0.30) && (gndviValue <= 0.40))
        {
          gndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 127, 12);
        }
        else if ((gndviValue > 0.40) && (gndviValue <= 0.50))
        {
          gndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 94, 0);
        }
        else if ((gndviValue > 0.50) && (gndviValue <= 0.60))
        {
          gndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(1, 59, 0);
        }
        else if (gndviValue > 0.60)
        {
          gndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 9, 0);
        }
        else if ((gndviValue < 0.00) && (gndviValue >= -0.05))
        {
          gndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 128, 128);
        }
        else if ((gndviValue < -0.05) && (gndviValue >= -0.25))
        {
          gndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(96, 96, 96);
        }
        else if ((gndviValue < -0.25) && (gndviValue >= -0.50))
        {
          gndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(64, 64, 64);
        }
        else if (gndviValue < -0.50)
        {
          gndviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(32, 32, 32);
        }
      }
    }
  }

  // Calculate SAVI values with custom colormap
  void saviCalculator(cv::Mat &b3, cv::Mat &b6, cv::Mat &savi, cv::Mat &saviColor)
  {
    // SAVI = ((1 + L)(NIR - RED)) / (NIR + RED + L)
    double L = 0.5;
    for (std::size_t i = 0; i < BAND_HEIGHT; ++i)
    {
      for (std::size_t j = 0; j < BAND_WIDTH; ++j)
      {
        // Perform the calculations of the formula (SAVI values from -1.0 to 1.0)
        double saviValue = ((1.0 + L) * ((double)b6.at<uchar>(i, j) - (double)b3.at<uchar>(i, j))) / ((double)b6.at<uchar>(i, j) + (double)b3.at<uchar>(i, j) + L);
        // Normalized SAVI values from 0.0 to 1.0
        double saviNorm1 = 0.5 + 0.5 * saviValue;
        // Normalize pixels to have values from 0 to 255
        double saviNorm2 = 255 * saviNorm1;
        if (saviNorm2 > 255)
          saviNorm2 = 255;
        else if (saviNorm2 < 0)
          saviNorm2 = 0;
        savi.at<uchar>(i, j) = saviNorm2;

        // Colors for color-mapping in RGB palette
        // SAVI coloring for different materials
        // Soil, vegetation and other materials respectievly
        // The colors are BGR not RGB
        if ((saviValue >= 0.00) && (saviValue <= 0.05))
        {
          saviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(189, 233, 234);
        }
        else if ((saviValue > 0.05) && (saviValue <= 0.10))
        {
          saviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(148, 211, 215);
        }
        else if ((saviValue > 0.10) && (saviValue <= 0.15))
        {
          saviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(119, 189, 202);
        }
        else if ((saviValue > 0.15) && (saviValue <= 0.20))
        {
          saviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(77, 175, 175);
        }
        else if ((saviValue > 0.20) && (saviValue <= 0.30))
        {
          saviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(5, 169, 128);
        }
        else if ((saviValue > 0.30) && (saviValue <= 0.40))
        {
          saviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 127, 12);
        }
        else if ((saviValue > 0.40) && (saviValue <= 0.50))
        {
          saviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 94, 0);
        }
        else if ((saviValue > 0.50) && (saviValue <= 0.60))
        {
          saviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(1, 59, 0);
        }
        else if (saviValue > 0.60)
        {
          saviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 9, 0);
        }
        else if ((saviValue < 0.00) && (saviValue >= -0.05))
        {
          saviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 128, 128);
        }
        else if ((saviValue < -0.05) && (saviValue >= -0.25))
        {
          saviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(96, 96, 96);
        }
        else if ((saviValue < -0.25) && (saviValue >= -0.50))
        {
          saviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(64, 64, 64);
        }
        else if (saviValue < -0.50)
        {
          saviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(32, 32, 32);
        }
      }
    }
  }

  // Calculate GSAVI values with custom colormap
  void gsaviCalculator(cv::Mat &b1, cv::Mat &b6, cv::Mat &gsavi, cv::Mat &gsaviColor)
  {
    // GSAVI = ((1 + L)(NIR - GREEN)) / (NIR + GREEN + L)
    // GREEN (green band 1 (560nm) is not accurate needs to be approximately 510 nm for real green band)
    double L = 0.5;
    for (std::size_t i = 0; i < BAND_HEIGHT; ++i)
    {
      for (std::size_t j = 0; j < BAND_WIDTH; ++j)
      {
        // Perform the calculations of the formula (GSAVI values from -1.0 to 1.0)
        double gsaviValue = ((1.0 + L) * ((double)b6.at<uchar>(i, j) - (double)b1.at<uchar>(i, j))) / ((double)b6.at<uchar>(i, j) + (double)b1.at<uchar>(i, j) + L);
        // Normalized GSAVI values from 0.0 to 1.0
        double gsaviNorm1 = 0.5 + 0.5 * gsaviValue;
        // Normalize pixels to have values from 0 to 255
        double gsaviNorm2 = 255 * gsaviNorm1;
        if (gsaviNorm2 > 255)
          gsaviNorm2 = 255;
        else if (gsaviNorm2 < 0)
          gsaviNorm2 = 0;
        gsavi.at<uchar>(i, j) = gsaviNorm2;

        // Colors for color-mapping in RGB palette
        // GSAVI coloring for different materials
        // Soil, vegetation and other materials respectievly
        // The colors are BGR not RGB
        if ((gsaviValue >= 0.00) && (gsaviValue <= 0.05))
        {
          gsaviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(189, 233, 234);
        }
        else if ((gsaviValue > 0.05) && (gsaviValue <= 0.10))
        {
          gsaviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(148, 211, 215);
        }
        else if ((gsaviValue > 0.10) && (gsaviValue <= 0.15))
        {
          gsaviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(119, 189, 202);
        }
        else if ((gsaviValue > 0.15) && (gsaviValue <= 0.20))
        {
          gsaviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(77, 175, 175);
        }
        else if ((gsaviValue > 0.20) && (gsaviValue <= 0.30))
        {
          gsaviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(5, 169, 128);
        }
        else if ((gsaviValue > 0.30) && (gsaviValue <= 0.40))
        {
          gsaviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 127, 12);
        }
        else if ((gsaviValue > 0.40) && (gsaviValue <= 0.50))
        {
          gsaviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 94, 0);
        }
        else if ((gsaviValue > 0.50) && (gsaviValue <= 0.60))
        {
          gsaviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(1, 59, 0);
        }
        else if (gsaviValue > 0.60)
        {
          gsaviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 9, 0);
        }
        else if ((gsaviValue < 0.00) && (gsaviValue >= -0.05))
        {
          gsaviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 128, 128);
        }
        else if ((gsaviValue < -0.05) && (gsaviValue >= -0.25))
        {
          gsaviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(96, 96, 96);
        }
        else if ((gsaviValue < -0.25) && (gsaviValue >= -0.50))
        {
          gsaviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(64, 64, 64);
        }
        else if (gsaviValue < -0.50)
        {
          gsaviColor.at<cv::Vec3b>(i, j) = cv::Vec3b(32, 32, 32);
        }
      }
    }
  }

  // MCARI (Modified Chlorophyll Absorption Ratio Index)
  void mcariCalculator(cv::Mat &b1, cv::Mat &b4, cv::Mat &b5, cv::Mat &mcari)
  {
    // MCARI = ((R700 - R670) - 0.2 * (R700 - R550 )) * (R700 / R670)
    for (std::size_t i = 0; i < BAND_HEIGHT; ++i)
    {
      for (std::size_t j = 0; j < BAND_WIDTH; ++j)
      {
        // Perform the calculations of the formula
        double mcariValue = (((double)b5.at<uchar>(i, j) - (double)b4.at<uchar>(i, j)) - 0.2 * ((double)b5.at<uchar>(i, j) - (double)b1.at<uchar>(i, j))) * ((double)b5.at<uchar>(i, j) / (double)b4.at<uchar>(i, j));
        // Normalize pixels to have values from 0 to 255
        double mcariNorm = 255 * mcariValue;
        if (mcariNorm > 255)
          mcariNorm = 255;
        else if (mcariNorm < 0)
          mcariNorm = 0;
        mcari.at<uchar>(i, j) = mcariNorm;
      }
    }
  }

  // MSR (Modified Simple Ratio)
  void msrCalculator(cv::Mat &b3, cv::Mat &b6, cv::Mat &msr, cv::Mat &msrColor)
  {
    // MSR = (NIR/RED - 1) / (sqrt(NIR/RED) + 1)
    for (std::size_t i = 0; i < BAND_HEIGHT; ++i)
    {
      for (std::size_t j = 0; j < BAND_WIDTH; ++j)
      {
        // Perform the calculations of the formula (MSR values from -1.0 to 1.0)
        double msrValue = (((double)b6.at<uchar>(i, j) / (double)b3.at<uchar>(i, j) - 1.0)) / (sqrt((double)b6.at<uchar>(i, j) / (double)b3.at<uchar>(i, j)) + 1.0);
        // Normalized MSR values from 0.0 to 1.0
        double msrNorm1 = 0.5 + 0.5 * msrValue;
        // Normalize pixels to have values from 0 to 255
        double msrNorm2 = 255 * msrNorm1;
        if (msrNorm2 > 255)
          msrNorm2 = 255;
        else if (msrNorm2 < 0)
          msrNorm2 = 0;
        msr.at<uchar>(i, j) = msrNorm2;

        // Colors for color-mapping in RGB palette
        // MSR coloring for different materials
        // Soil, vegetation and other materials respectievly
        // The colors are BGR not RGB
        if ((msrValue >= 0.00) && (msrValue <= 0.05))
        {
          msrColor.at<cv::Vec3b>(i, j) = cv::Vec3b(189, 233, 234);
        }
        else if ((msrValue > 0.05) && (msrValue <= 0.10))
        {
          msrColor.at<cv::Vec3b>(i, j) = cv::Vec3b(148, 211, 215);
        }
        else if ((msrValue > 0.10) && (msrValue <= 0.15))
        {
          msrColor.at<cv::Vec3b>(i, j) = cv::Vec3b(119, 189, 202);
        }
        else if ((msrValue > 0.15) && (msrValue <= 0.20))
        {
          msrColor.at<cv::Vec3b>(i, j) = cv::Vec3b(77, 175, 175);
        }
        else if ((msrValue > 0.20) && (msrValue <= 0.30))
        {
          msrColor.at<cv::Vec3b>(i, j) = cv::Vec3b(5, 169, 128);
        }
        else if ((msrValue > 0.30) && (msrValue <= 0.40))
        {
          msrColor.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 127, 12);
        }
        else if ((msrValue > 0.40) && (msrValue <= 0.50))
        {
          msrColor.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 94, 0);
        }
        else if ((msrValue > 0.50) && (msrValue <= 0.60))
        {
          msrColor.at<cv::Vec3b>(i, j) = cv::Vec3b(1, 59, 0);
        }
        else if (msrValue > 0.60)
        {
          msrColor.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 9, 0);
        }
        else if ((msrValue < 0.00) && (msrValue >= -0.05))
        {
          msrColor.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 128, 128);
        }
        else if ((msrValue < -0.05) && (msrValue >= -0.25))
        {
          msrColor.at<cv::Vec3b>(i, j) = cv::Vec3b(96, 96, 96);
        }
        else if ((msrValue < -0.25) && (msrValue >= -0.50))
        {
          msrColor.at<cv::Vec3b>(i, j) = cv::Vec3b(64, 64, 64);
        }
        else if (msrValue < -0.50)
        {
          msrColor.at<cv::Vec3b>(i, j) = cv::Vec3b(32, 32, 32);
        }
      }
    }
  }

  // Calculate TVI (Triangular Vegetation Index)
  // Calculate MTVI1 (Modified Triangular Vegetation Index 1)
  // Calculate MTVI2 (Modified Triangular Vegetation Index 2)
  void tviCalculator(cv::Mat &b1, cv::Mat &b3, cv::Mat &b6, cv::Mat &b4, cv::Mat &b7, cv::Mat &tvi, cv::Mat &mtvi1, cv::Mat &mtvi2)
  {
    for (std::size_t i = 0; i < BAND_HEIGHT; ++i)
    {
      for (std::size_t j = 0; j < BAND_WIDTH; ++j)
      {
        // TVI = 0.5 * (120.0 * (NIR - GREEN) - 200.0 * (RED - GREEN))
        double tviValue = 0.5 * ((120.0 * ((double)b6.at<uchar>(i, j) - (double)b1.at<uchar>(i, j))) - (200.0 * ((double)b3.at<uchar>(i, j) - (double)b1.at<uchar>(i, j))));
        // Normalize pixels to have values from 0 to 255
        double tviNorm = 255 * tviValue;
        if (tviNorm > 255)
          tviNorm = 255;
        else if (tviNorm < 0)
          tviNorm = 0;
        tvi.at<uchar>(i, j) = tviNorm;
        // MTVI1 = 1.2 * (1.2 * (R800 - R550) - 2.5 * (R670 - R550))
        double mtvi1Value = 1.2 * ((1.2 * ((double)b7.at<uchar>(i, j) - (double)b1.at<uchar>(i, j))) - (2.5 * ((double)b4.at<uchar>(i, j) - (double)b1.at<uchar>(i, j))));
        double mtvi1Norm = 255 * mtvi1Value;
        if (mtvi1Norm > 255)
          mtvi1Norm = 255;
        else if (mtvi1Norm < 0)
          mtvi1Norm = 0;
        mtvi1.at<uchar>(i, j) = mtvi1Norm;
        // MTVI2 = (1.5 * (1.2 * (R800 - R550) - 2.5 * (R670 - R550)))/ (sqrt((2.0 * R800 + 1) ^ 2.0 - (6.0 * R800 - 5.0 * sqrt(R670)) - 0.5))
        double mtvi2Value = (1.5 * ((1.2 * ((double)b7.at<uchar>(i, j) - (double)b1.at<uchar>(i, j))) - (2.5 * ((double)b4.at<uchar>(i, j) - (double)b1.at<uchar>(i, j))))) / (sqrt(pow((2.0 * (double)b7.at<uchar>(i, j) + 1.0), 2.0) - (6.0 * (double)b7.at<uchar>(i, j) - 5.0 * sqrt((double)b4.at<uchar>(i, j))) - 0.5));
        // Normalized MTVI2 values from 0.0 to 1.0
        double mtvi2Norm1 = 0.5 + 0.5 * mtvi2Value;
        // Normalize pixels to have values from 0 to 255
        double mtvi2Norm2 = 255 * mtvi2Norm1;
        if (mtvi2Norm2 > 255)
          mtvi2Norm2 = 255;
        else if (mtvi2Norm2 < 0)
          mtvi2Norm2 = 0;
        mtvi2.at<uchar>(i, j) = mtvi2Norm2;
      }
    }
  }

  // Image segmentation
  void segmentation(cv::Mat &image, cv::Mat &erdImage, cv::Mat &segImage)
  {
    // Thresholding with OTSU
    cv::threshold(image, segImage, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    // Erosion removes noise
    cv::Mat erodedImage;
    int erosionSize = 2;
    int erosionType = cv::MORPH_ELLIPSE;
    cv::Mat el2 = cv::getStructuringElement(erosionType, cv::Size(2 * erosionSize + 1, 2 * erosionSize + 1), cv::Point(erosionSize, erosionSize));
    cv::erode(segImage, erodedImage, el2);
    // Dilation fills holes of the region of interest and expands it
    int dilatationSize = 3;
    int dilatationType = cv::MORPH_ELLIPSE;
    cv::Mat el1 = cv::getStructuringElement(dilatationType, cv::Size(2 * dilatationSize + 1, 2 * dilatationSize + 1), cv::Point(dilatationSize, dilatationSize));
    cv::dilate(erodedImage, erdImage, el1);
  }

  // Merge all bands in a single image
  void mergeBands(cv::Mat images[], cv::Mat &merged)
  {
    // Create 1278x1017 matrix for window
    // Copy small images into big matrix, declare the start of every sub-image and the size
    // From Band 1 to Band 8 - From lower wavelength to higher wavelength - Band 9 = Panchromaic filtered band

    //  Color codes / Bands / Pixels as descibed in the documentation.
    //    Color code 0 - Band 1: Pixel images[2] (Lowest wavelenght filter)
    //    Color code 1 - Band 2: Pixel images[5]
    //    Color code 2 - Band 3: Pixel images[4]
    //    Color code 3 - Band 4: Pixel images[3]
    //    Color code 4 - Band 5: Pixel images[0]
    //    Color code 5 - Band 6: Pixel images[6]
    //    Color code 6 - Band 7: Pixel images[7]
    //    Color code 7 - Band 8: Pixel images[8] (Highest wavelenght filter)
    //    Color code 8 - Band 9: Pixel images[1] (Panchromatic filter)

    images[8].copyTo(merged(cv::Rect(0, 0, 426, 339)));
    images[2].copyTo(merged(cv::Rect(426, 0, 426, 339)));
    images[5].copyTo(merged(cv::Rect(852, 0, 426, 339)));
    images[7].copyTo(merged(cv::Rect(0, 339, 426, 339)));
    images[1].copyTo(merged(cv::Rect(426, 339, 426, 339)));
    images[4].copyTo(merged(cv::Rect(852, 339, 426, 339)));
    images[6].copyTo(merged(cv::Rect(0, 678, 426, 339)));
    images[0].copyTo(merged(cv::Rect(426, 678, 426, 339)));
    images[3].copyTo(merged(cv::Rect(852, 678, 426, 339)));
  }

  // Image publisher
  void publisher(cv::Mat images[], cv::Mat &ndvi, cv::Mat &ndviColor, cv::Mat &bandsGrid)
  {
    // Bilinear Interpolation for resizing band 3 to 1278x1017
    cv::Mat b3I;
    cv::resize(images[4], b3I, cv::Size(1278, 1017), 0, 0, cv::INTER_LINEAR);
    CI3 = CAMERA_INFO;
    CI3.width = CRAW_WIDTH;
    CI3.height = CRAW_HEIGHT;

    // Create header for every published image
    CAMERA_INFO.header.stamp = ros::Time::now();

    // Band images publisher
    for (std::size_t i = 0; i < 9; ++i)
    {
      // Convert image to sensor message image
      sensor_msgs::ImagePtr imgCVB = cv_bridge::CvImage(CAMERA_INFO.header, IME_B, images[i]).toImageMsg();
      // Publish images for bands topics
      pub_b[bandsOrder[i]].publish(imgCVB);
    }
    // Convert image to sensor message image for NDVI and the other images
    sensor_msgs::ImagePtr imgCV3I = cv_bridge::CvImage(CI3.header, IME_B, b3I).toImageMsg();
    sensor_msgs::ImagePtr imgCVN = cv_bridge::CvImage(CAMERA_INFO.header, IME_B, ndvi).toImageMsg();
    sensor_msgs::ImagePtr imgCVNC = cv_bridge::CvImage(CAMERA_INFO.header, IME_C, ndviColor).toImageMsg();
    sensor_msgs::ImagePtr imgCVBG = cv_bridge::CvImage(std_msgs::Header(), IME_B, bandsGrid).toImageMsg();
    // Publish NDVI images
    pub_n.publish(imgCVN);
    pub_nc.publish(imgCVNC);
    // Publish other images
    pub_3i.publish(imgCV3I);
    pub_bg.publish(imgCVBG);
    // Publish camera info for bands topics
    pub_c.publish(CAMERA_INFO);
  }

  // Shut down everything
  void stop()
  {
    sub_m.shutdown();
    sub_c.shutdown();
    for (size_t i = 0; i < 9; ++i)
    {
      pub_b[i].shutdown();
    }
    pub_3i.shutdown();
    pub_n.shutdown();
    pub_nc.shutdown();
    pub_bg.shutdown();
    pub_c.shutdown();
    cv::destroyAllWindows();
    ros::shutdown();
    std::cout << "Band separator shutted down" << std::endl;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "band_separator");
  BandSeparator bandSeparator(argc, argv);
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
  bandSeparator.stop();
  return 0;
}
