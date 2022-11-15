#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>


static const std::string ROI_WINDOW = "ROI";
static const std::string TEMPLATE_WINDOW = "Template img";
static const std::string MASK_WINDOW = "Mask";
static const std::string HIST_WINDOW = "Histogram";
static const std::string TRACK_WINDOW = "Tracking";
static const std::string FLOW_WINDOW = "Optical Flow";

const int max_value = 255;
int low_S = 0;
int high_S = max_value;


static void on_low_S_thresh_trackbar(int, void *)
{
  low_S = min(high_S-1, low_S);
  cv::setTrackbarPos("Low S", MASK_WINDOW, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
  high_S = max(high_S, low_S+1);
  cv::setTrackbarPos("High S", MASK_WINDOW, high_S);
}


// Real code
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_top;
  image_transport::Subscriber image_sub_bot;
  image_transport::Publisher image_pub_;

  cv::Mat template_img;
  cv::Mat crop_image;
  cv::Mat crop_image_hsv;
  std::vector<cv::Mat> split_image;
  cv::Mat mask_image;
  cv::Mat hist_image;
  cv::Mat channels[3];
  cv::Mat backproj_image;

public:
  ImageConverter()
    : it_(nh_)
  {
  // Subscrive to input video feed and publish output video feed
  image_sub_top = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &ImageConverter::imageCb_top, this);
//   image_sub_bot = it_.subscribe("/nao_robot/camera/bottom/camera/image_raw", 1, &ImageConverter::imageCb_bot, this);
  image_pub_ = it_.advertise("/image_converter/output_video", 1);

//    cv::namedWindow(TEMPLATE_WINDOW);
//    cv::moveWindow(TEMPLATE_WINDOW,0,0);
//    cv::namedWindow(ROI_WINDOW);
//    cv::moveWindow(ROI_WINDOW,0,500);
  cv::namedWindow(MASK_WINDOW);
  cv::moveWindow(MASK_WINDOW,0,500);
  // cv::createTrackbar("Low S", MASK_WINDOW, &low_S, max_value, on_low_S_thresh_trackbar);
  // cv::createTrackbar("High S", MASK_WINDOW, &high_S, max_value, on_high_S_thresh_trackbar);
  cv::namedWindow(HIST_WINDOW);
  cv::moveWindow(HIST_WINDOW,500,0);
  cv::namedWindow(TRACK_WINDOW);
  cv::moveWindow(TRACK_WINDOW,500,500);
  cv::namedWindow(FLOW_WINDOW);
  cv::moveWindow(FLOW_WINDOW,1000,0);

  }
  
  ~ImageConverter()
  {
  }

  void imageCb_top(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

    cv::Mat srcimg = cv_ptr->image;

    // Camera parameters
    cv::Mat dist(1,5,CV_32FC1);
    dist.at<float>(0,0)=-0.066494;
    dist.at<float>(0,1)=0.095481;
    dist.at<float>(0,2)=-0.000279;
    dist.at<float>(0,3)=0.002292;
    dist.at<float>(0,4)=0.000000;
    cv::Mat cameraP(3,3,CV_32FC1);
    cameraP.at<float>(0,0)=551.543059;
    cameraP.at<float>(0,1)=0.000000;
    cameraP.at<float>(0,2)=327.382898;
    cameraP.at<float>(1,0)=0.000000;
    cameraP.at<float>(1,1)=553.736023;
    cameraP.at<float>(1,2)=225.026380;
    cameraP.at<float>(2,0)=0.000000;
    cameraP.at<float>(2,1)=0.000000;
    cameraP.at<float>(2,2)=1.000000;

    TheCameraParameters.setParams(cameraP,dist,Size(640,480));
    TheCameraParameters.resize( Size(640,480));

    // Task 3
    // read image
    template_img = cv::imread("/home/hrsd/tutorial_4_ws/templateImg.jpg");
    // Select ROI
    // cv::Rect2d region = cv::selectROI(template_img);
    cv::Rect crop_area(168,99,57,61);
    // Crop image
    crop_image = template_img(crop_area);

    // Task 4 Histogram
    cv::cvtColor(crop_image, crop_image_hsv, CV_BGR2HSV);
    cv::split(crop_image_hsv, split_image);
    cv::threshold(split_image[1], mask_image, 170, 255.0, CV_THRESH_BINARY);

    // parameters for histogram
    int hbins = 180;
    int histSize[] = {hbins};;
    // hue varies from 0 to 179, see cvtColor
    float hranges[] = { 0, 180 };
    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
    // float sranges[] = { 0, 256 };
    const float* ranges[] = { hranges };
    // we compute the histogram from the 0-st channels, hue channel
    int channels[] = {0};

    // create histogram
    cv::Mat historgram;
    cv::calcHist( &split_image[0], 1, channels, mask_image,
          historgram, 1, histSize, ranges,
          true, // the histogram is uniform
          false );
    cv::normalize(historgram, historgram, 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );

    // parameters of hist image, for visualization of historgram
    int hist_w = 512, hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize[0] );
    cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

    for( int i = 1; i < histSize[0]; i++ )
    {
      line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(historgram.at<float>(i-1)) ),
            cv::Point( bin_w*(i), hist_h - cvRound(historgram.at<float>(i)) ),
            cv::Scalar( 0, 255, 0), 2, 8, 0  );
    }

    // Task 5 Backprojection
    cv::calcBackProject(&crop_image_hsv, 1, channels, historgram, backproj_image, ranges);
    
    // Task 6 Meanshift
    cv::TermCriteria term_crit(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 1);
    meanShift(backproj_image,crop_area,term_crit);
    cv::rectangle(template_img, crop_area, 255, 2);

    // Task 7 Camshift
    cv::RotatedRect rot_rect = CamShift(backproj_image, crop_area, term_crit);
    // Draw it on image
    cv::Point2f points[4];
    rot_rect.points(points);
    for (int i = 0; i < 4; i++) {
      line(template_img, points[i], points[(i+1)%4], 255, 2);
    }
    
    // Task 8&9 Optical Flow


    // Task 10 3D marker position
    aruco::MarkerDetector mark_detector;
    std::vector<Marker> detected_marker;
    mark_detector.detect(srcimg,detected_marker,TheCameraParameters,0.01,false)
    for (unsigned i = 0; i< detectedMarkers.size(); i++ )
    {
      mark_detector.draw(srcimg,detected_marker);
    }

    // Update GUI Window
    cv::imshow(ROI_WINDOW, crop_image_hsv);
    cv::imshow(MASK_WINDOW, mask_image);
    cv::imshow(HIST_WINDOW, histImage);
    cv::imshow(TRACK_WINDOW, template_img);
    cv::waitKey(3);

      
    // cv::imshow(TEMPLATE_WINDOW, cv_ptr->image);
      

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

    

};

  
  
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
