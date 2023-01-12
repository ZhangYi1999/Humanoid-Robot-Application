// tutorial 4 by Yinlei Song, Yi Zhang, Tao ma, Chongyu Zhang

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
static const std::string MEANTRACK_WINDOW = "Meanshift Tracking";
static const std::string CAMTRACK_WINDOW = "Camshift Tracking";
static const std::string FLOW_WINDOW = "Optical Flow";
static const std::string MARKER_WINDOW = "3D marker position";


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

  // params for optical flow
  int flag;
  std::vector<cv::Point2f> p0, p1;
  cv::Mat mask;
  std::vector<uchar> status;
  std::vector<float> err;

  cv::Mat template_img;
  cv::Mat hsv_image;
  cv::Mat crop_image;
  cv::Mat crop_image_hsv;
  std::vector<cv::Mat> split_image;
  cv::Mat mask_image;
  cv::Mat hist_image;
  cv::Mat channels[3];
  cv::Mat backproj_image;
  cv::Mat old_grey;
  cv::Mat gray;
  std::vector<cv::Scalar> colors;

  cv::Mat dist;
  cv::Mat cameraP;
  aruco::CameraParameters TheCameraParameters;

public:
  ImageConverter()
    : it_(nh_)
  {
  // Subscrive to input video feed and publish output video feed
  image_sub_top = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &ImageConverter::imageCb_top, this);
  // image_sub_bot = it_.subscribe("/nao_robot/camera/bottom/camera/image_raw", 1, &ImageConverter::imageCb_bot, this);
  image_pub_ = it_.advertise("/image_converter/output_video", 1);

  cv::namedWindow(TEMPLATE_WINDOW);
  cv::moveWindow(TEMPLATE_WINDOW,0,0);
  cv::namedWindow(ROI_WINDOW);
  cv::moveWindow(ROI_WINDOW,0,500);
  cv::namedWindow(MASK_WINDOW);
  cv::moveWindow(MASK_WINDOW,0,500);
  // cv::createTrackbar("Low S", MASK_WINDOW, &low_S, max_value, on_low_S_thresh_trackbar);
  // cv::createTrackbar("High S", MASK_WINDOW, &high_S, max_value, on_high_S_thresh_trackbar);
  cv::namedWindow(HIST_WINDOW);
  cv::moveWindow(HIST_WINDOW,500,0);
  cv::namedWindow(MEANTRACK_WINDOW);
  cv::moveWindow(MEANTRACK_WINDOW,500,500);
  cv::namedWindow(CAMTRACK_WINDOW);
  cv::moveWindow(CAMTRACK_WINDOW,1000,1000);
  cv::namedWindow(FLOW_WINDOW);
  cv::moveWindow(FLOW_WINDOW,1000,0);
  cv::namedWindow(MARKER_WINDOW);
  cv::moveWindow(MARKER_WINDOW,1000,500);

  
  dist = cv::Mat(1,5,CV_32FC1);
  dist.at<float>(0,0)=-0.066494;
  dist.at<float>(0,1)=0.095481;
  dist.at<float>(0,2)=-0.000279;
  dist.at<float>(0,3)=0.002292;
  dist.at<float>(0,4)=0.000000;
  
  cameraP = cv::Mat(3,3,CV_32FC1);
  cameraP.at<float>(0,0)=551.543059;
  cameraP.at<float>(0,1)=0.000000;
  cameraP.at<float>(0,2)=327.382898;
  cameraP.at<float>(1,0)=0.000000;
  cameraP.at<float>(1,1)=553.736023;
  cameraP.at<float>(1,2)=225.026380;
  cameraP.at<float>(2,0)=0.000000;
  cameraP.at<float>(2,1)=0.000000;
  cameraP.at<float>(2,2)=1.000000;

  TheCameraParameters.setParams(cameraP,dist,cv::Size(640,480));
  TheCameraParameters.resize( cv::Size(640,480));

  flag = 0;
      
    cv::RNG rng;
    for(int i = 0; i < 100; i++)
    {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors.push_back(cv::Scalar(r,g,b));
    }

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
    cv::Mat srcimg = cv_ptr->image.clone();

    cv::cvtColor(cv_ptr->image, hsv_image ,CV_BGR2HSV);


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
    cv::calcBackProject(&hsv_image, 1, channels, historgram, backproj_image, ranges);
    
    cv::TermCriteria term_crit(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 1);
    cv::Mat meanshift_image = srcimg.clone();
    
    // Task 6 Meanshift
    cv::meanShift(backproj_image,crop_area,term_crit);
    cv::rectangle(meanshift_image, crop_area, cv::Scalar(0,255,0), 2);

    // Task 7 Camshift
    cv::Mat camshift_image = srcimg.clone();
    cv::RotatedRect rot_rect = CamShift(backproj_image, crop_area, term_crit);
    // Draw it on image
    cv::Point2f points[4];
    rot_rect.points(points);
    for (int i = 0; i < 4; i++) {
      line(camshift_image, points[i], points[(i+1)%4], cv::Scalar(0,0,255), 2);
    }
    
    // Task 8&9 Optical Flow
    cv::Mat flowimg = cv_ptr->image.clone();
    cv::Mat gray;
    cv::cvtColor(flowimg, gray, CV_RGB2GRAY);

    if(flag==0)
      {
        std::cout << "run once" << std::endl;
        old_grey = gray;
        goodFeaturesToTrack(old_grey, p0, 20, 0.01, 2, cv::Mat(), 7, true, 0.04);
        mask = cv::Mat::zeros(flowimg.size(), flowimg.type());
      }

    //conner detection
    std::vector<cv::Point2f> good_new;
    if(flag==1)
    {
      cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
      calcOpticalFlowPyrLK(old_grey ,gray, p0, p1, status, err, cv::Size(15,15), 2, criteria);
      for(uint i = 0; i < p0.size(); i++)
      {
          // Select good points
          if(status[i] == 1) {
              good_new.push_back(p1[i]);
              // draw the tracks
              cv::line(mask, p1[i], p0[i], colors[i],2);
              cv::circle(flowimg, p1[i], 1, cv::Scalar(0,0,255) ,-1);
          }
      }

      old_grey = gray;
      p0 = good_new;

    }

    cv::Mat flowresult;
    add(flowimg, mask, flowresult);

    // Task 10 3D marker position
    
    cv::Mat marker_img = srcimg.clone();
    aruco::MarkerDetector mark_detector;
    std::vector<aruco::Marker> detected_marker;
    mark_detector.detect(srcimg,detected_marker,TheCameraParameters,0.01,false);
    for (unsigned i = 0; i< detected_marker.size(); i++ )
    {
      detected_marker[i].draw(marker_img,cv::Scalar(255,0,255),3);
    }



    // Update GUI Window
    cv::imshow(ROI_WINDOW, crop_image);
    cv::imshow(MASK_WINDOW, mask_image);
    cv::imshow(HIST_WINDOW, histImage);
    cv::imshow(MEANTRACK_WINDOW, meanshift_image);
    cv::imshow(CAMTRACK_WINDOW, camshift_image);
    cv::imshow(FLOW_WINDOW, flowresult);
    cv::imshow(MARKER_WINDOW, marker_img);
    cv::waitKey(3);

      
    // cv::imshow(TEMPLATE_WINDOW, cv_ptr->image);
      
    flag = 1;
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
