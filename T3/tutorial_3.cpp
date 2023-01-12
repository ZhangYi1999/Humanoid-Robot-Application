#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string TOP_WINDOW = "Top camera";
static const std::string BOTTOM_WINDOW = "Bottom camera";
static const std::string COLOR_WINDOW = "Color extraction";
static const std::string BLOB_WINDOW = "Blob extraction";
static const std::string CIRCLE_WINDOW = "Circular shapes";

// Real code
class ImageConverter
{
     ros::NodeHandle nh_;
     image_transport::ImageTransport it_;
     image_transport::Subscriber image_sub_top;
     image_transport::Subscriber image_sub_bot;
     image_transport::Publisher image_pub_;
   
   public:
     ImageConverter()
       : it_(nh_)
     {
       // Subscrive to input video feed and publish output video feed
       image_sub_top = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &ImageConverter::imageCb_top, this);
       image_sub_bot = it_.subscribe("/nao_robot/camera/bottom/camera/image_raw", 1, &ImageConverter::imageCb_bot, this);
       image_pub_ = it_.advertise("/image_converter/output_video", 1);
   
       cv::namedWindow(TOP_WINDOW);
       cv::moveWindow(TOP_WINDOW,0,0);
       cv::namedWindow(BOTTOM_WINDOW);
       cv::moveWindow(BOTTOM_WINDOW,0,500);
       cv::namedWindow(COLOR_WINDOW);
       cv::moveWindow(COLOR_WINDOW,500,0);
       cv::namedWindow(CIRCLE_WINDOW);
       cv::moveWindow(CIRCLE_WINDOW,500,500);
       cv::namedWindow(BLOB_WINDOW);
       cv::moveWindow(BLOB_WINDOW,1000,0);
     }
   
     ~ImageConverter()
     {
       cv::destroyWindow(TOP_WINDOW);
     }
   
     void imageCb_top(const sensor_msgs::ImageConstPtr& msg)
     {
       cv_bridge::CvImagePtr cv_ptr;
       cv::Mat hsv_image;
       cv::Mat red_image;
       cv::Mat green_image;
       cv::Mat blue_image;
       cv::Mat dilate_image;
       cv::Mat coarse_blob_image;
       cv::Mat blob_image;
       std::vector<std::vector<cv::Point> >  contours;
       std::vector<cv::Vec4i> hierarchy;
       cv::Point circle_center;


       try
       {
         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       }
       catch (cv_bridge::Exception& e)
       {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
       }
       
       // convert BGR image to HSV image
       cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

       // extract red green and blue pixels
        cv::inRange(hsv_image, cv::Scalar(0, 159, 0), cv::Scalar(19, 255, 255), red_image);
      //  cv::inRange(hsv_image, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), green_image);
       cv::inRange(hsv_image, cv::Scalar(35,45,46), cv::Scalar(77,255,255), green_image);
       cv::inRange(hsv_image, cv::Scalar(100,43,46), cv::Scalar(124,255,255), blue_image);
       
       // blob
       cv::dilate(red_image, dilate_image, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
       cv::erode(dilate_image, coarse_blob_image, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

       // find contour
       cv::findContours(coarse_blob_image,contours,hierarchy,cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

       double last_area = 0;
       std::vector<cv::Point> largest_contours;
       size_t n;
       for (size_t t = 0; t < contours.size(); ++t)
       {
         double area = cv::contourArea(contours[t]);     
         if (area < last_area)         
           continue;
         else
         {
           last_area = area;
           largest_contours = contours[t];
           n=t;
         }  
       }
        cv::Moments mu = cv::moments( largest_contours );
    
         
        int x = mu.m10/mu.m00;
        int y = mu.m01/mu.m00;
        circle_center = cv::Point(x, y); 
        blob_image = cv::Mat::zeros(coarse_blob_image.size(), CV_8UC3);
        cv::drawContours( blob_image, contours, n, cv::Scalar(0,0,255), cv::FILLED);
        cv::circle(blob_image, circle_center, 2, cv::Scalar(0, 255, 0), 2, 8, 0);




       // Update GUI Window
       
       cv::imshow(TOP_WINDOW, cv_ptr->image);
       cv::imshow(COLOR_WINDOW, red_image);
       cv::imshow(BLOB_WINDOW, blob_image);
       cv::waitKey(3);


   
       // Output modified video stream
       image_pub_.publish(cv_ptr->toImageMsg());
     }

     void imageCb_bot(const sensor_msgs::ImageConstPtr& msg)
     {
       cv_bridge::CvImagePtr cv_ptr;
       cv::Mat gray_image;
       cv::Mat result_image;
       std::vector<cv::Vec3f> circles;

       try
       {
         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       }
       catch (cv_bridge::Exception& e)
       {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
       }

       result_image = cv_ptr->image.clone();

       cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
       // smooth it, otherwise a lot of false circles may be detected
       cv::GaussianBlur( gray_image, gray_image, cv::Size(9, 9), 2, 2 );
    
       cv::HoughCircles(gray_image, circles, cv::HOUGH_GRADIENT, 2, gray_image.rows/4, 180, 100, 20, 250);
       for( size_t i = 0; i < circles.size(); i++ )
       {
          cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
          // draw the circle center
          circle( result_image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
          // draw the circle outline
          circle( result_image, center, radius, cv::Scalar(255,0,0), 3, 8, 0 );
       }

       // Update GUI Window
       
       cv::imshow(BOTTOM_WINDOW, cv_ptr->image);
       cv::imshow(CIRCLE_WINDOW, result_image);
       cv::waitKey(3);

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