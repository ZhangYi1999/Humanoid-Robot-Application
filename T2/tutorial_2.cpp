#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string ORIGINAL_WINDOW = "Original image";
static const std::string GREYSACLE_WINDOW = "Greyscale image";
static const std::string BINARY_WINDOW = "Binary image";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &ImageConverter::imageCb, this);

    cv::namedWindow(ORIGINAL_WINDOW);
    cv::namedWindow(GREYSACLE_WINDOW);
    cv::namedWindow(BINARY_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(ORIGINAL_WINDOW);
    cv::destroyWindow(GREYSACLE_WINDOW);
    cv::destroyWindow(BINARY_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat grey_cv;
    cv::Mat binary_cv;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Convert color image to greyscale and binary image
    cv::cvtColor(cv_ptr->image, grey_cv, cv::COLOR_BGR2GRAY);
    cv::threshold(grey_cv, binary_cv, 128, 255, cv::THRESH_BINARY);

    // Update GUI Window
    cv::imshow(ORIGINAL_WINDOW, cv_ptr->image);
    cv::imshow(GREYSACLE_WINDOW, grey_cv);
    cv::imshow(BINARY_WINDOW, binary_cv);
    cv::waitKey(3);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}