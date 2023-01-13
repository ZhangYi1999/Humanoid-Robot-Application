#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <vector>
#include <unistd.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>


#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

#include "sensor_msgs/JointState.h"
#include "nao_control_tutorial_2/MoveJoints.h"
#include <string.h>

enum class Task_State{
    Stand_By,
    Walking,
    Check_Ticket,
    Check_Face,
    Switching
};


class Nao_control
{
private:
    // ros handler
    ros::NodeHandle nh_;

    Task_State state;
    // Image transport and subscriber:
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    cv::Mat current_image;

public:
    Nao_control() : it_(nh_)
    {
        this.state = Task_State::Stand_By;
        this.image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 500, &Nao_control::imageCallBack, this);
        this.
    }

    ~Nao_control()
    {
    }

    // Image callback function is executed when a new image is received:
    void imageCallBack(const sensor_msgs::ImageConstPtr& msg) 
    {
        // ROS_INFO_STREAM("Image callback execution");
        cv_bridge::CvImageConstPtr cvImagePointer;
        try 
        {
            cvImagePointer = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8);
            current_image = cvImagePointer->image.clone();
        }
        catch (cv_bridge::Exception& except) 
        {
            ROS_ERROR("cv_bridge exception: %s", except.what());
                return;
        }
    }

    void checkTicket(){
        cv::QRCodeDetector detector = cv::QRCodeDetector::QRCodeDetector();
        string message = detector.detectAndDecode(this.current_image);
        if(message!=null){
            
        }
        else{

        }
    }

    void Mode_Switching_loop()
    {
        while(true){
            switch(this.state){
                case Task_State::Stand_By:
                    break;
                case Task_State::Walking:
                    break;
                case Task_State::Check_Ticket:
                    
                    break;
                case Task_State::Check_Face:
                    
                    break;
                case Task_State::Switching:
                    
                    break;
            }

            ros::spinOnce();
        }    
    }



};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control");

    Nao_control robot;
    robot.Mode_Switching_loop();

    return 0;

}