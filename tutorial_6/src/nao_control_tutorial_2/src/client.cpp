#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <vector>

#include <ros/ros.h>

#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

#include "sensor_msgs/JointState.h"
#include "nao_control_tutorial_2/MoveJoints.h"
#include <string.h>


class Nao_control
{
    nao_control_tutorial_2::MoveJoints srv;
public:
    // ros handler
    ros::NodeHandle nh_;

    // Image transport and subscriber:
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    cv::Mat image;

    // Aruco camera parameters
    aruco::CameraParameters cameraParameters;

    // Aruco marker parameters
    float aruco_x, aruco_y, aruco_z;
    float markerSize;
    
    // tf boardcast
    tf::TransformBroadcaster my_tb;

    Nao_control() : it_(nh_)
    {
        image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 500, &Nao_control::imageCallBack, this);
    }

    ~Nao_control()
    {
    }

    // Image callback function is executed when a new image is received:
    void imageCallBack(const sensor_msgs::ImageConstPtr& msg) 
    {
        ROS_INFO_STREAM("Image callback execution");
        cv_bridge::CvImageConstPtr cvImagePointer;
	try 
	{
            cvImagePointer = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8);
	    image = cvImagePointer->image.clone();
	}
	catch (cv_bridge::Exception& except) 
	{
	    ROS_ERROR("cv_bridge exception: %s", except.what());
            return;
        }

	// Distortion matrix:
        cv::Mat dist = (cv::Mat_<double>(4, 1) <<
                -0.0870160932911717,
                0.128210165050533,
                0.003379500659424,
                -0.00106205540818586);

	// Camera matrix:
        cv::Mat cameraP = (cv::Mat_<double>(3, 3) <<
                274.139508945831, 0.0, 141.184472810944,
                0.0, 275.741846757374, 106.693773654172,
                0.0, 0.0, 1.0);

	cameraParameters.setParams(cameraP,dist,cv::Size(640,480));
        aruco::MarkerDetector arucoDetector;
        std::vector<aruco::Marker> arucoMarkers;
        arucoDetector.detect(image, arucoMarkers);

        if (arucoMarkers.size() > 0) 
	{
	    ROS_WARN_STREAM("I could detect "<<arucoMarkers.size()<<" aruco marker(s).");
	    markerSize = 0.064;
	    arucoMarkers[0].calculateExtrinsics(markerSize, cameraParameters, true);
            arucoMarkers[0].draw(image, cv::Scalar(0,0,255), 2);
            aruco_x = arucoMarkers[0].Tvec.at<float>(0);
            aruco_y = arucoMarkers[0].Tvec.at<float>(1);
            aruco_z = arucoMarkers[0].Tvec.at<float>(2);
        }
	else
	{
	    ROS_WARN_STREAM("No aruco marker detected");
            aruco_x = 0.0;
            aruco_y = 0.0;
            aruco_z = 0.0;
	}
	ROS_INFO_STREAM("aruco_x ="<<aruco_x);
	ROS_INFO_STREAM("aruco_y ="<<aruco_y);
	ROS_INFO_STREAM("aruco_z ="<<aruco_z);

    // tf boardcaster, boardcast the transform into rviz
    tf::Transform my_transform;
    my_transform.setOrigin(tf::Vector3(aruco_x,aruco_y,aruco_z));
    my_tb.sendTransform(tf::StampedTransform(my_transform, ros::Time::now(), "world", "my_transform"));

	// Display marker
        cv::imshow("marker", image);
        cv::waitKey(3);
    }

// TODO: create function for each task
// ask service to get current joint position
std::vector<float> task1(){
    srv.request.task = 1;
    if (move_joint_client.call(srv)){
        ROS_INFO("Service called");
        std::vector<float> current_pos;
        for(int i=0; i<6, ++i) current_pos.append(srv.response.current[i]);
        return current_pos;
    }
    else ROS_ERROR("Failed");
}

void task2(){
    // set the chain
    ROS_INFO("Please choose desired end effector with number: LArm(1), RArm(2)");
    int arm;
    std::cin >> arm;
    if (arm == 1) srv.request.name = "LArm";
    else if (arm == 2) srv.request.name = "RArm";

    // get current position and save it in the srv
    std::vector<float> current_pos = task1();
    ROS_INFO("current pose is: %f,%f,%f,%f,%f,%f",current_pos[0],current_pos[1],current_pos[2],current_post[3],current_pos[4],current_pos[5]);
    
    // get the target pose
    ROS_INFO("Please enter the target pose: ");
    float[6] target_pos;
    std::cin >> target_pos;
    srv.request.target = target_pos;
    
    // base on target pose, choose used axis number
    if (target_pos[3] == 0 && target_pos[4] == 0 && target_pos[5] == 0) srv.request.mask = 7;
    else if srv.request.mask = 63;
    
    // user specified methode
    ROS_INFO("Please enter number to choose desied fraction of max velocity or execution time: fraction of max velocity(1), execution time(2)");
    int num, para;
    std::cin >> number;
    if (number == 1){
        ROS_INFO("Please enter the value of desired fraction of max velocity:");
        std::cin >> para;
        srv.request.max_speed = para;
    }
    else if (number == 2){
        ROS_INFO("Please enter the value of desired execution time:");
        std::cin >> para;
        srv.request.time = para;
    }

    // service call
    if (move_joint_client.call(srv)) ROS_INFO("Service called");
    else ROS_ERROR("Failed");
}

};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "nao_tutorial_control_2");
    Nao_control my_NAO_ControlInstance;
    ROS_INFO("Please enter the task: ");
    int task_num;
    cin >> task_num;
    if (task_num == 1) my_NAO_ControlInstance.task1();
    else if (task_num == 2) my_NAO_ControlInstance.task2();
    ros::spin();
    return 0;

}
