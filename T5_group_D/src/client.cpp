#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "nao_control_tutorial_1/MoveJoints.h"
#include <string.h>
#include <vector>
#include <aruco/aruco.h>
#include <aruco/marker.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace std;

class Nao_control
{
public:
    // ros handler
    ros::NodeHandle nh_;

    // subscriber to joint states
    ros::Subscriber sensor_data_sub;
    ros::Subscriber image_sub_top;
    ros::ServiceClient move_joint_client;

    float Head_Yaw;
    float Head_Pitch;

    cv::Mat dist;
    cv::Mat cameraP;
    aruco::CameraParameters TheCameraParameters;

    Nao_control()
    {        
      sensor_data_sub=nh_.subscribe("/joint_states",1, &Nao_control::sensorCallback, this);
      move_joint_client = nh_.serviceClient <nao_control_tutorial_1::MoveJoints>("move_joints");
      image_sub_top = nh_.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &Nao_control::imageCb, this);
      cv::namedWindow("Robot Vision");
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
    }
    ~Nao_control()
    {
    }

  //handler for joint states
    void sensorCallback(const sensor_msgs::JointState::ConstPtr& jointState)
    {  
     /* ROS_INFO("--------------------------------------------------------");
      ROS_INFO("Function running!");    
      ROS_INFO_STREAM(jointState->name[0]<<jointState->position[0]); //HeadYaw
      ROS_INFO_STREAM(jointState->name[1]<<jointState->position[1]); //HeadPitch
      */
      Head_Yaw=jointState->position[0];
      Head_Pitch=jointState->position[1];
    }

// TODO: create function for each task
//////////// task1
void do1(string jointnames,double angle, float speed,int id)
    {
      nao_control_tutorial_1::MoveJoints srv;
      srv.request.names.push_back(jointnames);
      srv.request.angles.push_back(angle);
      srv.request.max_speed = speed;
      srv.request.mode = id;

      if (move_joint_client.call(srv)) ROS_INFO("Service called");
      else ROS_ERROR("Failed");
    }

////////////// task2
void do2(vector<string> jointnames,vector<double> angle,vector<double> time_second,float speed, int id)
    {
      nao_control_tutorial_1::MoveJoints srv;
      srv.request.names=jointnames; 
      srv.request.angles.push_back(angle[0]);  
      srv.request.angles.push_back(angle[1]);       
      
      srv.request.time_second.push_back(time_second[0]);     
      srv.request.time_second.push_back(time_second[1]); 
      
      srv.request.max_speed = speed;      
      srv.request.mode = id;      
      if (move_joint_client.call(srv))
          {
            ROS_INFO("Service called");
          }
      else
          {
            ROS_ERROR("Service Failed");
          }     
    }

/////////// task3
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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
    cv::Mat marker_img = srcimg.clone();
    aruco::MarkerDetector mark_detector;
    std::vector<aruco::Marker> detected_marker;
    mark_detector.detect(srcimg,detected_marker,TheCameraParameters,0.01,false);

    vector<string> jointnames;
    jointnames.push_back("HeadYaw");
    jointnames.push_back("HeadPitch");
    vector<double> time_second;
    time_second.push_back(0.5);
    

    for (unsigned i = 0; i< detected_marker.size(); i++ )
    {
      detected_marker[i].draw(marker_img,cv::Scalar(255,0,255),3);

      cv::Point2f marker_center = detected_marker[i].getCenter();
      ROS_INFO_STREAM(marker_center.x<<"  "<<marker_center.y);

      float camera_center_x=160.0;
      float camera_center_y=120.0;

      double target_angle_x;
      double target_angle_y;


      double err_x = camera_center_x - marker_center.x;
      if(camera_center_x - marker_center.x > 10)
      {
        target_angle_x = yawlimit(Head_Yaw + 0.04);
      }
      else if(camera_center_x - marker_center.x < 10)
      {
        target_angle_x = yawlimit(Head_Yaw - 0.04);
      }

      if(camera_center_y - marker_center.y < 30)
      {
        target_angle_y = pitchlimit(Head_Pitch + 0.04);
      }
      else if(camera_center_y - marker_center.y > 30)
      {
        target_angle_y = pitchlimit(Head_Pitch - 0.04);
      }

      nao_control_tutorial_1::MoveJoints srv;
      srv.request.names=jointnames; 
      srv.request.angles.push_back(target_angle_x);  
      srv.request.angles.push_back(target_angle_y);     
      srv.request.time_second.push_back(0.1); 
      srv.request.time_second.push_back(0.1);      
      srv.request.max_speed = 0.1;      
      srv.request.mode = 2;      
      if (move_joint_client.call(srv))
      {
        ROS_INFO("Service called");
      }
      else
      {
        ROS_ERROR("Service Failed");
      }
      cv::circle(marker_img,marker_center,2,cv::Scalar(0,0,255));
    }

    
    cv::imshow("Robot Vision", marker_img);
    cv::waitKey(3);
  }

  float yawlimit(float angle)
  {
    if(angle<-2.0857)
      angle = -2.0857;
    if(angle>2.0857)
      angle = 2.0857;
    return angle;
  }

  float pitchlimit(float angle)
  {
    if(angle<-0.6720)
      angle = -0.6720;
    if(angle>0.5149)
      angle = 0.5149;
    return angle;
  }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "nao_tutorial_control_1");
    ROS_INFO("Please enter the method: ");
    ROS_INFO("1: setAngles; 2: angleInterpolation");
    int id;
    // std::cin >> id;
    Nao_control ic;
///////// task 1   
    ic.do1("LShoulderPitch",0.7,0.1,1);

    
////////// task 2
    std::vector<string> jointnames;
    jointnames.push_back("LShoulderRoll");
    jointnames.push_back("LShoulderPitch");
    std::vector<double> angle;
    angle.push_back(0.53);
    angle.push_back(0);
    std::vector<double> time_second;
    time_second.push_back(2);
    time_second.push_back(1.0);
    ic.do2(jointnames,angle,time_second,0.1,id);

    ros::spin();
    return 0;
}
