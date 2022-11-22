#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "nao_control_tutorial_1/MoveJoints.h"
#include <string.h>
#include <qi/os.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video.hpp>
#include <aruco/aruco.h>
#include <image_transport/image_transport.h>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>
using namespace std;
using namespace cv;
using namespace aruco;
using namespace Eigen;

int method_id;
Mat image;
float H_Yaw;
float H_Pitch;

class Nao_control
{
public:
    // ros handler
    ros::NodeHandle nh_;

    // subscriber to joint states
    ros::Subscriber sensor_data_sub;
    image_transport::ImageTransport it_;

    ros::Subscriber sensor_data_sub;
    image_transport::Subscriber image_sub;

    
    Nao_control()
    {
        sensor_data_sub=nh_.subscribe("/joint_states",1, &Nao_control::sensorCallback, this);
        image_sub = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1,
                                &Nao_control::imageCb, this);
    }
    ~Nao_control()
    {
    }

  //handler for joint states
    void sensorCallback(const sensor_msgs::JointState::ConstPtr& jointState)
    {  
      ROS_INFO("--------------------------------------------------------");
      ROS_INFO("Function running!");    
      ROS_INFO_STREAM(jointState->name[0]<<jointState->position[0]); //HeadYaw
      ROS_INFO_STREAM(jointState->name[1]<<jointState->position[1]); //HeadPitch
      H_Yaw=jointState->position[0];
      H_Pitch=jointState->position[1];
    }

// TODO: create function for each task
void do1(string jointnames,double angle, float speed,int id)
    {
      ros::ServiceClient move_joint_client = nh_.serviceClient <nao_control_tutorial_1::MoveJoints>("Movejoints");
      nao_control_tutorial_1::MoveJoints srv;
      
      srv.request.names.push_back(jointnames);
      
      srv.request.angles.push_back(angle);
      
      srv.request.fractionMaxSpeed = speed;

      srv.request.id = id;

      if (move_joint_client.call(srv))
          {
            ROS_INFO("Service called");
          }
        else
          {
            ROS_ERROR("Failed");
          }

     
    }


    void do2(vector<string> jointnames,vector<double> angle,vector<double> time_second,float speed)
    {
      ros::ServiceClient move_joint_client = nh_.serviceClient <nao_control_tutorial_1::MoveJoints>("Movejoints");
      nao_control_tutorial_1::MoveJoints srv;
     
      srv.request.names=jointnames; 
      srv.request.angles=angle;      
      srv.request.time_second=time_second;      
      srv.request.fractionMaxSpeed = speed;      
      srv.request.id = method_id;      
      if (move_joint_client.call(srv))
          {
            ROS_INFO("Service called");
          }
      else
          {
            ROS_ERROR("Service Failed");
          }     
    }


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
        image=cv_ptr->image;
        // create a variable in the class
        CameraParameters TheCameraParameters;
        //camera parameter
        Mat dist(1,5,CV_32FC1);
        dist.at<float>(0,0)=-0.066494;
        dist.at<float>(0,1)=0.095481;
        dist.at<float>(0,2)=-0.000279;
        dist.at<float>(0,3)=0.002292;
        dist.at<float>(0,4)=0.000000;
        Mat cameraP(3,3,CV_32FC1);

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
        aruco::MarkerDetector Detector;

        vector< Marker >  detectedMarkers;
        Detector.detect(cv_ptr->image,detectedMarkers,TheCameraParameters,5,true);
        vector<string> jointnames;
        jointnames.push_back("HeadPitch");
        jointnames.push_back("HeadYaw");
        vector<double> time_second;
        time_second.push_back(0.5);
        time_second.push_back(0.5);
        float marker_center_x;
        float marker_center_y;
        for (unsigned i = 0; i< detectedMarkers.size(); i++ )
        {
          detectedMarkers[i].draw(cv_ptr->image,Scalar(0,0,255),2);

          vector<float> center_refer;
          center_refer.push_back(0);
          center_refer.push_back(0);
          center_refer.push_back(0);
          std::vector<cv::Point2f> projectedPoints_center;
          cv::projectPoints(center_refer,detectedMarkers[i].Rvec,detectedMarkers[i].Tvec,cameraP,dist,projectedPoints_center);

          cout << "x"<<projectedPoints_center[0].x << endl;
          float image_center_x=160.0;
          float image_center_y=120.0;
          float dangle_x ;
          float dangle_y ;
          float proporiton_x=0.002;
          float proporiton_y=0.0015;
          float error_x,error_y;
          float H_Yaw_target,H_Pitch_target;
          
          // P control for the tracking
          error_x=projectedPoints_center[0].x-image_center_x;
          error_y=projectedPoints_center[0].y-image_center_y;
          dangle_x=proporiton_x*error_x;
          dangle_y=proporiton_y*error_y;

          H_Yaw_target=H_Yaw-dangle_x;
          H_Pitch_target=H_Pitch+dangle_y;

          vector<double> angle;
          angle.push_back(H_Pitch_target);
          angle.push_back(H_Yaw_target);

          if ((abs(H_Yaw_target)<2.0857) && (H_Pitch_target>-0.6720) && (H_Pitch_target<0.5149) ) // safe threshold
          {
            Nao_control::do2(jointnames,angle,time_second,0.1);
          }


        }

        namedWindow("Marker tracking");
        imshow("Marker tracking", cv_ptr->image);
        cv::waitKey(3);
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "nao_tutorial_control_1");
    ROS_INFO("Please enter the method: ");
    ROS_INFO("1: setAngles; 2: angleInterpolation");
    cin>>method_id;
    Nao_control ic;
    
    //ic.do1("LShoulderPitch",-0.7,0.1,1);
    
    vector<string> jointnames;
    jointnames.push_back("LShoulderRoll");
    jointnames.push_back("LShoulderPitch");
    vector<double> angle;
    angle.push_back(0.53);
    angle.push_back(0);
    vector<double> time_second;
    time_second.push_back(1.5);
    time_second.push_back(1.0);

    //ic.do2(jointnames,angle,time_second,0.1);

    ros::spin();
    return 0;
}
