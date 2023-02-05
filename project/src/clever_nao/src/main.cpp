// C++ include
#include <iostream>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <vector>
#include <unistd.h>
#include <chrono>
#include <time.h>

// ROS include
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>

// custom service include
#include "clever_nao/MoveJoints.h"
#include "clever_nao/Speak.h"

// Nao include
#include <naoqi_bridge_msgs/HeadTouch.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <naoqi_bridge_msgs/BlinkActionGoal.h>

#include <naoqi_bridge_msgs/WordRecognized.h>

// OpenCV include
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>



#include "sensor_msgs/JointState.h"
#include <string.h>

#include "ticket_checker.h"

enum class Task_State{
    Not_Start_Yet,
    Stand_By,
    Walking,
    Check_Ticket,
    Wait_Questions,
    New_Station
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "control");
    // std::cout<<"start programm, start to check ticket"<<std::endl;
    // std::cout<<"please show the ticket"<<std::endl;
    // add a motion to nao, for example raise hand that shows nao is waiting for a ticket
    
    Nao_control robot;
    ros::spinOnce();
    robot.Mode_Switching_loop();

    return 0;

}