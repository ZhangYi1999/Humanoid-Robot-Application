// C++ include
#include <iostream>
#include <fstream>
#include <iomanip>
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
//#include "control/MoveJoints.h"
#include "control/Speak.h"
#include "control/Speech.h"

// Nao include
#include <naoqi_bridge_msgs/HeadTouch.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <naoqi_bridge_msgs/BlinkActionGoal.h>

#include <naoqi_bridge_msgs/WordRecognized.h>

// OpenCV include
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>



#include "sensor_msgs/JointState.h"
#include <string.h>

#include "ticket_checker.h"

class Nao_control {

private:
    // ros handler
    ros::NodeHandle nh_;

    // subscribe the detected faces
    ros::Subscriber detected_face_sub;

    // vector to save current faces in the picture, 4 int arrays. element 1 for yinglei, 2 for yi, 3 for tao, 4 for chongyu
    std::vector<int> current_faces;

public:
    Nao_control() : it_(nh_)
    {
        detected_face_sub = nh_.subscribe("/tactile_touch",1, &Nao_control::detect_face_Callback, this);
    }

    void top_camera_Callback(const std_msgs::Int8MultiArrayConstPtr& msg) {
        current_faces = msg->data;
    }

    bool check_face(string ticket_name) {
        if (strcmp(ticket_name, 'Mike')) {
            if (current_faces[0]) {
                return true;
            }
            else {
                return false;
            }
        }
        else if (strcmp(ticket_name, 'Jack')) {
            if (current_faces[1]) {
                return true;
            }
            else {
                return false;
            }
        }
        else if (strcmp(ticket_name, 'Tom')) {
            if (current_faces[2]) {
                return true;
            }
            else {
                return false;
            }
        }
        else if (strcmp(ticket_name, 'Amy')) {
            if (current_faces[4]) {
                return true;
            }
            else {
                return false;
            }
        }
    }

}