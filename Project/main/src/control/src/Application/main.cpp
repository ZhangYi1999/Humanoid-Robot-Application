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

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>


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
#include <string.h>

#include "ticket_checker.h"

enum class Task_State{
    Stand_By,
    Walking,
    Check_Ticket,
    Wait_Questions,
    Switching
};

enum class Train_station{
    Hamburg,
    Hannover,
    Nuremberg,
    Munich
}

class Nao_control
{
private:
    // ros handler
    ros::NodeHandle nh_;

    Task_State state;
    Train_station current_station;

    // Image transport and subscriber:
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    cv::Mat current_image;
    TicketChecker checker;

    int checked_ticket; // we only have 5 tickets, as along as we already checked 5 tickets, change mode
    bool check_ticket_flag; // means robot is already in check ticket mode
    std::string name_on_ticket;// name on ticket, in order to varify if name on ticket is the same as name from face detection

    // used for the 3. task
    bool check_attention;


public:
    Nao_control() : it_(nh_)
    {
        state = Task_State::Stand_By;
        image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 500, &Nao_control::imageCallBack, this);
        checker = TicketChecker();
        img_updated=false;
        current_station = Train_station::Hamburg;
        checked_ticket = 0;
        check_ticket_flag = false;

        // task 3
        check_attention = false;
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

            // detected QR code, change the state from standby to check ticket
            if(state == Task_State::Check_Ticket && (checked_ticket < 5) && checker.checkQRcode(current_image)){
                if (!check_ticket_flag) {
                    state = Task_State::Check_Ticket; // avoid nao always change mode
                    check_ticket_flag = true;
                }
                checkTicket();
            }

            // frame of face recognition will also come from camera, so the followed line should be wrote about face detection condition
            else if (state == Task_State::Check_Ticket && (checked_ticket < 5) && checker.check_face(current_image)){
                // find face 
                checker.get_check_face_flag = true;
            }

            ///////////////////////////////////////////////// task 3  /////////////////////////////////////////////////////////
            else if (state == Task_State::Wait_Questions){
                check_attention = checker.check_attation();
            }
        }
        catch (cv_bridge::Exception& except) 
        {
            ROS_ERROR("cv_bridge exception: %s", except.what());
                return;
        }
        cv::imshow("camera",current_image);
        cv::waitKey(3);
    }
/////////////////////////////////////////////////////////// task 2 ///////////////////////////////////////7
    // check ticket valid
    void checkTicket(){
        if(checker.checkValid()){
            // success check the ticket vaild, wait 5 seconds for face recognition
            auto start_time = std::chrono::system_clock::now();
            std::chrono::seconds sec(5);

            bool detected_face_valid = false;

            while ((start_time - std::chrono::system_clock::now()) < sec){
                if(checker.get_check_face_flag()){
                    // increase the checked ticket number
                    checked_ticket ++;
                    detected_face_valid = true;
                    break;
                }

                // ticket is vaild but ticket is not correspond to the face
                else{
                    usleep(500000);
                    std::cout<<"show you face, dont be shy"<<std::endl;
                    // and do or say something perhaps

                }
            }
            if (detected_face_valid){
                // ticket check success nao should do something, for example say something
            }
            else {
                // neither dont detected face nor face is not correspond to ticket
            }

        }
        else{
            std::cout<<"Ticket is not valid."<<std::endl; 
            // and do something perhaps

        }
        
    }

    // change train station
    void set_current_station(){
        int mid(static_cast<int>(current_station));
        mid++;
        current_station = static_cast<Train_station>(mid);
    }
            
//////////////////////// task 3////////////////////////////////////////////////////////////
    void wait_questions(){

    }


//////////////////////////////////////////// mode switching ////////////////////////////////////////////
    void Mode_Switching_loop()
    {
        std::cout<<"Start Loop."<<std::endl;
        ros::Rate rate(100);
        while(ros::ok()){
            
            switch(state){
                case Task_State::Stand_By:
                    //state = Task_State::Check_Ticket;
                    break;

                case Task_State::Walking:
                    break;
                // task 2
                case Task_State::Check_Ticket:
                    if (checked_ticket >= 5) state = Task_State::Wait_Questions
                    break;
                // task 3
                case Task_State::Wait_Questions:
                    // wait for the questions use voice recognition
                    //
                    wait_questions();
                    break;

                case Task_State::Switching:
                    
                    break;
            }

            ros::spinOnce();
            rate.sleep();
        }    
    }



};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control");
    std::cout<<"start programm, start to check ticket"<<std::endl;
    std::cout<<"please show the ticket"<<std::endl;
    // add a motion to nao, for example raise hand that shows nao is waiting for a ticket
    Nao_control robot;
    robot.Mode_Switching_loop();

    return 0;

}