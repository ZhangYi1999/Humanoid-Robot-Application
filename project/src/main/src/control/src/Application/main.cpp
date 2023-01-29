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
    Pass_Count
};

enum class Train_station{
    Hamburg,
    Hannover,
    Munich
}

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
    TicketChecker checker;

    // task 2
    bool check_ticket_flag; // means robot is already in check ticket mode
    std::string name_on_ticket;// name on ticket, in order to varify if name on ticket is the same as name from face detection

    // used for the 3. task
    bool check_attention;

    // used for the 4. task
    int pass_num;

    // used for 5. task
    Train_station current_station;


public:
    Nao_control() : it_(nh_)
    {
        state = Task_State::Stand_By;
        image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &Nao_control::imageCallBack, this);
        checker = TicketChecker();
        img_updated=false;
        checked_ticket = 0;
        check_ticket_flag = false;

        // task 3
        check_attention = false;

        // task 4
        pass_num = 0; // in the init state, we assum that there is no one in the container

        // task 5
        current_station = Train_station::Hamburg;
    }

    ~Nao_control(){
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

            //////////////////////////////////////////////// task 2 ///////////////////////////////////////////////////////
            // detected QR code, change the state from standby to check ticket
            if((state == Task_State::Stand_By || state == Task_State::Check_Ticket) && (pass_num < 3) && checker.checkQRcode(current_image)){
                if (!check_ticket_flag) {
                    state = Task_State::Check_Ticket; // avoid nao always change mode
                    check_ticket_flag = true;
                }
                checkTicket(current_station);
            }

            // frame of face recognition will also come from camera, so the followed line should be wrote about face detection condition
            else if (state == Task_State::Check_Ticket && (pass_num < 3) && checker.check_face(current_image)){
                // find face , nao do something

                //checker.get_check_face_flag = true;
            }

            // container is full, but someone wants go in 
            else if (state == Task_State::Check_Ticket && pass_num >= 3 && 
                    (checker.check_face(current_image) || checker.checkQRcode(current_image))){
                        // nao do something to tell that one go away
                        container_full();
            }

            ///////////////////////////////////////////////// task 3  /////////////////////////////////////////////////////////
            else if (state == Task_State::Wait_Questions){
                check_attention = checker.check_attation(current_image);
            }

            ///////////////////////////////////////////////// task 4 //////////////////////////////////////////////////7
            // someone wants to leave, show nao QR code
            else if (state == Task_State::Pass_Count && checker.checkQRcode(current_image)){
                pass_leave();
            }
            // someone wants go in, so he shows his face, change mode
            else if (state == Task_State::Pass_Count && checker.check_face(current_image)){
                state = Task_State::Check_Ticket;
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
/////////////////////////////////////////////////////////// task 2 ////////////////////////////////////////
    // check ticket valid
    void checkTicket(Train_station current_station){
        // travel direction of passenger
        bool direction = true;
        if(checker.checkValid(current_station, direction)){
            // success check the ticket vaild, and get the name on ticket, nao do something 
            // and ask him to show his face to do face detection


            /* perhaps the following code will block callback function
            // success check the ticket vaild, wait 5 seconds for face recognition
            auto start_time = std::chrono::system_clock::now();
            std::chrono::seconds sec(5);

            bool detected_face_valid = false;
            // task 4, check if the ticket is already used
            if (checker.get_ticket_valid() == 0){
                // wait 5 sec to let passenger show the face
                while ((start_time - std::chrono::system_clock::now()) < sec){
                    if(checker.get_check_face_flag()){
                        // increase the checked ticket number
                        detected_face_valid = true;
                        // task 4, check the num of passagers
                        pass_num++;
                        checker.change_ticket();
                        break;
                    }
                    // ticket is vaild but ticket is not correspond to the face
                    else{
                        usleep(500000);
                        std::cout<<"not correct face"<<std::endl;
                        // and do or say something perhaps

                    }
                }
                if (detected_face_valid){
                    // ticket check success nao should do something, for example say something
                }
                else {
                    // neither dont detected face nor face is not correspond to ticket
                }
            */
            }
            else {
                // nao can do something?
                if (!direction){
                    std::cout<<"travel in wrong direction"<<std::endl;
                }
                else{ 
                    std::cout<<"Ticket has been used"<< std::endl;
                }
            }
        }
        else{
            std::cout<<"Ticket is not valid."<<std::endl; 
            // and do something perhaps

        }
        
    }

//////////////////////// task 3////////////////////////////////////////////////////////////
    void wait_questions(){
        if (check_attention){
            // wait for the questions from visitor by voice detection
            // provide explanation and motion
        }
        else {
            // wait for 5 seconds, if still cannot detect face, visitor may go away
            auto start_time = std::chrono::system_clock::now();
            std::chrono::seconds sec(5);
            bool detect_attention = false;
            while ((start_time - std::chrono::system_clock::now()) < sec)
                if (check_attention){
                    detect_attention = true;
                    break;
                }
        }
            // change mode
        if (!detect_attention)
            state = Task_State::Pass_Count;
    }   

///////////////////////////////// task 4 ////////////////////////////////////////////////////
// in this mode, passengers will leave or go in.
    void pass_count(){
        change_station();
        // current station has been changed, nao should do something means train station change
    }

// container full, nao do something to let that one go away
    void container_full(){

    }

// passenger wants to leave, should nao do something?
    void pass_leave(){
        pass_num --;
    }

/////////////////////////////////// task 5 /////////////////////////////////////////////////
// change train station
        // change train station
    void change_station(){
        int mid(static_cast<int>(current_station));
        mid++;
        current_station = static_cast<Train_station>(mid);
    }

//////////////////////////////////////////// mode switching ////////////////////////////////////////////
    void Mode_Switching_loop()
    {
        std::cout<<"Start Loop."<<std::endl;
        ros::Rate rate(100);
        while(ros::ok()){
            
            switch(state){
                // init mode
                case Task_State::Stand_By:
                    //state = Task_State::Check_Ticket;
                    break;
                // task 2
                case Task_State::Check_Ticket:
                    if (pass_num >= 3) state = Task_State::Wait_Questions
                    break;
                // task 3
                case Task_State::Wait_Questions:
                    // wait for the questions use voice recognition
                    wait_questions();
                    break;
                // task 4&5, passenger count mode, in this mode we need to count the number of passenger
                // and change the train station
                case Task_State::Pass_Count:
                    pass_count();
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