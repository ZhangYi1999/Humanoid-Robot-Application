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

enum class Task_State{
    Not_Start_Yet,
    Stand_By,
    Walking,
    Check_Ticket,
    Wait_Questions,
    Pass_Count
};


class Nao_control
{
private:
    // ros handler
    ros::NodeHandle nh_;

    Task_State state;

    // Image transport and subscriber:
    image_transport::ImageTransport it_;
    image_transport::Subscriber top_camera_sub;
    image_transport::Subscriber bot_camera_sub;

    cv::Mat top_current_image;
    bool topimg_updated;
    cv::Mat bottom_current_image;
    bool bottomimg_updated;



    // subscriber to head tactile states
	ros::Subscriber tactile_sub;

	//publisher for nao speech
	ros::Publisher speech_pub;
    naoqi_bridge_msgs::SpeechWithFeedbackActionGoal speech_content;

    //publisher for nao vocabulary parameters
	ros::Publisher voc_params_pub;
    naoqi_bridge_msgs::SetSpeechVocabularyActionGoal voc_param;

    //client for starting speech recognition
    ros::ServiceClient recog_start_srv;

    //client for stoping speech recognition
    ros::ServiceClient recog_stop_srv;

    // subscriber to speech recognition
    ros::Subscriber recog_sub;

    //publisher for nao leds
	ros::Publisher eye_led_pub;

	//publisher for nao leds cancel
	ros::Publisher eye_led_cancel_pub;
    
    TicketChecker checker;
    bool welcome_once;
    std::string current_name;
    bool heard_name;

    // // task 2
    // bool check_ticket_flag; // means robot is already in check ticket mode
    // std::string name_on_ticket;// name on ticket, in order to varify if name on ticket is the same as name from face detection

    // // used for the 3. task
    // bool check_attention;

    // // used for the 4. task
    // int pass_num;

    // // used for 5. task
    // Train_station current_station;


public:
    Nao_control() : it_(nh_)
    {
        state = Task_State::Not_Start_Yet;

        top_camera_sub= it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &Nao_control::top_camera_Callback, this);
        bot_camera_sub= it_.subscribe("/nao_robot/camera/bottom/camera/image_raw", 1, &Nao_control::bottom_camera_Callback, this);
        
        tactile_sub = nh_.subscribe("/tactile_touch",1, &Nao_control::tactileCallback, this);

		speech_pub = nh_.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);

        voc_params_pub= nh_.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>("/speech_vocabulary_action/goal", 1);

        recog_start_srv=nh_.serviceClient<std_srvs::Empty>("/start_recognition");

		recog_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_recognition");

		recog_sub=nh_.subscribe("/word_recognized",1, &Nao_control::speechRecognitionCB, this);

        eye_led_pub = nh_.advertise<naoqi_bridge_msgs::BlinkActionGoal>("/blink/goal", 1);

        eye_led_cancel_pub =  nh_.advertise<actionlib_msgs::GoalID>("/blink/cancel", 1);


        checker.load_data(nh_);
        welcome_once = true;
        heard_name = false;

        topimg_updated = false;
        bottomimg_updated = false;

        // Init name recognition
        
        voc_param.goal_id.id="vocabulary";
        for(int i = 0; i < checker.ticket_num; i++){
            voc_param.goal.words.push_back(checker.tickets[i].passenger_Name);
            std::cout<<"Input Name "<< checker.tickets[i].passenger_Name << " into vocabulary." <<std::endl;
        }
        
        

        // check_ticket_flag = false;

        // // task 3
        // check_attention = false;

        // // task 4
        // pass_num = 0; // in the init state, we assum that there is no one in the container

        // // task 5
        // // current_station = Train_station::Hamburg;
        std::cout<<"End Init."<<std::endl;
    }

    ~Nao_control(){
    }

    // Image callback function is executed when a new image is received:
    void top_camera_Callback(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImageConstPtr cvImagePointer;
        try 
        {
            cvImagePointer = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8);
            top_current_image = cvImagePointer->image.clone();
            topimg_updated = true;
        }
        catch (cv_bridge::Exception& except) 
        {
            ROS_ERROR("cv_bridge exception: %s", except.what());
                return;
        }
        
        cv::imshow("top camera",top_current_image);
        cv::waitKey(3);
    }

    void bottom_camera_Callback(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImageConstPtr cvImagePointer;
        try 
        {
            cvImagePointer = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8);
            bottom_current_image = cvImagePointer->image.clone();
            bottomimg_updated = true;
        }
        catch (cv_bridge::Exception& except) 
        {
            ROS_ERROR("cv_bridge exception: %s", except.what());
                return;
        }
        
        cv::imshow("bottom camera",bottom_current_image);
        cv::waitKey(3);
    }


// //////////////////////////////////////////////// task 2 ///////////////////////////////////////////////////////
            // // detected QR code, change the state from standby to check ticket
            // if((state == Task_State::Stand_By || state == Task_State::Check_Ticket) && (pass_num < 3) && checker.checkQRcode(current_image)){
            //     if (!check_ticket_flag) {
            //         state = Task_State::Check_Ticket; // avoid nao always change mode
            //         check_ticket_flag = true;
            //     }
            //     checkTicket(current_station);
            // }

            // // frame of face recognition will also come from camera, so the followed line should be wrote about face detection condition
            // else if (state == Task_State::Check_Ticket && (pass_num < 3) && checker.check_face(current_image)){
            //     // find face , nao do something

            //     //checker.get_check_face_flag = true;
            // }

            // // container is full, but someone wants go in 
            // else if (state == Task_State::Check_Ticket && pass_num >= 3 && 
            //         (checker.check_face(current_image) || checker.checkQRcode(current_image))){
            //             // nao do something to tell that one go away
            //             container_full();
            // }

            // ///////////////////////////////////////////////// task 3  /////////////////////////////////////////////////////////
            // else if (state == Task_State::Wait_Questions){
            //     check_attention = checker.check_attation(current_image);
            // }

            // ///////////////////////////////////////////////// task 4 //////////////////////////////////////////////////7
            // // someone wants to leave, show nao QR code
            // else if (state == Task_State::Pass_Count && checker.checkQRcode(current_image)){
            //     pass_leave();
            // }
            // // someone wants go in, so he shows his face, change mode
            // else if (state == Task_State::Pass_Count && checker.check_face(current_image)){
            //     state = Task_State::Check_Ticket;
            // }
    void tactileCallback(const naoqi_bridge_msgs::HeadTouch::ConstPtr& tactileState){
        

        if (tactileState->button==tactileState->buttonMiddle)
        {
            if (tactileState->state == tactileState->statePressed){
                state = Task_State::Check_Ticket;
            }
        }

    }

    void speechRecognitionCB(const naoqi_bridge_msgs::WordRecognized::ConstPtr& msg)
	{
        if(msg->words.size()>0){
            current_name = msg->words[0];
            heard_name = true;
        }
	}

/////////////////////////////////////////////////////////// task 2 ////////////////////////////////////////
    // check ticket valid
    void checkTicket(){
        // Ask the passenger for ticket
        say("Hello! You need a ticket to get on the train. Please show me your ticket.");
        wait(4);
        // Wait the passenger to show ticket
        double begin = ros::Time::now().toSec();
        ros::Rate rate(10);
        bool over_time = false;
        start_blink(0,0,1,"finding qr code");
        while(!over_time){
            // set waiting time limit
            if(ros::Time::now().toSec() - begin > 30){ 
                over_time = true;
                stop_blink();
            }
            if(bottomimg_updated) {
                if(checker.checkQRcode(bottom_current_image)) {
                    break;
                }
            }
            ros::spinOnce();
            rate.sleep();
        }
        stop_blink();
        wait(5);
        if(over_time){
            say("I didn't see your ticket. Please try it again.");
        }
        else{
            if(checker.checkValid()){
                say("What's your name?");

                wait(1);

                voc_params_pub.publish(voc_param);
                std_srvs::Empty emp1;
                recog_start_srv.call(emp1);

                double t = ros::Time::now().toSec();
                while(ros::Time::now().toSec() - t < 10 && !heard_name)
                {
                    ros::spinOnce();
                }
                std_srvs::Empty emp2;
                recog_stop_srv.call(emp2);

                wait(2);
                if(heard_name){
                    heard_name = false;
                    if(checker.current_ticket.passenger_Name.compare(current_name)==0){
                        start_blink(0,1,0,"pass");
                        say("Hello, "+current_name+". Welcome onboard.");
                    }
                    else{
                        start_blink(1,0,0,"not self ticket");
                        say("Sorry, "+current_name+". The ticket is not yours.");
                    }
                }
                else{
                    start_blink(1,0,0,"none say");
                    say("I didn't hear you.");
                }
                wait(8);
                stop_blink();
            }
            else{
                wait(2);
                start_blink(1,0,0,"invalid qr code");
                
                say("The ticket is invalid. You can not get on the train.");
                wait(8);
                stop_blink();
            }

            
        }
    
    }

//////////////////////// task 3////////////////////////////////////////////////////////////
//     void wait_questions(){
//         if (check_attention){
//             // wait for the questions from visitor by voice detection
//             // provide explanation and motion
//         }
//         else {
//             // wait for 5 seconds, if still cannot detect face, visitor may go away
//             auto start_time = std::chrono::system_clock::now();
//             std::chrono::seconds sec(5);
//             bool detect_attention = false;
//             while ((start_time - std::chrono::system_clock::now()) < sec)
//                 if (check_attention){
//                     detect_attention = true;
//                     break;
//                 }
//         }
//             // change mode
//         if (!detect_attention)
//             state = Task_State::Pass_Count;
//     }   

// ///////////////////////////////// task 4 ////////////////////////////////////////////////////
// // in this mode, passengers will leave or go in.
//     void pass_count(){
//         change_station();
//         // current station has been changed, nao should do something means train station change
//     }

// // container full, nao do something to let that one go away
//     void container_full(){

//     }

// // passenger wants to leave, should nao do something?
//     void pass_leave(){
//         pass_num --;
//     }

// /////////////////////////////////// task 5 /////////////////////////////////////////////////
// // change train station
//         // change train station
//     void change_station(){
//         int mid(static_cast<int>(current_station));
//         mid++;
//         current_station = static_cast<Train_station>(mid);
//     }

    void wait(double duration){
        double begin = ros::Time::now().toSec();
        while(ros::Time::now().toSec() - begin < duration){
             ros::spinOnce();
        }
    }

    void say_welcome(){
        speech_content.goal_id.id = "Welcome";
        speech_content.goal.say = "Hello, welcome to the central station. Here is the German Train " + checker.train.name + ". I am Ticket checker Nao.";
        speech_pub.publish(speech_content);
        speech_content.goal.say="";
        wait(2);
    }

    void say(std::string msg){
        speech_content.goal_id.id = "Just Saying";
        speech_content.goal.say = msg;
        speech_pub.publish(speech_content);
        speech_content.goal.say="";
    }


    void start_blink(double r, double g, double b, std::string id){
        std_msgs::ColorRGBA color;
		std::vector<std_msgs::ColorRGBA> colors;
        naoqi_bridge_msgs::BlinkActionGoal blink_goal;

        color.r=r;
        color.g=g;
        color.b=b;
        color.a=1;
        colors.push_back(color);

        blink_goal.goal_id.id = id;
        blink_goal.goal.colors = colors;
        blink_goal.goal.blink_duration = ros::Duration(1);
        blink_goal.goal.blink_rate_mean = 0.5;
        blink_goal.goal.blink_rate_sd = 0.05;
        eye_led_pub.publish(blink_goal);

    }

    void stop_blink(){
        actionlib_msgs::GoalID blink_cancel;

        blink_cancel.stamp = ros::Time::now();
        blink_cancel.id = "blink cancel";

		eye_led_cancel_pub.publish(blink_cancel);

    }

//////////////////////////////////////////// mode switching ////////////////////////////////////////////
    void Mode_Switching_loop()
    {
        std::cout<<"Start Loop."<<std::endl;
        ros::Rate rate(10);
       
        while(ros::ok()){
            switch(state){
                case Task_State::Not_Start_Yet:
                    say_welcome();
                    // state = Task_State::Stand_By;
                    // state = Task_State::Check_Ticket;
                    break;
                case Task_State::Check_Ticket:
                    checkTicket();
                    state = Task_State::Stand_By;
                    break;
                // task 3
                case Task_State::Wait_Questions:
                    // wait for the questions use voice recognition
                    // wait_questions();
                    break;
                // task 4&5, passenger count mode, in this mode we need to count the number of passenger
                // and change the train station
                case Task_State::Pass_Count:
                    // pass_count();
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
    // std::cout<<"start programm, start to check ticket"<<std::endl;
    // std::cout<<"please show the ticket"<<std::endl;
    // add a motion to nao, for example raise hand that shows nao is waiting for a ticket
    
    Nao_control robot;
    ros::spinOnce();
    robot.Mode_Switching_loop();

    return 0;

}