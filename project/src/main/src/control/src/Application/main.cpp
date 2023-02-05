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
#include <numeric>
#include <string.h>

// ROS include
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8MultiArray.h>
// #include "control/MoveJoints.h"
#include "control/Speak.h"
#include "control/Speech.h"
#include "sensor_msgs/JointState.h"

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

// self define function include
#include "ticket_checker.h"

enum class Task_State
{
    Not_Start_Yet,
    Stand_By,
    Walking,
    Check_Ticket,
    Wait_Questions,
    New_Station
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

    // publisher for nao speech
    ros::Publisher speech_pub;
    naoqi_bridge_msgs::SpeechWithFeedbackActionGoal speech_content;

    // client for speech recognition
    ros::ServiceClient speech_client;
    control::Speech speech_srv;

    // speak service
    control::Speak speak_srv;
    ros::ServiceClient speak_client;

    // subscriber to speech recognition
    ros::Subscriber recog_sub;

    // publisher for nao leds
    ros::Publisher eye_led_pub;

    // publisher for nao leds cancel
    ros::Publisher eye_led_cancel_pub;

    TicketChecker checker;
    bool welcome_once;
    std::string current_name;
    bool heard_name;

    // subscribe the detected faces
    ros::Subscriber detected_face_sub;

    // vector to save current faces in the picture, 4 int arrays. element 1 for yinglei, 2 for yi, 3 for tao, 4 for chongyu
    std::vector<int> current_faces;

    // // task 2
    bool check_ticket_flag;     // means robot is already in check ticket mode
    std::string name_on_ticket; // name on ticket, in order to varify if name on ticket is the same as name from face detection

    // // used for the 3. task
    bool check_attention;

    bool tactile_flag;

    // // used for the 4. task
    int pass_num;

public:
    Nao_control() : it_(nh_)
    {
        state = Task_State::Not_Start_Yet;

        top_camera_sub = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &Nao_control::top_camera_Callback, this);
        bot_camera_sub = it_.subscribe("/nao_robot/camera/bottom/camera/image_raw", 1, &Nao_control::bottom_camera_Callback, this);

        detected_face_sub = nh_.subscribe("/detected_human", 1, &Nao_control::detect_face_Callback, this);

        tactile_sub = nh_.subscribe("/tactile_touch", 1, &Nao_control::tactileCallback, this);

        speech_pub = nh_.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);

        speech_client = nh_.serviceClient<control::Speech>("speech_service");

        eye_led_pub = nh_.advertise<naoqi_bridge_msgs::BlinkActionGoal>("/blink/goal", 1);

        eye_led_cancel_pub = nh_.advertise<actionlib_msgs::GoalID>("/blink/cancel", 1);

        // speak client
        speak_client = nh_.serviceClient<control::Speak>("speak_service");

        checker.load_data(nh_);

        welcome_once = true;
        heard_name = false;

        topimg_updated = false;
        bottomimg_updated = false;

        tactile_flag = false;

        current_faces.push_back(0);
        current_faces.push_back(0);
        current_faces.push_back(0);
        current_faces.push_back(0);

        // Init name recognition

        voc_param.goal_id.id = "vocabulary";
        for (int i = 0; i < checker.ticket_num; i++)
        {
            voc_param.goal.words.push_back(checker.tickets[i].passenger_Name);
            std::cout << "Input Name " << checker.tickets[i].passenger_Name << " into vocabulary." << std::endl;
        }

        // check_ticket_flag = false;

        // // task 3
        check_attention = true;

        // // task 4
        pass_num = 0; // in the init state, we assum that there is no one in the container

        // // task 5
        // current_station = Train_station::Hamburg;
        std::cout << "End Init." << std::endl;
    }

    ~Nao_control()
    {
    }

    // Image callback function is executed when a new image is received:
    void top_camera_Callback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImageConstPtr cvImagePointer;
        try
        {
            cvImagePointer = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
            top_current_image = cvImagePointer->image.clone();
            topimg_updated = true;
        }
        catch (cv_bridge::Exception &except)
        {
            ROS_ERROR("cv_bridge exception: %s", except.what());
            return;
        }

        cv::imshow("top camera", top_current_image);
        cv::waitKey(3);
    }

    void bottom_camera_Callback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImageConstPtr cvImagePointer;
        try
        {
            cvImagePointer = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
            bottom_current_image = cvImagePointer->image.clone();
            bottomimg_updated = true;
        }
        catch (cv_bridge::Exception &except)
        {
            ROS_ERROR("cv_bridge exception: %s", except.what());
            return;
        }

        cv::imshow("bottom camera", bottom_current_image);
        cv::waitKey(3);
    }

    void tactileCallback(const naoqi_bridge_msgs::HeadTouch::ConstPtr &tactileState)
    {

        if (tactileState->button == tactileState->buttonMiddle)
        {
            if (tactileState->state == tactileState->statePressed)
            {
                tactile_flag = true;
            }
        }
    }

    void detect_face_Callback(const std_msgs::Int8MultiArrayConstPtr &msg)
    {
        current_faces[0] = msg->data[0];
        current_faces[1] = msg->data[1];
        current_faces[2] = msg->data[2];
        current_faces[3] = msg->data[3];
        if (std::accumulate(current_faces.begin(), current_faces.end(), 0) > 0)
        {
            check_attention = true;
        }
        else
        {
            check_attention = false;
        }
    }

    bool check_face(std::string ticket_name)
    {
        if (ticket_name.compare("Mike") == 0)
        {
            if (current_faces[0])
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else if (ticket_name.compare("Jack") == 0)
        {
            if (current_faces[1])
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else if (ticket_name.compare("Tom") == 0)
        {
            if (current_faces[2])
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else if (ticket_name.compare("Amy") == 0)
        {
            if (current_faces[3])
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    /////////////////////////////////////////////////////////// task 2 ////////////////////////////////////////
    void waitforticket()
    {
        say("start checking ticket", false, "");
        double start = ros::Time::now().toSec();
        ros::Rate rate2(10);
        while (ros::Time::now().toSec() - start < 10)
        {
            if (tactile_flag)
            {
                tactile_flag = false;
                checkTicket();
                start = ros::Time::now().toSec();
            }
            ros::spinOnce();
            rate2.sleep();
        }
        state = Task_State::Wait_Questions;
        say("train departure", false, "");
    }

    // check ticket valid
    void checkTicket()
    {
        if (pass_num >= 3)
        {
            say("The container is full, please go to another one.", true, "deny");
            return;
        }
        // Ask the passenger for ticket
        say("Hello! You need a ticket to get on or get off the train. Please show me your ticket.",
            false,
            "");

        // Wait the passenger to show ticket
        double begin = ros::Time::now().toSec();
        ros::Rate rate(10);
        bool over_time = false;
        start_blink(0, 0, 1, "finding qr code");
        while (!over_time)
        {
            // set waiting time limit
            if (ros::Time::now().toSec() - begin > 10)
            {
                over_time = true;
            }
            if (bottomimg_updated)
            {
                if (checker.checkQRcode(bottom_current_image))
                {
                    break;
                }
            }
            ros::spinOnce();
            rate.sleep();
        }
        stop_blink();

        if (over_time)
        {
            say("I didn't see your ticket. Please try it again.", true, "deny");
        }
        else
        {
            if (checker.checkValid())
            {

                if (checker.current_ticket.used == 1)
                {
                    say("See you. Wish you have a pleasant journey.", true, "up");
                    checker.tickets[checker.current_ticket.ticket_id].used = 2;
                    pass_num--;
                    return;
                }
                else if (checker.current_ticket.used == 2)
                {
                    say("Sorry, the ticket has already been used.", true, "deny");
                    return;
                }
                else
                {
                    say("What's your name?", false, "");
                    speech_srv.request.mode = 2;
                    if (speech_client.call(speech_srv))
                        ROS_INFO("Speech Recognition Service called");
                    else
                        ROS_ERROR("Speech Recognition Service Failed");

                    current_name = speech_srv.response.name;

                    if (current_name.compare("None") != 0)
                    {

                        if (checker.current_ticket.passenger_Name.compare(current_name) == 0)
                        {
                            if (check_face(current_name))
                            {
                                start_blink(0, 1, 0, "pass");
                                say("Hello, " + current_name + ". Welcome onboard. Wish you have a pleasant journey.", true, "agree");
                                checker.tickets[checker.current_ticket.ticket_id].used = 1;
                                pass_num++;
                            }
                            else
                            {
                                start_blink(1, 0, 0, "not ticket owner");
                                say("You are not " + current_name + ". You can not get in the train.", true, "deny");
                            }
                        }
                        else
                        {
                            start_blink(1, 0, 0, "not self ticket");
                            say("Sorry, " + current_name + ". The ticket is not yours. You can not get in the train.", true, "deny");
                        }
                    }
                    else
                    {
                        start_blink(1, 0, 0, "none say");
                        say("I didn't hear you.", true, "deny");
                    }
                    stop_blink();
                }
            }
            else
            {
                start_blink(1, 0, 0, "invalid qr code");
                say("The ticket is invalid. You can not get on the train.", true, "deny");
                stop_blink();
            }
        }
    }

    //////////////////////// task 3////////////////////////////////////////////////////////////
    void wait_questions()
    {
        bool detect_attention = true;
        double start_time = ros::Time::now().toSec();
        ros::Rate rate(10);
        while (ros::Time::now().toSec() - start_time < 5)
        {
            if (check_attention)
            {
                say("Get attention", false, "");
                detect_attention = true;
                break;
            }
            detect_attention = false;
            rate.sleep();
            ros::spinOnce();
        }

        // change mode
        if (!detect_attention)
        {
            state = Task_State::New_Station;
        }
        else
        {
            // wait for the questions from visitor by voice detection
            // provide explanation and motion

            say("If you have any questions. Please ask.", true, "stand");

            speech_srv.request.mode = 1;
            if (speech_client.call(speech_srv))
                ROS_INFO("Service called");
            else
                ROS_ERROR("Failed");
            answer_questions(speech_srv.response.question);
        }
    }

    void answer_questions(std::string question)
    {
        if (question.compare("When") == 0)
        {
            // when: when will arrive hamburg
            say("It is about three o'clock", false, "up");
        }
        else if (question.compare("Long") == 0)
        {
            // long: how long will it take from current station to hamburg
            // checker.train.current_station_id
        }
        else if (question.compare("Many") == 0)
        {
            // many: how many stations still have
            int station_num = 4 - checker.train.current_station_id;
            std::string temp_str;
            if (station_num == 0)
                say("Next station is Hamburg", true, "up");
            else
                temp_str = "still have " + std::to_string(station_num) + " stations until Hamburg";
            say(temp_str, true, "up");
        }
        else if (question.compare("no") == 0)
        {
            say("I did not hear your questions", true, "up");
        }
    }

    // ///////////////////////////////// task 4 and 5////////////////////////////////////////////////////
    // // in this mode, passengers will leave or go in.
    void new_station()
    {
        if (checker.train.moveToNextStation())
        {
            say("Final station hamburg is arrive.", true, "up");
        }
        else
        {
            say(checker.train.getCurrentStation().Name + " arrived", true, "Stand");
        }
        state = Task_State::Check_Ticket;
    }

    void pass_leave()
    {
        pass_num--;
    }

    void check_current_tickets()
    {
    }

    void wait(double duration)
    {
        double begin = ros::Time::now().toSec();
        while (ros::Time::now().toSec() - begin < duration)
        {
            ros::spinOnce();
        }
    }

    void say_welcome()
    {
        control::Speak speak_srv;

        // speak_srv.request.message = "Hello, welcome to the central station. Here is the German Train " \
        // + checker.train.name + ". I am Ticket checker Nao. please presse tacile to start";
        speak_srv.request.message = "Welcome";
        speak_srv.request.with_motion = false;
        speak_srv.request.motion_name = "";

        ros::Rate rate(10);
        while (!speak_client.call(speak_srv))
        {
            rate.sleep();
            ros::spinOnce();
        }
    }

    void say(std::string msg, bool with_motion, std::string motion)
    {
        control::Speak speak_srv;

        speak_srv.request.message = msg;
        speak_srv.request.with_motion = with_motion;
        speak_srv.request.motion_name = motion;

        if (speak_client.call(speak_srv))
        {
            ROS_INFO("say called");
            return;
        }
        else
            ROS_INFO("say failed");
    }

    void start_blink(double r, double g, double b, std::string id)
    {
        std_msgs::ColorRGBA color;
        std::vector<std_msgs::ColorRGBA> colors;
        naoqi_bridge_msgs::BlinkActionGoal blink_goal;

        color.r = r;
        color.g = g;
        color.b = b;
        color.a = 1;
        colors.push_back(color);

        blink_goal.goal_id.id = id;
        blink_goal.goal.colors = colors;
        blink_goal.goal.blink_duration = ros::Duration(1);
        blink_goal.goal.blink_rate_mean = 0.5;
        blink_goal.goal.blink_rate_sd = 0.05;
        eye_led_pub.publish(blink_goal);
    }

    void stop_blink()
    {
        actionlib_msgs::GoalID blink_cancel;

        blink_cancel.stamp = ros::Time::now();
        blink_cancel.id = "blink cancel";

        eye_led_cancel_pub.publish(blink_cancel);
    }

    //////////////////////////////////////////// mode switching ////////////////////////////////////////////
    void Mode_Switching_loop()
    {
        std::cout << "Start Loop." << std::endl;
        ros::Rate rate(10);

        say_welcome();

        state = Task_State::Check_Ticket;

        while (ros::ok())
        {
            switch (state)
            {
            case Task_State::Check_Ticket:
                waitforticket();

                break;
            // task 3
            case Task_State::Wait_Questions:
                // wait for the questions use voice recognition
                ROS_INFO("Wait for question.");
                wait_questions();

                break;
            // task 4&5, passenger count mode, in this mode we need to count the number of passenger
            // and change the train station
            case Task_State::New_Station:
                new_station();
                // pass_count();
                break;
            }

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
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