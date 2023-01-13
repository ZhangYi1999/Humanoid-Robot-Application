#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/JointState.h"
#include "message_filters/subscriber.h"
#include <string.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/HeadTouch.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedAction.h>
#include <std_srvs/Empty.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <naoqi_bridge_msgs/BlinkActionGoal.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <std_msgs/ColorRGBA.h>
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include "actionlib/client/simple_action_client.h"



using namespace std;

bool stop_thread=false;

void spinThread()
{
	while(!stop_thread)
	{
		ros::spinOnce();
		//ROS_INFO_STREAM("Spinning the thing!!");
	}
}


class Nao_control
{
public:
	// ros handler
	ros::NodeHandle nh_;

	// subscriber to bumpers states
	ros::Subscriber bumper_sub;

	// subscriber to head tactile states
	ros::Subscriber tactile_sub;

	//publisher for nao speech
	ros::Publisher speech_pub;

	//publisher for nao leds
	ros::Publisher leds_pub;

	//publisher for nao leds cancel
	ros::Publisher leds_cancel_pub;

	//publisher for nao vocabulary parameters
	ros::Publisher voc_params_pub;

	//client for starting speech recognition
	ros::ServiceClient recog_start_srv;

	//client for stoping speech recognition
	ros::ServiceClient recog_stop_srv;

	// subscriber to speech recognition
	ros::Subscriber recog_sub;

	// publisher to nao walking
	ros::Publisher walk_pub;

	//client for stop walking
	ros::ServiceClient stop_walk_srv;

	//subscriber for foot contact
	ros::Subscriber footContact_sub;

	boost::thread *spin_thread;

	bool foot_contact; // declare for walk
	naoqi_bridge_msgs::SpeechWithFeedbackActionGoal speech; // declare for speech
	naoqi_bridge_msgs::BlinkActionGoal blink_goal;
	actionlib_msgs::GoalID blink_cancel;
	bool walking;

	Nao_control()
	{

		// subscribe to topic bumper and specify that all data will be processed by function bumperCallback
		bumper_sub=nh_.subscribe("/bumper",1, &Nao_control::bumperCallback, this);

		// subscribe to topic tactile_touch and specify that all data will be processed by function tactileCallback
		tactile_sub=nh_.subscribe("/tactile_touch",1, &Nao_control::tactileCallback, this);

		speech_pub = nh_.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);

		leds_pub= nh_.advertise<naoqi_bridge_msgs::BlinkActionGoal>("/blink/goal", 1);

		leds_cancel_pub= nh_.advertise<actionlib_msgs::GoalID>("/blink/cancel", 1);

		voc_params_pub= nh_.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>("/speech_vocabulary_action/goal", 1);

		recog_start_srv=nh_.serviceClient<std_srvs::Empty>("/start_recognition");

		recog_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_recognition");

		recog_sub=nh_.subscribe("/word_recognized",1, &Nao_control::speechRecognitionCB, this);

		footContact_sub = nh_.subscribe<std_msgs::Bool>("/foot_contact", 1, &Nao_control::footContactCB, this);

		walk_pub=nh_.advertise<geometry_msgs::Pose2D>("/cmd_pose", 1);

		stop_walk_srv = nh_.serviceClient<std_srvs::Empty>("/stop_walk_srv");

		stop_thread=false;
		spin_thread=new boost::thread(&spinThread);
		walking = false;
	}
	~Nao_control()
	{
		stop_thread=true;
		sleep(1);
		spin_thread->join();
	}

	void footContactCB(const std_msgs::BoolConstPtr& contact)
	{
		/*
		 foot contact information
		 */
		foot_contact=contact->data;
	}

	void speechRecognitionCB(const naoqi_bridge_msgs::WordRecognized::ConstPtr& msg)
	{
		/*
		 * speak, get and store them 
		 */
		speech.goal_id.id="recognition";

		for (int i=0; i<msg->words.size();i++)
		{
			speech.goal.say+=msg->words[i];
			speech.goal.say+=" ";
		}

	}

	void bumperCallback(const naoqi_bridge_msgs::Bumper::ConstPtr& bumperState)
	{
		/*
		 * for 1, blink : but i can not make the blink action. 
		 */

		std_msgs::ColorRGBA color;
		std::vector<std_msgs::ColorRGBA> colors;

		if (bumperState->bumper==bumperState->left)
		{
			if (bumperState->state==bumperState->statePressed){
				color.r=1;
				color.g=0;
				color.b=0;
				color.a=1;
				colors.push_back(color);

				blink_goal.goal_id.id = "LEFT Bumper touched";
				blink_goal.goal.colors = colors;
				blink_goal.goal.blink_duration = ros::Duration(2.0);
				blink_goal.goal.blink_rate_mean = 2;
				blink_goal.goal.blink_rate_sd = 0.05;
				leds_pub.publish(blink_goal);
			}
			else{
				blink_cancel.stamp = ros::Time::now();
				blink_cancel.id = "blink cancel";

				leds_cancel_pub.publish(blink_cancel);
			}
		}
		else if (bumperState->bumper==bumperState->right)
		{
			if (bumperState->state==bumperState->statePressed){
				color.r=0;
				color.g=1;
				color.b=0;
				color.a=1;
				colors.push_back(color);

				blink_goal.goal_id.id = "RIGHT Bumper touched";
				blink_goal.goal.colors = colors;
				blink_goal.goal.blink_duration = ros::Duration(4.0);
				blink_goal.goal.blink_rate_mean = 0.2;
				blink_goal.goal.blink_rate_sd = 0.05;
				leds_pub.publish(blink_goal);
			}
			else{
				blink_cancel.stamp = ros::Time::now();
				blink_cancel.id = "blink cancel";

				leds_cancel_pub.publish(blink_cancel);
			}
		}
	}

	void tactileCallback(const naoqi_bridge_msgs::HeadTouch::ConstPtr& tactileState)
	{ 

		/*
		 * recevied and move, reacktion 
		 */
    if (tactileState->button==tactileState->buttonFront)
    {
      if (tactileState->state == tactileState->statePressed)
      {
        ROS_WARN_STREAM("FRONT tactile has been Pressed!");
        
        /*set vocabulary parameters*/
        naoqi_bridge_msgs::SetSpeechVocabularyActionGoal voc_param;
        voc_param.goal_id.id="vocabulary";
        voc_param.goal.words.push_back("hello");
        voc_param.goal.words.push_back("how");
        voc_param.goal.words.push_back("are");
		voc_param.goal.words.push_back("you");
        voc_params_pub.publish(voc_param);
        std_srvs::Empty emp1;
        /*
		 *start processing
		 */
        if(recog_start_srv.call(emp1))
        {
         ROS_INFO("START Recording");
        }
      }
    }
    if (tactileState->button==tactileState->buttonMiddle)
    {
      if (tactileState->state == tactileState->statePressed){
        ROS_WARN_STREAM("MIDDLE tactile has been Pressed!");
        std_srvs::Empty emp2;
        /*
		 *stop
		 */
        if(recog_stop_srv.call(emp2))
            {
             ROS_INFO("End recording!");
            }
        /*
		 *talk
		*/
        speech_pub.publish(speech);
        speech.goal.say="";
      }
    }

    if (tactileState->button==tactileState->buttonRear)
    {
       	if (tactileState->state == tactileState->statePressed){ 
			
			if (!walking){			
				ROS_WARN_STREAM("Start walking!");
				walking = true;			
				walker(0.2,0.0,0.0);
				sleep(6);
				walker(0.0,0.0,1.57);
				sleep(5);
				walker(0.0,0.0,-3.14);
			}
			else{
				ROS_WARN_STREAM("Stop walking!");
				walking = false;
				stopWalk();
			}      
        }
    }
  }


	void main_loop()
	{
		ros::Rate rate_sleep(10);
		int step = 0;
		
		while(nh_.ok())
		{
			/*
			 * control
			 */
			if(!foot_contact)
			{
				ROS_WARN_STREAM("Not on the ground!");
				walking=false;
				stopWalk();
			}
			
			rate_sleep.sleep();
		}
	}

	void walker(double x, double y, double theta)
	{
		/*
		 * for WALKING
		 */
		geometry_msgs::Pose2D Postion2D;
    	Postion2D.x = x;
    	Postion2D.y = y;
    	Postion2D.theta = theta;
    	walk_pub.publish(Postion2D);
	}

	void stopWalk()
	{
		/*
		 * stop walking
		 */
		std_srvs::Empty empty_req;
		stop_walk_srv.call(empty_req);
      	
	}

};
int main(int argc, char** argv)
{
	ros::init(argc, argv, "tutorial_control");

	ros::NodeHandle n;
	ros::Rate rate_sleep(20);
	Nao_control ic;
	ic.main_loop();
	return 0;

}
