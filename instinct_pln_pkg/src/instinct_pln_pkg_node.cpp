#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Range.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include "Instinct.h"
#include "instinct_pln_pkg_node.h"
#include <math.h>
#include <string>
#include <fstream>
#include <iostream>


ros::Publisher action_pub;
geometry_msgs::PoseStamped pose;
geometry_msgs::Twist set_vel;

float deg2rad = 3.14159/180;

float distFL = 0;
float distFR = 0;
float distBL = 0;
float distBR = 0;

float alpha = 0;
float alpha0 = 0;

float X = 0;
float Y = 0;
float Z = 0;
float angVel= 0;

void distFL_callback(const sensor_msgs::Range &range) {
   distFL = range.range;
}


void distFR_callback(const sensor_msgs::Range &range) {
   distFR = range.range;
   
}

void distBL_callback(const sensor_msgs::Range &range) {
   distBL = range.range;
}


void distBR_callback(const sensor_msgs::Range &range) {
   distBR = range.range;
   
}

void imu_callback(const geometry_msgs::Vector3 &imu)
{
	alpha = imu.z;
	//printf("alpha: %f\n",alpha);
}

void pose_callback(const geometry_msgs::PoseStampedPtr &pose) {
   	X = pose->pose.position.x;
	Y = pose->pose.position.y;
	Z = pose->pose.orientation.z;
	
}

int senseFL(float f){
	int s = (int)(f*100);
	return s;	
	
}

float IntToF(int i){

	float f = (float)i/10;
	return f;

}

int Robot::readSense(const Instinct::senseID nSense){
	
	//printf("sense id: %i \n", nSense);
	
	int sense = 0;
	int temp = 0;

	switch(nSense){
	
	case SENSE_FL:
		
		sense = senseFL(distFL);
		printf("fl dist: %i \n", sense);
		return sense;		
	break;

	case SENSE_FR:
		sense = senseFL(distFR);
		return sense;
	break;

	case SENSE_BL:
		sense = senseFL(distBL);
		return sense;
	break;

	case SENSE_BR:
		sense = senseFL(distBR);
		return sense;
	break;

	case SENSE_FC:
		sense = senseFL(distFR);
		temp = senseFL(distFL);
		sense = sense - temp;
		//printf("sense front comp value: %i \n", sense);
		return sense;
	break;

	case SENSE_BC:

 				
		sense = senseFL(distBR);
		temp = senseFL(distBL);
		sense = sense -temp;
		return sense;
	break;
	
	case SENSE_ORIENT:
	sense = (int)Orient;
	return sense;
	break;	

	case SENSE_X:
	sense = Pos_x;
	return sense;
	break;

	case SENSE_Y:
	sense = Pos_y;
	return sense;
	break;

	case SENSE_IS_DEST:
	sense = (int)(Pos_x*10);
	//printf("sense is dest value: %i \n", sense);
	return sense;
	break;
	
	case SENSE_FRONT_MIN:

	if(distFL <= distFR){
	sense = senseFL(distFL);
	}
	else{
	sense = senseFL(distFR);
	}	
	printf("sense front min value: %i \n", sense);	
	
	return sense;
	break;


	case SENSE_FLAG:
	sense = flag;
	return sense;
	break;	
	

	}

	return sense;

}


void Robot::LoadPln(std::string file_path){

	printf("loading plan\n");
	std::string fpath;
	fpath = file_path;
	std::cout << fpath << '\n';
	char szMsgBuff[100];
	std::size_t found;
	std::string line;

	std::ifstream myfile(fpath,std::ios::app );
	char szName[20];
	unsigned int uiID;
	const char* linep5;
	const char* linep6;

	if (myfile.is_open())
	{	
		_pPlan->executeCommand("M G 1 1 1 1 1 1",szMsgBuff, sizeof(szMsgBuff));	
		printf("file opened\n");
		while (std::getline(myfile, line))
		{
			//printf("reading line\n");
			if (line.find("PLAN") == 0) {
				
				linep5 = &line[5];
				

				if(!(_pPlan->executeCommand(linep5, szMsgBuff, sizeof(szMsgBuff)))){
						printf("failed to execute command \n");
					}
				
				std::cout << line << '\n';
			}
			else if (line.find("PELEM")==0) {

				found = line.find("=");
				line.at(found) = ' ' ;

				std::string name = line.substr(6,found-6);
				std::string uID = line.substr(found+1, line.size()-found);

				linep6 = &line[6];
				std::cout << line << '\n';				
				
				sscanf(linep6, "%s %u", szName, &uiID);

				
				_pNames->addElementName(uiID, szName);
			
			}
		
			
		}
		myfile.close();
		printf("plan loaded\n");
	}
	
	else printf("unable to open file \n");
	

	
	
}


unsigned char Robot::executeAction(const Instinct::actionID nAction, const int nActionValue, const unsigned char bCheckForComplete){
	unsigned char action = INSTINCT_FAIL;
	float s = 0;
	float angVel = 0;
	float dx = 0;
	float dy = 0;
	float rx = 0;
	float ry = 0;
	double time_begin = 0;
	double time_end = 0;
	
	//printf("action: %i \n",nAction);
	
	
	switch(nAction){
	

	case ACTION_MOVE_FORWARDS:
		{
		set_vel.linear.x = 0.1;		

		float ActTime = (float)nActionValue/2;

		time_begin = ros::Time::now().toSec();
		time_end = ros::Time::now().toSec();
		while(time_end - time_begin < ActTime){
		time_end = ros::Time::now().toSec();
		}
		
		set_vel.linear.x = 0;

		action = INSTINCT_SUCCESS;
		return action;
		
	break;	
	}

	case ACTION_SET_SPEED:
		{
		s = IntToF(nActionValue);
		
		if(Speed != s){
			set_vel.linear.x = s;
			Speed = s;
			action_pub.publish(set_vel);
		}
		action = INSTINCT_SUCCESS;
		return action;
		
	break;
	}

	case ACTION_IMMEDIATE_STOP:
		{

		if(Speed == 0 && AngularVel ==0){
			
		}
		else{
			set_vel.linear.x = 0;
   			set_vel.angular.z = 0;
			action_pub.publish(set_vel);
			Speed =0;
			AngularVel = 0;
		}
		
		action = INSTINCT_SUCCESS;
		return action;
		
	break;
	}

	case ACTION_ROTATE_LEFT:
		{
		angVel = nActionValue*deg2rad;
		
		set_vel.linear.x = 0;
   		set_vel.angular.z = angVel;
		action_pub.publish(set_vel);

		time_begin = ros::Time::now().toSec();
		time_end = ros::Time::now().toSec();
		while(time_end - time_begin < 1){
		time_end = ros::Time::now().toSec();
		}
		
		set_vel.linear.x = 0;
   		set_vel.angular.z = 0;
		action_pub.publish(set_vel);
		Speed = 0;
		
		action = INSTINCT_SUCCESS;
		return action;
		

	break;
		}
	case ACTION_ROTATE_RIGHT:
		{
		angVel = nActionValue*deg2rad;
		
		set_vel.linear.x = 0;
   		set_vel.angular.z = -angVel;
		action_pub.publish(set_vel);

		time_begin = ros::Time::now().toSec();
		time_end = ros::Time::now().toSec();
		while(time_end - time_begin < 1){
		time_end = ros::Time::now().toSec();
		}
		
		set_vel.linear.x = 0;
   		set_vel.angular.z = 0;
		action_pub.publish(set_vel);
		action = INSTINCT_SUCCESS;
		return action;
	break;
	}

	case ACTION_FACE_DEST:
	{

	dx = nActionValue/100;
	dy = nActionValue - dx*100;
	rx = (Pos_y*10);
	ry = (Pos_x*10);

	angVel = atan((dx-rx)/(dy-ry)) - (Orient*deg2rad);
	
		set_vel.linear.x = 0;
   		set_vel.angular.z = -angVel;
		action_pub.publish(set_vel);

		sleep(1);
		
		set_vel.linear.x = 0;
   		set_vel.angular.z = 0;
		action_pub.publish(set_vel);
		action = INSTINCT_SUCCESS;
		return action;
	break;
	}
	case ACTION_TURN_LEFT:
		{
		set_vel.linear.x = 0;
   		set_vel.angular.z = 1;
		action_pub.publish(set_vel);
		action = INSTINCT_SUCCESS;
		return action;
	break;
	}
	case ACTION_TURN_RIGHT:
		{
		set_vel.linear.x = 0;
   		set_vel.angular.z = -1;
		action_pub.publish(set_vel);
		action = INSTINCT_SUCCESS;
		return action;
	break;
	}
	case ACTION_SLEEP:
		{
		time_begin = ros::Time::now().toSec();
		time_end = ros::Time::now().toSec();
		while(time_end - time_begin < nActionValue){
		time_end = ros::Time::now().toSec();
		}
		action = INSTINCT_SUCCESS;
		return action;
	break;
	}
	case ACTION_SET_FLAG:
		{
		flag = nActionValue;
		
		action = INSTINCT_SUCCESS;
		return action;

	break;
	}
	case ACTION_SET_START:
	{
		Pos_x = 0;
		Pos_y = 0;
		
		action = INSTINCT_SUCCESS;
		return action;
	break;
	}

	}
	
	return action;
	
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "instinct_node");
    ros::NodeHandle n("~");

    ros::Subscriber distFL_sub = n.subscribe("/range/fl", 1, distFL_callback);
    ros::Subscriber distFR_sub = n.subscribe("/range/fr", 1, distFR_callback);
    ros::Subscriber distBL_sub = n.subscribe("/range/bl", 1, distBL_callback);
    ros::Subscriber distBR_sub = n.subscribe("/range/br", 1, distBR_callback);
    ros::Subscriber posit_sub = n.subscribe("/pose", 1, pose_callback);
    ros::Subscriber imu_sub = n.subscribe("/rpy", 1, imu_callback);
    
    action_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::Rate loop_rate(10);
    set_vel.linear.x = 0;
    set_vel.angular.z = 0;
    action_pub.publish(set_vel);


    int count = 0;
    
    printf("I am starting probably\n");
	
    Robot rob;  
    std::string fpath;
    n.getParam("file_path",fpath);
	
    rob.LoadPln(fpath);
    while (ros::ok())
    {	
	if(count < 10){
		alpha0 = alpha;
	}		
	else{
	rob.Orient = alpha - alpha0;
	rob.Pos_x = X;
	rob.Pos_y = Y;
	
	rob.runPlan();
	}
        ros::spinOnce();
        loop_rate.sleep();
	count++;
    }
}
