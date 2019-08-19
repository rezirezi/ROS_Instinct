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
#include "instinct_pkg_node.h"
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
	
	printf("sense id: %i \n", nSense);
	
	int sense = 0;
	int temp = 0;

	switch(nSense){
	
	case SENSE_FL:
		
		sense = senseFL(distFL);
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
		printf("sense front comp value: %i \n", sense);
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
	printf("sense is dest value: %i \n", sense);
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

	}

	return sense;

}


void Robot::LoadPln(std::string file_path){
	std::cout << "load plan\n";
	printf("loading plan\n");
	std::string fpath;
	fpath = file_path;
	//std::cout << fpath << '\n';
	char szMsgBuff[100];
	std::size_t found;
	std::string line;
	std::ifstream myfile("src/instinct_pkg/src/DiaPlan8.inst",std::ios::app );
	char szName[20];
	unsigned int uiID;
	const char* linep5;
	const char* linep6;

	if (myfile.is_open())
	{
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
				//uiID = std::stoi(uID);
				//strcpy(szName, name.c_str());
				//szName = name.c_str();
				//s.copy();
				//std::cout << name << '\n';
				std::cout << uiID << '\n';
				
				std::cout << szName << '\n';
				
				_pNames->addElementName(uiID, szName);
				printf("element added \n");
			
			}
		
			
		}
		myfile.close();
		printf("plan file closed \n");
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
	
	printf("action: %i \n",nAction);
	
	
	switch(nAction){
	
	case ACTION_SET_SPEED:
		
		printf("action set speed \n");
		
		s = IntToF(nActionValue);
		printf("%f \n", s);
		
		if(Speed != s){
			set_vel.linear.x = s;
			Speed = s;
			action_pub.publish(set_vel);
		}
		action = INSTINCT_SUCCESS;
		return action;
		
	break;

	case ACTION_IMMEDIATE_STOP:

		printf("action stop \n");
		
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

	case ACTION_ROTATE_LEFT:
		printf("action rotate \n");
		angVel = nActionValue*deg2rad;
		
		set_vel.linear.x = 0;
   		set_vel.angular.z = angVel;
		action_pub.publish(set_vel);

		sleep(1);
		
		set_vel.linear.x = 0;
   		set_vel.angular.z = 0;
		action_pub.publish(set_vel);
		Speed = 0;
		
		action = INSTINCT_SUCCESS;
		return action;
		

	break;
	case ACTION_ROTATE_RIGHT:
		
		angVel = nActionValue*deg2rad;
		
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


	case ACTION_FACE_DEST:
	
	printf("action face dest \n");

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

	case ACTION_TURN_LEFT:
		
		printf("action turn left \n");
	
		set_vel.linear.x = 0;
   		set_vel.angular.z = 1;
		action_pub.publish(set_vel);
		action = INSTINCT_SUCCESS;
		return action;
	break;

	case ACTION_TURN_RIGHT:
		
		printf("action turn right \n");		

		set_vel.linear.x = 0;
   		set_vel.angular.z = -1;
		action_pub.publish(set_vel);
		action = INSTINCT_SUCCESS;
		return action;
	break;

	case ACTION_SLEEP:
	
		sleep(nActionValue);
		printf("action sleep\n");
		return action;
	break;
	}
	
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
    rob.LoadPlan();
	std::string fpath;
	//n.getParam("~file_path",fpath);
	
    //rob.LoadPln(fpath);

    while (ros::ok())
    {	
	if(count < 10){
		alpha0 = alpha;
	}		
	else{
	rob.Orient = alpha - alpha0;
	rob.Pos_x = X;
	rob.Pos_y = Y;

	//printf("%f \n",rob.Orient);
	//printf("pos_x : %f \n", rob.Pos_x);	
	
	rob.runPlan();
	}
	
		

        ros::spinOnce();
        loop_rate.sleep();
	count++;
    }
}
