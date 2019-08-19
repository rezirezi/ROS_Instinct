#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Range.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include "Instinct.h"
#include <iostream>
#include <time.h>
#include <string>
#include <fstream>





#define WORLD_X 50
#define WORLD_Y 20

#define ACTION_MOVE_FORWARDS 1
#define ACTION_TURN_LEFT 2
#define	ACTION_TURN_RIGHT 3
#define ACTION_TURN_BACK 4
#define ACTION_SET_SPEED 5
#define ACTION_IMMEDIATE_STOP 6
#define ACTION_ROTATE_RIGHT 7
#define ACTION_ROTATE_LEFT 8
#define ACTION_FACE_DEST 9
#define ACTION_SLEEP 10

#define SENSE_FL 1
#define SENSE_FR 2
#define SENSE_BL 3
#define SENSE_BR 4
#define SENSE_FC 5  //comparing two front sensors, if right is further it's + if left is further its -
#define SENSE_BC 6  //comparing two back sensors, if right is further it's + if left is further its -
#define SENSE_ORIENT 7
#define SENSE_X 8
#define SENSE_Y 9
#define SENSE_IS_DEST 10
#define SENSE_FRONT_MIN 11



const char *pNodeTypeNames[INSTINCT_NODE_TYPES] =
{ "AP" ,"APE" , "C" , "CE" , "D" , "A" };


int senseFL(float f);
float IntToF(int i);

class Robot : public Instinct::Senses, public Instinct::Actions, public Instinct::Monitor
{
public:
	float Pos_x;
	float Pos_y;
	float Speed = 0;
	float AngularVel = 0;
	float Orient = 0;
	Instinct::CmdPlanner *_pPlan;
	Instinct::Names *_pNames;
	Robot();
	~Robot();
	
	int readSense(const Instinct::senseID nSense);
	unsigned char executeAction(const Instinct::actionID nAction, const int nActionValue, const unsigned char bCheckForComplete);

	unsigned char nodeExecuted(const Instinct::PlanNode *pPlanNode) {
		unsigned char action = 'e';
		return action;
	};
	unsigned char nodeSuccess(const Instinct::PlanNode *pPlanNode) {
		unsigned char action = 's';
		return action;
	};
	unsigned char nodeInProgress(const Instinct::PlanNode *pPlanNode) {
		unsigned char action = 'a';
		return action;
	};
	unsigned char nodeFail(const Instinct::PlanNode *pPlanNode) {
		unsigned char action = 'a';
		return action;
	};
	unsigned char nodeError(const Instinct::PlanNode *pPlanNode) {
		unsigned char action = 'a';
		return action;
	};
	unsigned char nodeSense(const Instinct::ReleaserType *pReleaser, const int nSenseValue) {
		unsigned char action = 'a';
		return action;
	};
	unsigned char runPlan(void)
	{
		//printf("run plan\n");
		unsigned char bRtn;
		_pPlan->processTimers(1);
		bRtn = _pPlan->runPlan();
		return bRtn;
	}

	void LoadPlan() {

		char szMsgBuff[80];

		_pPlan->executeCommand("R C", szMsgBuff, sizeof(szMsgBuff));
		_pPlan->executeCommand("R I 20 20 20 20 20 20", szMsgBuff, sizeof(szMsgBuff));
		

		_pPlan->addDrive(200,400,30,0,10,INSTINCT_COMPARATOR_TR, 30,0,0,0,0,0);

		_pPlan->addCompetence(400,1);
		_pPlan->addCompetenceElement(401,400,202,1,0,0,INSTINCT_COMPARATOR_TR,0,0,0);
		
		_pPlan->addActionPattern(201);
		_pPlan->addActionPatternElement(202,201,300,1);
		_pPlan->addActionPatternElement(203,201,301,2);
		_pPlan->addActionPatternElement(204,201,302,3);
		_pPlan->addActionPatternElement(205,201,303,4);

		_pPlan->addAction(300, ACTION_SET_SPEED, 1);
		_pPlan->addAction(301, ACTION_SLEEP, 1);
		_pPlan->addAction(302, ACTION_IMMEDIATE_STOP, 1);
		_pPlan->addAction(303, ACTION_SET_SPEED, 1);

		
		/*
		_pPlan->addAction(201, ACTION_SET_SPEED, 1);

		_pPlan->addDrive(300,301,32,0,10,INSTINCT_COMPARATOR_GT, 29,0,0,0,0,0);
		_pPlan->addAction(301, ACTION_IMMEDIATE_STOP, 1);
		*/
		
		

		/*
		_pPlan->addDrive(200,201,30,0,1,INSTINCT_COMPARATOR_GT, 41,0,0,0,0,0);
		_pPlan->addAction(201, ACTION_SET_SPEED, 1);

		_pPlan->addDrive(300,301,32,0,1,INSTINCT_COMPARATOR_LT, 30,0,0,0,0,0);
		_pPlan->addAction(301, ACTION_IMMEDIATE_STOP, 1);
		
		
		_pPlan->addDrive(400,401,31,0,1,INSTINCT_COMPARATOR_LT, 40,0,0,0,0,0);
		_pPlan->addAction(401, ACTION_ROTATE_LEFT, 90);
		

		/*
		_pPlan->addActionPattern(90);
		_pPlan->addActionPatternElement(91, 90, 51, 1);
		_pPlan->addActionPatternElement(92, 90, 52, 2);
		_pPlan->addActionPatternElement(93, 90, 51, 3);
		_pPlan->addActionPatternElement(94, 90, 51, 3);
		_pPlan->addActionPatternElement(95, 90, 53, 4);

		*/

	};

	void LoadPln(std::string file_path);
	

private:

};
Robot::Robot()
{
	Instinct::instinctID nPlanSize[INSTINCT_NODE_TYPES] = { 0, 0, 0, 0, 0, 0 };
	_pPlan = new Instinct::CmdPlanner(nPlanSize, this, this, this);
}

Robot::~Robot()
{
}
