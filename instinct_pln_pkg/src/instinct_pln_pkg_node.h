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
#define ACTION_SET_FLAG 11
#define ACTION_SET_START 12

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
#define SENSE_FLAG 12



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
	int flag = 0;
	Instinct::CmdPlanner *_pPlan;
	Instinct::Names *_pNames;
	Robot();
	~Robot();
	
	int readSense(const Instinct::senseID nSense);
	unsigned char executeAction(const Instinct::actionID nAction, const int nActionValue, const unsigned char bCheckForComplete);

	unsigned char nodeExecuted(const Instinct::PlanNode *pPlanNode) {
	
		/*
		printf("node executed: ");
		std::cout << pNodeTypeNames[pPlanNode->bNodeType];
		std::cout << pPlanNode->sElement.sReferences.bRuntime_ElementID << '\n';
		std::cout << pPlanNode << '\n';
		*/
		
		return true;
	};
	unsigned char nodeSuccess(const Instinct::PlanNode *pPlanNode) {
		/*
		printf("node successfull: ");
		std::cout << pNodeTypeNames[pPlanNode->bNodeType];
		std::cout << pPlanNode->sElement.sReferences.bRuntime_ElementID << '\n';
		*/
		
		return true;
	};
	unsigned char nodeInProgress(const Instinct::PlanNode *pPlanNode) {
		
		/*
		printf("node in progress: ");
		std::cout << pNodeTypeNames[pPlanNode->bNodeType];
		std::cout << pPlanNode->sElement.sReferences.bRuntime_ElementID << '\n';
		*/
		return true;
	};
	unsigned char nodeFail(const Instinct::PlanNode *pPlanNode) {
		unsigned char action = 'a';
		//printf("node fail: ");
		//std::cout << pNodeTypeNames[pPlanNode->bNodeType];
		//std::cout << pPlanNode->sElement.sReferences.bRuntime_ElementID << '\n';
		return true;
	};
	unsigned char nodeError(const Instinct::PlanNode *pPlanNode) {
		
		/*
		printf("node error: ");
		std::cout << pNodeTypeNames[pPlanNode->bNodeType];
		std::cout << pPlanNode->sElement.sReferences.bRuntime_ElementID << '\n';
		*/
		return true;
	};
	unsigned char nodeSense(const Instinct::ReleaserType *pReleaser, const int nSenseValue) {
		
		//printf("node sense: %i\n", nSenseValue);
		return true;
	};
	unsigned char runPlan(void)
	{
		//printf("run plan\n");
		unsigned char bRtn;
		_pPlan->processTimers(1);
		bRtn = _pPlan->runPlan();
		return bRtn;
	}


private:

};
Robot::Robot()
{
	Instinct::instinctID nPlanSize[INSTINCT_NODE_TYPES] = { 0, 0, 0, 0, 0, 0 };
	_pPlan = new Instinct::CmdPlanner(nPlanSize, this, this, this);
	_pNames = new Instinct::Names(4000);
}

Robot::~Robot()
{
}
