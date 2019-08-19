// RoboWorld.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <time.h>
#include "Windows.h"
#include "instinct/Instinct.h"

#define WORLD_X 50
#define WORLD_Y 20

#define ACTION_MOVE_FORWARDS 1
#define ACTION_TURN_LEFT 2
#define	ACTION_TURN_RIGH 3
#define ACTION_TURN_BACK 4

const char *pNodeTypeNames[INSTINCT_NODE_TYPES] =
{ { "AP" },
	{ "APE" },
	{ "C" },
	{ "CE" },
	{ "D" },
	{ "A" },
};




class Robot : public Instinct::Senses, public Instinct::Actions, public Instinct::Monitor
{
public:
	int Pos_x;
	int Pos_y;
	char C;
	int Dir; // direction 1-n 2-e 3-s 4-w
	int Speed = 0;
	unsigned char cMap[WORLD_Y][WORLD_X]; // this is the 2D world
	Instinct::CmdPlanner *_pPlan;
	Robot();
	~Robot();
	int readSense(const Instinct::senseID nSense) {
		
		printf("sense\n");
		int s = 0;
		//printf("%i",s);
		bool sense;
		sense = SenseFront();

		if (sense)
		{
			s = 2;
		}
		else
		{
			s = 0;
		}

		return sense;
	};
	unsigned char executeAction(const Instinct::actionID nAction, const int nActionValue, const unsigned char bCheckForComplete) {
		unsigned char action = 'a';

		switch (nAction)
		{
		case ACTION_MOVE_FORWARDS:
			Move(nActionValue);
			break;

		case ACTION_TURN_LEFT:

			Dir--;
			if (Dir <= 0)
				Dir = 4;
			break;

		case ACTION_TURN_RIGH:
			Dir++;
			if (Dir > 4)
			{
				Dir = 0;
			}

		case ACTION_TURN_BACK:
			Dir += 2;
			if (Dir > 4)
			{
				Dir -= 4;
			}
		}



		return action;
	};
	unsigned char nodeExecuted(const Instinct::PlanNode *pPlanNode) {
		unsigned char action = 'a';
		return action;
	};
	unsigned char nodeSuccess(const Instinct::PlanNode *pPlanNode) {
		unsigned char action = 'a';
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
		printf("run plan\n");
		unsigned char bRtn;
		_pPlan->processTimers(1);
		bRtn = _pPlan->runPlan();
		//_pWorld->setRobotChar(this, '0' + INSTINCT_RTN_DATA(bRtn)); // enable this to show which action ID the robot last executed
		return bRtn;
	}
	void initialiseWorld() {

		for (int i = 0; i < WORLD_Y; i++) {
			for (int j = 0; j < WORLD_X; j++) {
				cMap[i][j] = ' ';
			}
		}


		for (int i = 0; i < WORLD_X; i++)
		{
			cMap[0][i] = 'X';
			cMap[WORLD_Y - 1][i] = 'X';
		}
		for (int j = 0; j < WORLD_Y; j++)
		{
			cMap[j][0] = 'X';
			cMap[j][WORLD_X - 1] = 'X';
		}

	}
	void SetSpeed(int s) {

		Speed = s;

	};
	bool SenseFront() {
		bool sense = false;

		switch (Dir)
		{
		case 1:
			if (Pos_y - Speed <= 0)
				sense = true;
			return sense;
			break;
		case 2:
			if (Pos_x + Speed >= WORLD_X - 1)
				sense = true;
			return sense;
			break;
		case 3:
			if (Pos_y + Speed >= WORLD_Y - 1)
				sense = true;
			return sense;
			break;
		case 4:
			if (Pos_x - Speed <= 0)
				sense = true;
			return sense;
			break;
		}

		return sense;


	};
	void ChangeDir(int d) {
		Dir = d;
	};

	unsigned char Move(int spd) {


		switch (Dir)
		{
		case 1:
			if (Pos_y - spd > 0) {
				Pos_y = Pos_y - spd;
				return INSTINCT_RTN_COMBINE(INSTINCT_SUCCESS, ACTION_MOVE_FORWARDS);
			}
			else
			{
				return INSTINCT_ERROR;
			}
			break;
		case 2:
			if (Pos_x + spd < WORLD_X - 1) {
				Pos_x = Pos_x + spd;
				return INSTINCT_RTN_COMBINE(INSTINCT_SUCCESS, ACTION_MOVE_FORWARDS);
			}
			else
			{
				return INSTINCT_ERROR;
			}
			break;
		case 3:
			if (Pos_y + spd < WORLD_Y - 1) {
				Pos_y = Pos_y + spd;
				return INSTINCT_RTN_COMBINE(INSTINCT_SUCCESS, ACTION_MOVE_FORWARDS);
			}
			else
			{
				return INSTINCT_ERROR;
			}
			break;
		case 4:
			if (Pos_x - spd > 0) {
				Pos_x = Pos_x - spd;
				return INSTINCT_RTN_COMBINE(INSTINCT_SUCCESS, ACTION_MOVE_FORWARDS);
			}
			else
			{
				return INSTINCT_ERROR;
			}
			break;
		}
	};

	void PrintMap() {

		unsigned char TempMap[WORLD_Y][WORLD_X];

		for (int m = 0; m < WORLD_Y; m++) {

			for (int n = 0; n < WORLD_X; n++) {
				TempMap[m][n] = cMap[m][n];
			}

		}

		TempMap[Pos_y][Pos_x] = 'A';

		for (int m = 0; m < WORLD_Y; m++) {

			for (int n = 0; n < WORLD_X; n++) {
				printf("%c", TempMap[m][n]);
			}

			printf("\n");
		}



	};

	void Tick() {

		Move(Speed);

	};

	void LoadPlan() {

		char szMsgBuff[80];

		_pPlan->executeCommand("R C", szMsgBuff, sizeof(szMsgBuff));
		_pPlan->executeCommand("R I 20 20 20 20 20 20", szMsgBuff, sizeof(szMsgBuff));


		_pPlan->addDrive(200,201,30,0,1,INSTINCT_COMPARATOR_LT, 1,0,0,0,0,0);
		//_pPlan->addDrive(300, 301, 32, 0, 1, INSTINCT_COMPARATOR_GT, 1, 0, 0, 0, 0, 0);
		_pPlan->addAction(201, ACTION_MOVE_FORWARDS, 1);

		/*
		_pPlan->addActionPattern(90);
		_pPlan->addActionPatternElement(91, 90, 51, 1);
		_pPlan->addActionPatternElement(92, 90, 52, 2);
		_pPlan->addActionPatternElement(93, 90, 51, 3);
		_pPlan->addActionPatternElement(94, 90, 51, 3);
		_pPlan->addActionPatternElement(95, 90, 53, 4);

		*/

	};
	

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



int main()
{
   // std::cout << "Hello World!\n"; 

	printf("start\n");

	Robot Rob;

	Rob.Pos_x = 25;
	Rob.Pos_y = 10;
	Rob.Dir = 1;
	Rob.Speed = 0;

	Rob.initialiseWorld();

	Rob.LoadPlan();
	//Rob._pPlan->addDrive(200, 201, 30, 0, 1, INSTINCT_COMPARATOR_LT, 1, 0, 0, 0, 0, 0);
	//Rob._pPlan->addAction(201, ACTION_MOVE_FORWARDS, 1);
	//std::cout << RobotWorld::cMap[1];
	Rob.PrintMap();

	unsigned char ch = Rob.runPlan();
	std::cout << ch;
	

	for (int i = 0; i < 15; i++) {

		Sleep(100);
		Rob.PrintMap();
		Rob.runPlan();

	}







/**
	for (int i = 0; i < 15; i++) {

		std::cout << Rob.SenseFront();
		if (Rob.SenseFront()) {
			std::cout << "true\n";
			Rob.ChangeDir(2);
		}

		if (!Rob.SenseFront()) {
			std::cout << "false\n";

		}


		Rob.Tick();
		Rob.PrintMap();
		Sleep(100);
	}
	
	for (int i = 0; i < 40; i++) {

		
		std::cout << Rob.SenseFront();
		if (Rob.SenseFront()) {
			std::cout << "true\n";
			Rob.ChangeDir(4);
		}

		if (!Rob.SenseFront()) {
			std::cout << "false\n";

		}

		Rob.Tick();
		Rob.PrintMap();
		Sleep(100);
	}


	for (int i = 0; i < 40; i++) {


		std::cout << Rob.SenseFront();
		if (Rob.SenseFront()) {
			std::cout << "true\n";
			Rob.ChangeDir(3);
		}

		if (!Rob.SenseFront()) {
			std::cout << "false\n";

		}

		Rob.Tick();
		Rob.PrintMap();
		Sleep(100);
	}

	**/


	//printf("%c\n", RobotWorld::cMap[0][0]);
	std::cout << "Hello World!\n";

}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
