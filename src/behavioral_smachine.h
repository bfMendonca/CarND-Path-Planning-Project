#ifndef BEHAVIORAL_SMACHINE_H
#define BEHAVIORAL_SMACHINE_H

#include <iostream>
#include <memory>

#include "path_planner.h"

using std::vector;

class BehavioralSMachine {
public:
	enum States {
		KL = 0x00,	//Keep lane
		PLC,		//Prepare to change 
		LC			//Change lane
	};

	explicit BehavioralSMachine( const vector< double > &mx, const vector< double > &my,
								 const vector< double > &ms, const vector< double > &dx, const vector< double > &dy );
	virtual ~BehavioralSMachine();

	//Methods for updating the state machine
	Path update( const CarState &currentState, const vector< CarState > &cars, const Path & currentPath );

private:
	vector< CarState > getCarsAtLane( int laneId, const vector< CarState > &cars );
	CarState getNextCarInLane( const CarState &currentState, const vector< CarState > & carsInThisLane );

	int getLaneChangeCost( int laneId, const CarState &currentState, const vector< CarState > &cars );

	/*
	* - This methos will return a speed inc suitable for matching the speed of the next car
	* - If there is no valid car ahead, just speed up. 
	*/
	double followNextVehicle( const CarState &currentState, const CarState &nextCarState );

	//Methods that define the state machine conditions
	Path klState( 	const CarState &currentState, const vector< CarState > &cars, const Path & currentPath );
	Path plcState( 	const CarState &currentState, const vector< CarState > &cars, const Path & currentPath );
	Path lcState( 	const CarState &currentState, const vector< CarState > &cars, const Path & currentPath );

	//Methods for changing states
	void goToKlState( );
	void goToPlcState( int targetLane );
	void goToLcState( );


	int m_currentLane;
	int m_targetLane;

	const vector<double> & mapWaypointsX;
	const vector<double> & mapWaypointsY;
	const vector<double> & mapWaypointsS;
	const vector<double> & mapWaypointsDX;
	const vector<double> & mapWaypointsDY;

	std::unique_ptr< PathPlanner > m_planner;

	double m_targetSpeed;
	double m_speedInc;

	int m_keepLaneCounter;

	static constexpr double LANE_WIDTH = 4.0;
	static constexpr double MAX_SPEED = 49.5;
	static constexpr double MIN_SPEED = 20.0;

	static constexpr double CAR_LENGHT = 3.0;

	static constexpr double DISTANCE_TO_LANE_CHANGE = 50.0;

	static constexpr double TAKEOVER_AHEAD_LENGHT_MULTIPLIER = 3.0;

	static constexpr double TAKEOVER_MINIMUM_REQUIRED_SPACE = 30.0; //meters

	static constexpr double TAKEOVER_MINIMUM_REQUIRED_DISTANCE = 30.0; //meters

	static constexpr double CLEAR_LANE_DISTACE = 150.0;	//If a car is farther than this dist, assume clear lane

	static constexpr double MAXIMUM_COST = 100.0;

	static constexpr double MINIMUM_SPEED_DIFF = 3.0; //mph. Muminum speed that our car need to be faster to be considered faster than other cars

	static constexpr double DESIRED_FOLLOWING_DISTANCE = 15.0;	//meters. How many meters to stay behind the next car when matching speed

	static constexpr double SPEED_MATCH_CONTROLLER_KP = 0.005;
	static constexpr double SPEED_MATCH_CONTROLLER_KD = 0.09;

	static constexpr double MAXIMUM_ACCELERATION = 0.2;
	static constexpr double MAXIMUM_DECELERATION = 0.5;

	static constexpr double MINIMUM_DISTANCE = 7.5;

	States m_state;

};

#endif //BEHAVIORAL_SMACHINE_H