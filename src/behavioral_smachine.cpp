#include "behavioral_smachine.h"


BehavioralSMachine::BehavioralSMachine( const vector< double > &mx, const vector< double > &my,
								 		const vector< double > &ms, const vector< double > &dx, const vector< double > &dy ) :
	m_state( KL),
	m_currentLane( 1 ),
	m_targetLane( 1 ),
	mapWaypointsX( mx ),
	mapWaypointsY( my ), 
	mapWaypointsS( ms ), 
	mapWaypointsDX( dx ),
	mapWaypointsDY( dy ),
	m_planner( new PathPlanner( mx, my, ms ) ),
	m_targetSpeed( 0.0 ),
	m_speedInc( 0.05 ),
	m_keepLaneCounter( 0 ) {

}


BehavioralSMachine::~BehavioralSMachine() { 
}


Path BehavioralSMachine::update( const CarState &currentState, const vector< CarState > &cars, const Path & currentPath ) {
	Path nextPath;

	switch( m_state ) {
		case KL: {
			nextPath = klState( currentState, cars, currentPath);
		}
		break;

		case PLC: {
			nextPath = plcState( currentState, cars, currentPath );
		}
		break;

		case LC: {
			nextPath = lcState( currentState, cars, currentPath );
		}
		break;

	}

	return nextPath;

}


vector< CarState > BehavioralSMachine::getCarsAtLane( int laneId, const vector< CarState > &cars ) {

	double currentD = LANE_WIDTH*( 0.5 + laneId );

	vector< CarState > carsInThisLane;

	for( const CarState &car : cars ) {
		if( car.d > ( currentD - 2.0 ) && car.d < ( currentD + 2.0 ) ) {
			carsInThisLane.push_back( car );
		}
	}

	return carsInThisLane;

}


CarState BehavioralSMachine::getNextCarInLane( const CarState &currentState, const vector< CarState > & carsInThisLane ) {
	double nearCarDistance = 1e6;
	double distance = 1e6;

	bool valid = false;

	CarState nextCarState;

	for( const CarState &car : carsInThisLane ) {
		if( ( car.s - 2 ) > ( currentState.s + 2) ) {
			distance = ( car.s - currentState.s );

			if( distance < nearCarDistance ) {
				nearCarDistance = distance;
				nextCarState = car;
				valid = true;
			}
		}
	}

	return nextCarState;
}


int BehavioralSMachine::getLaneChangeCost( int laneId, const CarState &currentState, const vector< CarState > &cars ) {
	//Not  rightmost lane, let's see if we can find a spot to change lane
	vector< CarState > nextLaneCars = getCarsAtLane( laneId, cars );

	if( laneId < 0 || laneId > 2 ) {
		return -1;
	}

	int laneChangeCost = 0;

	CarState nextCar;
	CarState precedingCar;	

	double nextCarDist = 1e6;
	double precedingCarDist = 1e6;

	for( const CarState & car : nextLaneCars ) {

		double distance = ( car.s - currentState.s );

		if( distance >= 0 ) {
			if( fabs(distance) < nextCarDist ) {
				nextCarDist = fabs(distance);
				nextCar = car;
			}

		}else {
			if( fabs(distance) < precedingCarDist ) {
				precedingCarDist = fabs(distance);
				precedingCar = car;
			}
		}

		//Here we will check if is impossible to change lanes, if so, doesn't matter the cost
		//we will not change lanes
		constexpr double MINIMUM_DIST_AHEAD = -TAKEOVER_AHEAD_LENGHT_MULTIPLIER*CAR_LENGHT;


		bool invalidCarPos = 	distance > MINIMUM_DIST_AHEAD && 
								distance < TAKEOVER_MINIMUM_REQUIRED_SPACE;

		if( invalidCarPos ) {
			return -1;
		}else {
			
			//The car is behind us and it's faster
			if( distance < 0 && ( car.speed - currentState.speed ) > MINIMUM_SPEED_DIFF ) {
				if( fabs(distance) < TAKEOVER_MINIMUM_REQUIRED_DISTANCE ) {
					//To close for a faster car, do not change lane
					return -1;
				}

			}
		}
	}


	//Clear lane. 
	laneChangeCost = MAXIMUM_COST;
	
	if( nextCar.id != -1 ) {
		//If we endup here, so we have a valid lane change, let's rate it
		double speedDif = ( nextCar.speed - currentState.speed );
		double distance = ( nextCar.s - currentState.s );

		//Car at 20 meters - Maximum cost (0)
		//Car at 200 meters - minumum cost( 100 )
		constexpr double DISTANCE_COST_MULTIPLIER = MAXIMUM_COST/(CLEAR_LANE_DISTACE - TAKEOVER_MINIMUM_REQUIRED_SPACE);

		laneChangeCost = round(distance - TAKEOVER_MINIMUM_REQUIRED_SPACE)*DISTANCE_COST_MULTIPLIER;

		if( speedDif > MINIMUM_SPEED_DIFF )  {
			//A faster car, if we need to change lane, this would be better
			laneChangeCost += round(speedDif);

		}

				//Capping the value
		if( laneChangeCost > MAXIMUM_COST ) {
			laneChangeCost = MAXIMUM_COST;
		}


	}

	return laneChangeCost;

}

double BehavioralSMachine::followNextVehicle( const CarState &currentState, const CarState &nextCarState ) {
	const double distance = nextCarState.s - currentState.s;


	//Let's try to "follow" the next car at costand distance
	//Some sort of PID controller
	const double distanceError = DESIRED_FOLLOWING_DISTANCE - distance;
	const double speedDif = nextCarState.speed - currentState.speed;


	double speedOutput = 5.0*m_speedInc;

	//Close enought to start matching speed and maintain a distance
	if( distance < 2.0*DESIRED_FOLLOWING_DISTANCE ) {

		speedOutput = -( distanceError*SPEED_MATCH_CONTROLLER_KP - speedDif*SPEED_MATCH_CONTROLLER_KD);
		
		if( speedOutput > MAXIMUM_ACCELERATION ) {
			speedOutput = MAXIMUM_ACCELERATION;
		}else if( speedOutput < -MAXIMUM_DECELERATION ) {
			speedOutput = -MAXIMUM_DECELERATION;
		}
	}

	if( distance > 0 && distance < MINIMUM_DISTANCE ) {
		//TOO close, emergency stop
		speedOutput -= 3.0*MAXIMUM_ACCELERATION;

	}

	if( currentState.speed > MAX_SPEED) {
		speedOutput = 0.0;
	}

	return speedOutput;
}


Path BehavioralSMachine::klState( const CarState &currentState, const vector< CarState > &cars, const Path & currentPath ) {

	vector< CarState > carsInThisLane = getCarsAtLane( m_currentLane, cars );

	CarState nextCar = getNextCarInLane( currentState, carsInThisLane );
	
	bool toClose = false;

	if( nextCar.id != -1 ) {

		double distance = ( nextCar.s - currentState.s );
		toClose = distance > 0 && distance < DISTANCE_TO_LANE_CHANGE;
		
		m_targetSpeed += followNextVehicle( currentState, nextCar );

	}else {
		m_targetSpeed += 5.0*m_speedInc;
	}
	
	if( m_targetSpeed >= MAX_SPEED ) {
		m_targetSpeed = MAX_SPEED;
	}


	//We are at least for one second in this lane and we are fast enought for lane change
	if( toClose && m_keepLaneCounter > 10 ) {
		/*--- This where is something "fun" happen. Let's search for a spot for lane change. ---*/
		int leftLaneIndex = m_currentLane - 1;
		int rightLaneIndex = m_currentLane + 1;

		int leftLaneChangeCost =   getLaneChangeCost( leftLaneIndex, currentState,	 cars );
		int rightLanechangeCost =  getLaneChangeCost( rightLaneIndex, currentState,	 cars );
		

		if( leftLaneChangeCost >= 0 && rightLanechangeCost >= 0 ) {
			//Both are valid, 
			if( leftLaneChangeCost > rightLanechangeCost ) {
				//Left lane change better than right lane change
				goToPlcState( leftLaneIndex );
			}else {
				//Right lane change better than left lane change
				goToPlcState( rightLaneIndex );
			}
		}else if( leftLaneChangeCost >= 0) {
			//Left is the unique valid
			goToPlcState( leftLaneIndex );
		}else if( rightLanechangeCost >= 0 ) {
			goToPlcState( rightLaneIndex );
		}

	}else {
		++m_keepLaneCounter;	
	}


	Path nextPath;
	double targetD = LANE_WIDTH*(0.5 + m_currentLane);
	m_planner->generatePath( currentState, currentPath, targetD, m_targetSpeed, nextPath );

	return nextPath;
}

Path BehavioralSMachine::plcState( const CarState &currentState, const vector< CarState > &cars, const Path & currentPath ) {
	vector< CarState > carsInThisLane = getCarsAtLane( m_currentLane, cars );	
	
	CarState nextCar = getNextCarInLane( currentState, carsInThisLane );

	if( nextCar.id != -1 ) {
		m_targetSpeed += followNextVehicle( currentState, nextCar );
	}else {
		m_targetSpeed += 5.0*m_speedInc;
	}
	
	if( m_targetSpeed >= MAX_SPEED ) {
		m_targetSpeed = MAX_SPEED;
	}

	
	/*--- Let's generate some trajectories. ---*/
	Path proposalPath;
	double targetD = LANE_WIDTH*(0.5 + ( m_targetLane ) );
	m_planner->generatePath( currentState, currentPath, targetD, m_targetSpeed, proposalPath );	
	
	bool okToLaneChange = true;

	if( okToLaneChange ) {
		goToLcState();
		return proposalPath;
	}

	/*--- .---*/


	Path nextPath;
	targetD = LANE_WIDTH*(0.5 + m_currentLane);
	m_planner->generatePath( currentState, currentPath, targetD, m_targetSpeed, nextPath );

	return nextPath;


}

Path BehavioralSMachine::lcState( const CarState &currentState, const vector< CarState > &cars, const Path & currentPath ) {
	//We are currently lane changing, let's stay here until we effectively change lane, then go to the keep lane state
	vector< CarState > carsInThisLane = getCarsAtLane( m_currentLane, cars );	
	
	//The difference between the keep lane, is that we need to match next lane car speed
	CarState nextCar = getNextCarInLane( currentState, carsInThisLane );

	if( nextCar.id != -1 ) {
		m_targetSpeed += followNextVehicle( currentState, nextCar );;
	}else {
		m_targetSpeed += 5.0*m_speedInc;
	}
	
	if( m_targetSpeed >= MAX_SPEED ) {
		m_targetSpeed = MAX_SPEED;
	}

	
	/*--- Let's generate some trajectories. ---*/
	Path proposalPath;
	double targetD = LANE_WIDTH*(0.5 + ( m_targetLane ) );
	m_planner->generatePath( currentState, currentPath, targetD, m_targetSpeed, proposalPath );	
	
	bool okToLaneChange = true;

	//Let's check if we finished changing lane
	if( currentState.d > ( targetD - 1.0 ) && currentState.d < ( targetD + 1.0 ) ) {
		m_currentLane = 0;
		goToKlState();
	}

	return proposalPath;

}


void BehavioralSMachine::goToKlState( ) {
	m_state = KL;

	m_currentLane = m_targetLane;

	m_keepLaneCounter = 0;
}

void BehavioralSMachine::goToPlcState( int targetLane ) {
	m_state = PLC;
	m_targetLane = targetLane;
}


void BehavioralSMachine::goToLcState( ) {
	m_state = LC;
}

