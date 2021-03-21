#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H


#include <vector>
#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using std::string;
using std::vector;


/*--- The objetive of this class if to perform countain all the logic and methods regaring the 
planning of an viable trajectory. It will receveive all the needed information, like previous_path, 
which indicates how much the car has travelled along the previous generated path and generate
more smooth points toward the new trajectory. As viewed at the "Questions and Answers" video, the proccess
of generating a new path seems to have state so, make sense to design a class to store the data needed 
for the plapnning procces and therefore provide the methods for extending ith. */

struct CarState {
	explicit CarState( ) :
	id( -1 ),
	x( 0.0 ),
	y( 0.0 ),
	speed( 0.0),
	yaw( 0.0 ),
	s( 0.0 ),
	d( 0.0 ) {

	}

	explicit CarState( int id_, double x_, double y_, double speed_, double yaw_, double s_, double d_ ) :
	id( id_ ),
	x( x_ ),
	y( y_ ),
	speed( speed_ ),
	yaw( yaw_ ),
	s( s_ ),
	d( d_ ) {

	}

	int id;

	double x;
	double y;

	double speed;

	double yaw;
	double s;
	double d;
};

struct Path {
	bool fromJSON( const nlohmann::json& px, const nlohmann::json& py) {

		if( px.size() == py.size() ) {
			x.resize( px.size() );
			y.resize( py.size() );

			for( size_t i = 0; i < px.size(); ++i ) {
				x[i] = px[i];
				y[i] = py[i];
			}

			hasEuclidean = true;
			return true;
		}
		return false;
	}

	double targetD;

	//Path represetation in frenet coordinates
	vector< double > s;
	vector< double > d;

	//Path represetation in Euclidean coordinates
	vector< double > x;
	vector< double > y;

	bool hasFrenet = false;
	bool hasEuclidean = false;
};

class PathPlanner {
public:
	explicit PathPlanner( const vector<double> & mx, const vector<double> & my, const vector<double> & ms );
	~PathPlanner();

 
	bool generatePath( const CarState &st, const Path &lastPath, double targetD, double targetSpeed, Path &newPath );
	bool generateLCPath( const CarState &st, const Path &lastPath, double targetD, double targetSpeed, Path &newPath );

private:
	//Maps "loaded from outside"
	const vector<double> & map_x;
	const vector<double> & map_y;
	const vector<double> & map_s;

	size_t pathSize;

	double lastTargetD;

	double sameLameRefDInc;


private:


};


#endif //PATH_PLANNER_H