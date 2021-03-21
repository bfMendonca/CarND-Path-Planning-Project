#include "path_planner.h"


PathPlanner::PathPlanner( const vector<double> & mx, const vector<double> & my, const vector<double> & ms) :
	map_x( mx ),
	map_y( my ),
	map_s( ms ), 
	pathSize( 50 ),
	lastTargetD( -1 ),
	sameLameRefDInc( 50.0 ) {

}


PathPlanner::~PathPlanner() {

}


bool PathPlanner::generatePath( const CarState &st, const Path &lastPath, double targetD, double targetSpeed, Path &newPath ) {

	bool lcPath = false;

	if( lastTargetD != targetD ) {
		
		//We need to get rid of everying planned before and start from scratch
		//Planning for lane change
		lastTargetD = targetD;

		lcPath = true;
	}


	//We just need to "update" our path, actually, extended it

	double dist_inc = 0.4;

	double s = st.s;
	double d = floor( lastTargetD );

	vector< double> refPtsX, refPtsY; 
	refPtsX.clear();
	refPtsY.clear();

	vector< double > origin( 3, 0 );

	//We have and valid euclidean path and it's size is greater than 2
	if( lastPath.hasEuclidean and lastPath.x.size() >= 2 ) {
		size_t lIdx = lastPath.x.size() - 1;
		size_t llIdx = lastPath.x.size() - 2;

		//Keeping the last two euclidean coordinates to keep the trajectory
		vector< double > lp{ lastPath.x[lIdx], lastPath.y[lIdx] };
		vector< double > llp{ lastPath.x[llIdx], lastPath.y[llIdx] };

		double dx = lp[0] - llp[0];
		double dy = lp[1] - llp[1];

		origin[0] = lp[0];
		origin[1] = lp[1];
		origin[2] = atan2( dy, dx );

		//Conveting to local frame
		lp = Utils::toLocalFrame( lp, origin );
		llp = Utils::toLocalFrame( llp, origin );

		refPtsX.push_back( llp[0] );
		refPtsY.push_back( llp[1] );

		refPtsX.push_back( lp[0] );
		refPtsY.push_back( lp[1] );


	}else {
		//Nothing bether to do, let's use the cars current state. The car is at the "origin"
		refPtsX.push_back( 0 );
		refPtsY.push_back( 0 );

		origin[0] = st.x;
		origin[1] = st.y;
		origin[2] = st.yaw/57.3;
	}

	vector< double > newEuclidean;
	double news;

	int counter = 0;
	while( refPtsX.size() < 6 ) {
		news = st.s + (counter + 1)*sameLameRefDInc;

		newEuclidean = Utils::getXY( news, lastTargetD, map_s, map_x, map_y );
		
		//Converting to local frame
		newEuclidean = Utils::toLocalFrame( newEuclidean, origin );

		refPtsX.push_back( newEuclidean.at(0) );
		refPtsY.push_back( newEuclidean.at(1) );


		++counter;
	}


	//Com essa lista de pontos, precisamos suavizar
	tk::spline splineInterpol;
	splineInterpol.set_points( refPtsX, refPtsY );


	//
	double tx = 30.0;
	double ty = splineInterpol( tx );
	double td = sqrt( pow( tx,2.0) + pow( ty,2.0) );

	double xStep = 0.0;

	constexpr double factor = 0.02/2.24;

	vector< double > coord( 2, 0.0 );

	for( size_t i = 0; i < lastPath.x.size(); ++i ) {
		newPath.x.push_back( lastPath.x[i] );
	  	newPath.y.push_back( lastPath.y[i] );
	}

	for( size_t i = 1; i <= pathSize - lastPath.x.size(); ++i ) {
	  	double N = td/(targetSpeed*factor);
	  	coord[0] = xStep + tx/N;
	  	coord[1] = splineInterpol( coord[0] ) ;

	  	xStep = coord[0];

		coord = Utils::fromLocalFrame( coord, origin );


	  // std::cout << distance(  map_waypoints_x[i-1], map_waypoints_y[i-1],
	  //                          map_waypoints_x[i], map_waypoints_y[i] ) << std::endl;
	  	newPath.x.push_back( coord[0] );
	  	newPath.y.push_back( coord[1] );
	}

	return true;

	

}