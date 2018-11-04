#include <vector>
#include <iostream>
#include <random>
#include <tuple>
#include <map>
#include <algorithm>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include "path_planner.hpp"
#include "fsm.hpp"
#include "utils.hpp"

#define PATH_PLANNER_DEBUG true

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


Planner::Planner(const vector<double>& maps_x, const vector<double>& maps_y, const vector<double>& maps_s) {
	this->maps_x = maps_x;
	this->maps_y = maps_y;
	this->maps_s = maps_s;

	this->prevTimestamp = clock_time_ms();
	// this->prevXYAccelSD = vector<tuple<double, double, double, double, double, double> >();
}
Planner::~Planner() {}

void Planner::updatePrevPaths(const vector<double>& prevX, const vector<double>& prevY) {
	this->prevPathX = prevX;
	this->prevPathY = prevY;
}

vector<double> Planner::JMT(vector<double> start, vector<double> end, double T) {
	/*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    double si = start[0];
    double si_dot = start[1];
    double si_dot_dot = start[2];
    double sf = end[0];
    double sf_dot = end[1];
    double sf_dot_dot = end[2];
    double T3 = T * T * T;
    double T2 = T * T;
    double T4 = T * T * T * T;
    double T5 = T * T * T * T * T;
    
    MatrixXd A = MatrixXd(3,3);
    VectorXd B = VectorXd(3);
    A << T3, T4, T5,  3*T2, 4*T3, 5*T4, 6*T, 12*T2, 20*T3;
    B << sf - (si + si_dot * T + 0.5 * si_dot_dot * T2),
        sf_dot - (si_dot + si_dot_dot * T),
        sf_dot_dot - si_dot_dot;
    VectorXd ans = A.colPivHouseholderQr().solve(B);
    return {si, si_dot, 0.5 * si_dot_dot, ans(0), ans(1), ans(2)};
}

// v, targetSpeed is velocity in mph, appendOnly to generate a new path to add on to previous path
pair<vector<double>, vector<double> > Planner::generatePath(double targetSpeed, int targetLane, bool appendOnly) {
	// targetLane = 2;
	if (appendOnly) assert(prevPathX.size() > 1);
	unsigned long long currTimestamp = clock_time_ms();
	const double elapsedTime = (currTimestamp - prevTimestamp) / 1000.0;
	const double targetSpeedMs = mph2ms(targetSpeed);
	const double v_ms = mph2ms(v);

	// Generate trajectories for other cars for collision detection
	generateTrajectoriesForPredictions(0);
	
	// Specify initial conditions
	// double currAccelS, currAccelD;
	double start_x, start_y, start_s, start_d, start_yaw, start_speed, start_accel_s, start_accel_d;
	int size = prevXYAccelSD.size();
	int numOfXYPassed = prevXYAccelSD.size() - prevPathX.size();
	if (numOfXYPassed > 0) {
		start_accel_s = appendOnly ? get<2>(prevXYAccelSD[size-1]) : get<2>(prevXYAccelSD[numOfXYPassed]);
		start_accel_d = appendOnly ? get<3>(prevXYAccelSD[size-1]) : get<3>(prevXYAccelSD[numOfXYPassed]);
	} else {
		start_accel_s = 0.0;
		start_accel_d = 0.0;
	}
	start_x = appendOnly ? get<0>(prevXYAccelSD[size-1]) : x;
	start_y = appendOnly ? get<1>(prevXYAccelSD[size-1]) : y;
	start_s = appendOnly ? get<4>(prevXYAccelSD[size-1]) : s;
	start_d = appendOnly ? get<5>(prevXYAccelSD[size-1]) : d;
	start_yaw = appendOnly ? getYawAtEndOfPath() : yaw;
	start_speed = appendOnly ? getSpeedAtEndOfPath() : v_ms;
	
	// work in frenet coordinates
	const vector<double> start_speed_sd = getFrenetSpeed(start_x, start_y, start_yaw, start_speed*cos(start_yaw), start_speed*sin(start_yaw), maps_x, maps_y);

	// s = ut + 1/2 * a * t^2, assuming linear kinematics, we apply a correction factor, if not no solution exists
	const double ave_accel = min(8.0, (targetSpeedMs - start_speed) / PATH_PLANNING_HORIZON);
	const double displacement = (start_speed_sd[0] * PATH_PLANNING_HORIZON +
			 												0.5 * ave_accel * PATH_PLANNING_HORIZON * PATH_PLANNING_HORIZON) 
															* NONLINEARITY_CORRECTION_FACTOR;

	cout << s << ", start_s : " << start_s << " , end_s " << start_s + displacement << " , start_speed "  << start_speed << endl;
	// Generate a bunch of possible end configurations
	vector<vector<double> > possibleSTrajectories;
	vector<vector<double> > possibleDTrajectories;
	vector<double> Ts;
	const vector<tuple<double, double, double> > endConfig = generateEndConfigurations(SAMPLE_SIZE, fmod(start_s + displacement, MAX_S), LANE_CENTER[targetLane], PATH_PLANNING_HORIZON);
	
	if (PATH_PLANNER_DEBUG) cout << "Time elapsed: " << (currTimestamp - this->prevTimestamp) << " ms, start_accel_s " 
				<< start_accel_s << ", start_accel_d " << start_accel_d 
				<< " , num of pts passed " << numOfXYPassed <<  endl;

	for (auto p : endConfig) {
		const double startSDot = start_speed_sd[0];
		const double startDDot = start_speed_sd[1];
		const double endS = get<0>(p);
		const double endD = get<1>(p);
		const double T = get<2>(p);
		// cout << "Start S: " << startS << ", " << startSDot << ", " << startSDotDot << endl;
		// cout << "End S: " << endS << ", " << endSDot << ", " << endSDotDot << endl;
		// cout << "T is " << T << ", displacement " << displacement << ", " << endD << endl;
		const vector<double> startSCoeffs = { start_s, startSDot, start_accel_s };
		const vector<double> endSCoeffs = { endS, targetSpeedMs, 0 };
		const vector<double> startDCoeffs = { start_d, startDDot, start_accel_d };
		const vector<double> endDCoeffs = { endD, 0, 0 };

		vector<double> s_coeffs = JMT(startSCoeffs, endSCoeffs, T);
		// if (PATH_PLANNER_DEBUG) {
		// 	for (auto p: s_coeffs) { cout << p << ", "; }
		// 	cout << endl;
		// }
		vector<double> d_coeffs = JMT(startDCoeffs, endDCoeffs, T);
		// cout << "START and END for d: " << end
		if (isFeasible(s_coeffs, d_coeffs)) {
			possibleSTrajectories.push_back(s_coeffs);
			possibleDTrajectories.push_back(d_coeffs);
			Ts.push_back(T);
		}
	}

	// Push back prev trajectory to be considered among the rest
	// Be careful, we need to translate by the T elapsed
	bool hasPreviousTrajectory = false;
	if (!appendOnly && prevXYAccelSD.size() > 0 && prevPathX.size() > 0) {
		hasPreviousTrajectory = true;
		possibleSTrajectories.push_back(prevSCoeffs);
		possibleDTrajectories.push_back(prevDCoeffs);
	}

	// pick trajectory with lowest cost
	if (PATH_PLANNER_DEBUG) cout << possibleSTrajectories.size() << " feasible trajectories." << endl;
	vector<double> s_coeffs;
	vector<double> d_coeffs;
	double T;
	double minCost = 10e10;
	for (int i = 0; i < possibleSTrajectories.size(); ++i) {
		const double cost =  (i == possibleSTrajectories.size()-1 && hasPreviousTrajectory) ? 
			calculateCost(possibleSTrajectories[i], possibleDTrajectories[i], targetLane, elapsedTime) :
			calculateCost(possibleSTrajectories[i], possibleDTrajectories[i], targetLane, 0);
		if (cost < minCost) {
			minCost = cost;
			s_coeffs = possibleSTrajectories[i];
			d_coeffs = possibleDTrajectories[i];
			T = Ts[i];
		}
	}

	if (PATH_PLANNER_DEBUG) cout << "Min cost " << minCost << endl;
	// Generate chosen trajectory
	vector<double> next_xs, next_ys, next_ss, next_ds, accelSs, accelDs;
	if (appendOnly) {
		next_xs = prevPathX;
		next_ys = prevPathY;
		for (int i = numOfXYPassed; i < prevXYAccelSD.size(); ++i) {
			accelSs.push_back(get<2>(prevXYAccelSD[i]));
			accelDs.push_back(get<3>(prevXYAccelSD[i]));
			next_ss.push_back(get<4>(prevXYAccelSD[i]));
			next_ds.push_back(get<5>(prevXYAccelSD[i]));
		}
	}

	// vector<tuple<double, double, double, double> > xyAccelSD;
	if (s_coeffs.size() > 0) {
		const double tDelta = 0.02;
		// for (auto s: s_coeffs) { cout << s << ","; }
		// cout << endl;
		this->prevSCoeffs = s_coeffs;
		this->prevDCoeffs = d_coeffs;
		for (double i = 0; i < T; i+=tDelta) {
			const double next_s = eval(s_coeffs, i);
			const double next_d = eval(d_coeffs, i);
			const double s_accel = evalA(s_coeffs, i);
			const double d_accel = evalA(d_coeffs, i);
			const vector<double> xy = getXY(next_s , next_d, maps_s, maps_x, maps_y);
			// cout << next_s << ", " << next_d << ", " << xy[0] << ", " << xy[1] << endl;
			next_ss.push_back(next_s);
			next_ds.push_back(next_d);
		  next_xs.push_back(xy[0]);
		  next_ys.push_back(xy[1]);
		  accelSs.push_back(s_accel);
		  accelDs.push_back(d_accel);
		  // if (next_s > distToCarAhead[targetLane] - CAR_S_SAFETY_DISTANCE) { cout << "truncated!" << endl; break; } // truncate
		}
	} else {
		if (PATH_PLANNER_DEBUG) cout << "Use previous trajectory" << endl;
		next_xs = prevPathX;
		next_ys = prevPathY;
	}
	this->prevTimestamp = currTimestamp;
	// this->prevXYAccelSD = xyAccelSD;
	return (next_xs.size() >= 5) ? smoothenPath(next_xs, next_ys, next_ss, next_ds, accelSs, accelDs) : make_pair(next_xs, next_ys);
}

// mostly for lane keeping, targetSpeed and v is in mph
pair<vector<double>, vector<double> > Planner::extendPath(double x, double y, double yaw, double v, double targetSpeed, int targetLane) {
	prevTimestamp = clock_time_ms();
	const int size = prevPathX.size();
	vector<tuple<double, double, double, double, double, double> > xyAccelSD;
	if (prevPathX.size() == 0) return make_pair(prevPathX, prevPathY);
	const double ms = mph2ms(v);
	const double targetSpeedMs = mph2ms(targetSpeed);
	const double displacement = 0.5 * (targetSpeedMs + ms) * PATH_PLANNING_HORIZON;
	vector<double> next_xs, next_ys, next_ss, next_ds, accelS, accelD;
	next_xs = prevPathX;
	next_ys = prevPathY;

	int numOfXYPassed = prevXYAccelSD.size() - size;


	for (int i = numOfXYPassed; i < prevXYAccelSD.size(); ++i) {
		const double accS = get<2>(prevXYAccelSD[i]);
		const double accD = get<3>(prevXYAccelSD[i]);
		const double nextS = get<4>(prevXYAccelSD[i]);
		const double nextD = get<5>(prevXYAccelSD[i]);
		accelS.push_back(accS);
		accelD.push_back(accD);
		next_ss.push_back(nextS);
		next_ds.push_back(nextD);
	}
	
	const int TARGET_NUM_PTS = (int) ceil(PATH_PLANNING_HORIZON/0.02);
	const int ptsToAdd = (int) TARGET_NUM_PTS - size;
	if (PATH_PLANNER_DEBUG) cout << "[PathPlanner]prevPathX size: " << size <<  " numOfXYPassed " << numOfXYPassed << ", ptsToAdd " << ptsToAdd << endl;
	const double deltaS = next_ss[size-1] - next_ss[size-2];
	const double deltaD = next_ds[size-1] - next_ds[size-2];

	if (ptsToAdd > 0) {
		double prevS = next_ss[size-1];
		double prevD = next_ds[size-1];
		for (int i = 0; i < ptsToAdd; ++i) {
			const vector<double> next_xy = getXY(prevS + deltaS, prevD, maps_s, maps_x, maps_y);
			prevS += deltaS;
			prevD += 0;
			// cout << prevS << ", " << prevD << endl;
			next_xs.push_back(next_xy[0]);
			next_ys.push_back(next_xy[1]);
			accelS.push_back(0);
			accelD.push_back(0);
			next_ss.push_back(prevS);
			next_ds.push_back(prevD);
			if ( prevS > distToCarAhead[targetLane] - CAR_S_SAFETY_DISTANCE) { cout << "truncated!" << endl; break; } // truncate
		}
		return smoothenPath(next_xs, next_ys, next_ss, next_ds, accelS, accelD);
	} else {
		return make_pair(next_xs, next_ys);
	}
}

// pathX and pathY must be sorted in ascending X
pair<vector<double>, vector<double> > Planner::smoothenPath(const vector<double>& pathX, const vector<double>& pathY, const vector<double>& pathS, const vector<double>& pathD, const vector<double>& accelS, const vector<double>& accelD) {
	tk::spline xt, yt;
	vector<tuple<double, double, double, double, double, double> > xyAccelSD;
	const int POINTS_TO_FIT = 5;
	assert(pathX.size() >= POINTS_TO_FIT);
	const int START = 0;
	const int END = pathX.size() - 1;
	const int MID = (int) (END - START) / 2;
	const int FIRST_QUARTILE = (int) (MID - START) / 2;
	const int THIRD_QUARTILE = (int) (MID + END) / 2;
	vector<double> smoothedX, smoothedY;
	xt.set_points( { 0.02*START, 0.02*FIRST_QUARTILE, .02*MID, .02*THIRD_QUARTILE, .02*END },
								 { pathX[START], pathX[FIRST_QUARTILE], pathX[MID], pathX[THIRD_QUARTILE], pathX[END] } );

	yt.set_points( { 0.02*START, 0.02*FIRST_QUARTILE, .02*MID, .02*THIRD_QUARTILE, .02*END },
								 { pathY[START], pathY[FIRST_QUARTILE], pathY[MID], pathY[THIRD_QUARTILE], pathY[END] } );

	for (int i = 0; i < pathX.size(); ++i ) {
		const double evalX = xt(.02*i);
		const double evalY = yt(.02*i);
		smoothedX.push_back(evalX);
		smoothedY.push_back(evalY);
		// we need to use the smoothed xy values here
		xyAccelSD.push_back(make_tuple(evalX, evalY, accelS[i], accelD[i], pathS[i], pathD[i]));
	}
	this->prevXYAccelSD = xyAccelSD;
	return make_pair(smoothedX, smoothedY);
}

// evaluate displacement at time t
double Planner::eval(const vector<double>& coeffs, double T) {
	return coeffs[0] + coeffs[1] * T + coeffs[2] * T * T + 
				 coeffs[3] * pow(T, 3) + coeffs[4] * pow(T, 4) + 
				 coeffs[5] * pow(T, 5);
}

// evaluate velocity at time t
double Planner::evalV(const vector<double>& coeffs, double T) {
	return coeffs[1] + 2 * coeffs[2] * T + 3 * coeffs[3] * pow(T,  2) +
				 4 * coeffs[4] * pow(T, 3) + 5 * coeffs[5] * pow(T, 4);
}

// evaluate acceleration at time t
double Planner:: evalA(const vector<double>& coeffs, double T) {
	return 2 * coeffs[2] + 6 * coeffs[3] * T + 12 * coeffs[4] * pow(T, 2) +
				 20 * coeffs[5] * pow(T, 3);
}

// evaluate jerk at time t
double Planner::evalJ(const vector<double>& coeffs, double T) {
	return 6 * coeffs[3] + 24 * coeffs[4] * T + 60 * coeffs[5] * pow(T, 2);
}

vector<tuple<double, double, double> > Planner::generateEndConfigurations(int n, double center_s, double center_d, double T) {
	default_random_engine gen;
	std::normal_distribution<double> dist_s(center_s, SIGMA_S);
	std::normal_distribution<double> dist_d(center_d, SIGMA_D);
	std::normal_distribution<double> dist_t(T, SIGMA_T);
	vector<tuple<double, double, double> > results;
	for (int i = 0; i < n; ++i) {
		results.push_back(make_tuple(dist_s(gen), dist_d(gen), dist_t(gen)));
	}
	return results;
}

bool Planner::isFeasible(const vector<double>& s_coeffs, const vector<double>& d_coeffs) {
	double currentHeading = yaw;
	const double deltaT = 0.02;
	const int NUM_PTS = (int) floor(PATH_PLANNING_HORIZON / deltaT);
	for (int i = 0; i < PATH_PLANNING_HORIZON; ++i) {
		const double ss = eval(s_coeffs, i);
		const double vs = evalV(s_coeffs, i);
		const double as = evalA(s_coeffs, i);
		const double sd = eval(d_coeffs, i);
		const double vd = evalV(d_coeffs, i);
		const double ad = evalA(d_coeffs, i);
		if (sd > 13 || sd < 1) {
			// cout << "infeasible sd " << sd<< endl;
			return false;
		}
		if (fabs(vs) > MAX_VEL || vs < 0) {
			// cout << "infeasible vs " << vs << endl;;
			return false;
		}
		if (fabs(vd) > MAX_VEL) {
			// cout << "infeasible vd " << vd << endl;
			return false;
		}
		if (fabs(as) > MAX_ACCEL || (as < 0 && fabs(as) > MAX_DECEL)) {
			// cout << "infeasible as " << as << endl;
			return false;
		}
		if (fabs(ad) > MAX_ACCEL) {
			// cout << "infeasible ad " << ad << endl;
			return false;
		}

		// TODO: check steering angle
		const double nextSS = eval(s_coeffs, i+1);
		const double nextSD = eval(d_coeffs, i+1);
		const double angleSD = atan2( (nextSD - sd), (nextSS - ss) ); // angle in S-D coordinates
		const vector<double> currXY = getXY(ss, sd, maps_s, maps_x, maps_y);
		const vector<double> nextXY = getXY(nextSS, nextSD, maps_s, maps_x, maps_y);
		const double angleXY = atan2( (nextXY[1] - currXY[1]), (nextXY[0] - currXY[0]) ); // angle in X-Y coordinates
		const double steering_angle = calcAngleDelta(currentHeading, angleXY);
		currentHeading = angleXY;
		if (steering_angle > deg2rad(MAX_STEER_ANGLE) ) {
			// if (PATH_PLANNER_DEBUG) {
			// 	cout << "infeasible steering angle "  
			// 		<< currentHeading << ", " 
			// 		<< steering_angle << ", " 
			// 		<< "(" << currXY[0] << "," << currXY[1] << ")" 
			// 		<< " -> " << "(" << nextXY[0] << "," << nextXY[1] << ")" << endl;
			// }
			return false;
		}

		// Check for collision with other cars
		for (auto trajectory: trajectories) {
			const int j = (int) i * (1.0/0.02);
			if (fabs(trajectory[i].first - ss) < CAR_S_SAFETY_DISTANCE && fabs(trajectory[i].second - sd) < CAR_D_SAFETY_DISTANCE) {
				if (PATH_PLANNER_DEBUG) cout << "[PathPlanner]infeasible - collision in s-d coordinates" << endl;
				return false;
			}
		}
	}

	// final heading should be parallel to the lane
	const double finalS = eval(s_coeffs, PATH_PLANNING_HORIZON);
	const double finalHeading = angleXYtoFrenet(currentHeading, getLaneCurvature(finalS, maps_s, maps_x, maps_y));
	// cout << "final heading " << finalHeading << endl;
	if (finalHeading > 1e-1) {
		if (PATH_PLANNER_DEBUG) {
			// cout << "infeasible final heading " << finalHeading << endl;
		}
		return false;
	}

	return true;
}

double Planner::calculateCost(const vector<double>& s_coeffs, const vector<double>& d_coeffs, int targetLane, double translateT) {
	// Here we assume T = 10 is the end of the trajectory
	double cost = 0;
	const double startS = eval(s_coeffs, translateT + 0);
	const double startD = eval(d_coeffs, translateT + 0);
	const double endS = eval(s_coeffs, translateT + PATH_PLANNING_HORIZON);
	const double endD = eval(d_coeffs, translateT + PATH_PLANNING_HORIZON);

	// closeness to center of the lane
	cost += fabs(LANE_CENTER[targetLane] - endD);

	// safety distance to car in front
	cost += 1;

	// safety distance to car behind
	cost += 1;

	// closeness to previous trajectory - this could be seen as a feasibility
	// difference is discounted by timesteps into the future
	for (int i = 0; i < prevPathX.size(); ++i) {
		const double s = eval(s_coeffs, translateT + i*0.02);
		const double d = eval(d_coeffs, translateT + i*0.02);
		const vector<double> prevSD = getFrenet(prevPathX[i], prevPathY[i], yaw, maps_x, maps_y);
		cost += 10.0 * fabs(s - prevSD[0]) / (i+1 * 1.0);
		cost += 100.0 * fabs(d - prevSD[1]) / (i+1 * 1.0); // heavily penalize changes in d
	}

	// comfort - we only count lateral jerk
	double total_lateral_jerk = 0;
	for (int i = translateT + 0; i <= translateT + PATH_PLANNING_HORIZON; ++i) {
		const double jd = evalJ(d_coeffs, i);
		total_lateral_jerk += jd;
	}
	cost += total_lateral_jerk / (translateT + PATH_PLANNING_HORIZON);
	if (PATH_PLANNER_DEBUG) cout << "cost " << cost << ", ";
	return cost;
}

void Planner::generateTrajectoriesForPredictions(const double T) {
	trajectories.clear();
	const double deltaT = 0.02;
	for (auto p: predictions) {
		// assume speed remains constant
		const double thetaXY = angleFrenetToXY(p[1], p[2], 0, yaw, maps_x, maps_y);
		const vector<double> frenetSpeed = getFrenetSpeed(p[1], p[2], thetaXY, p[3], p[4], maps_x, maps_y);
		vector<pair<double, double> > trajectory;
		for (int i = 0; i < PATH_PLANNING_HORIZON; ++i) {
			const double s = p[5] + i * frenetSpeed[0];
			const double d = p[6] + i * frenetSpeed[1];
			trajectory.push_back(make_pair(s, d));
			// cout << "car_id: " << p[0] << " speed_s: " << frenetSpeed[0] << " speed_d: " << frenetSpeed[1]
			// << " s : " << s << " d: " << d << endl;
		}
		trajectories.push_back(trajectory);
	}
}


vector<double> Planner::calculateDeltaSD(const double theta) {
	const vector<double> initialSD = getFrenet(get<0>(prevXYAccelSD[0]), get<1>(prevXYAccelSD[0]), theta, maps_x, maps_y);
	const vector<double> finalSD = getFrenet(prevPathX[0], prevPathY[0], theta, maps_x, maps_y);
	return { finalSD[0] - initialSD[0], finalSD[1] - initialSD[1] };
}

void Planner::updateState(double x, double y, double s, double d, double yaw, double v) {
	this->x = x;
	this->y = y;
	this->s = s;
	this->d = d;
	this->yaw = yaw;
	this->v = v;
}

void Planner::updatePredictions(const vector<vector<double> >& predictions) {
	this->predictions = predictions;
	distToCarAhead[1] = INF;
	distToCarAhead[2] = INF;
	distToCarAhead[3] = INF;
	for (auto p: predictions) {
		int laneNum = calcCurrentLane(p[6]);
		if (p[5] > s && p[5] < distToCarAhead[laneNum]) {
			distToCarAhead[laneNum] = p[5];
		}
	}
	if (PATH_PLANNER_DEBUG) {
		// cout << "[PathPlanner] Dist to car ahead Lane 1: " << distToCarAhead[1] - s << endl;
		// cout << "[PathPlanner] Dist to car ahead Lane 2: " << distToCarAhead[2] - s << endl;
		// cout << "[PathPlanner] Dist to car ahead Lane 3: " << distToCarAhead[3] - s << endl;
	}
}

// speed - scalar, ms-1
double Planner::getSpeedAtEndOfPath() {
	if (prevPathX.size() > 1) {
		const double deltaX = prevPathX[prevPathX.size()-1] - prevPathX[prevPathX.size()-2];
		const double deltaY = prevPathY[prevPathY.size()-1] - prevPathY[prevPathY.size()-2];
		return sqrt(deltaX * deltaX + deltaY * deltaY) / 0.02;
	} else {
		return 0.0;
	}
}

double Planner::getYawAtEndOfPath() {
	if (prevPathX.size() > 1) {
		const double deltaX = prevPathX[prevPathX.size()-1] - prevPathX[prevPathX.size()-2];
		const double deltaY = prevPathY[prevPathY.size()-1] - prevPathY[prevPathY.size()-2];
		return atan2(deltaY, deltaX);
	} else {
		return yaw;
	}
}