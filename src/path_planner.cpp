#include <vector>
#include <iostream>
#include <random>
#include <tuple>
#include <map>
#include <thread>
#include <chrono>
#include <numeric>
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
	this->hasTrajectoryBefore = false;
	this->numOfXYPassed = 0;
	this->distTravelled = 0;
}

Planner::~Planner() {}

// Add in prev values for subsequent calculations, can only call this after first state is given
void Planner::init(const double x, const double y, const double s, const double d, const double yaw, const double v) {
	updateState(x, y, s, d, yaw, v);
	// this->prevPathX = { x };
	// this->prevPathY = { y };
	// this->prevXYAccelSD = vector<tuple<double, double, double, double, double, double> > { make_tuple(x, y, 0, 0, s, d) };
	this->hasTrajectoryBefore = false;
}

void Planner::updatePrevPaths(const vector<double>& prevX, const vector<double>& prevY) {
	this->prevPathX = prevX;
	this->prevPathY = prevY;
}

// Must call this after updatePrevPaths
void Planner::updateState(double x, double y, double s, double d, double yaw, double v) {
	this->x = x;
	this->y = y;
	this->s = s;
	this->d = d;
	this->yaw = yaw;
	this->v = v;
	this->currentLane = calcCurrentLane(d);

	if (hasReachedEndOfTrajectory()) {
		endOfCurrentTrajectory.clear();
	}

	numOfXYPassed = prevXYAccelSD.size() - prevPathX.size();
	if (prevXYAccelSD.size() > 0) {
		const double finalX = get<0>(prevXYAccelSD[numOfXYPassed]);
		const double finalY = get<1>(prevXYAccelSD[numOfXYPassed]);
		const double initialX = get<0>(prevXYAccelSD[0]);
		const double initialY = get<1>(prevXYAccelSD[1]);

		distTravelled = sqrt((finalX-initialX)*(finalX-initialX) + (finalY-initialY)*(finalY-initialY));
	}

	generateTrajectoriesForPredictions(PATH_PLANNING_HORIZON);
	isOnCollisionCourse = onCollisionCourse();
	if (isOnCollisionCourse) cout << "[PathPlanner] ON COLLISION COURSE!" << endl;
}

void Planner::updatePredictions(const vector<vector<double> >& predictions) {
	this->predictions = predictions;
	distToCarAhead[1] = INF;
	distToCarAhead[2] = INF;
	distToCarAhead[3] = INF;
	distToCarBehind[1] = INF;
	distToCarBehind[2] = INF;
	distToCarBehind[3] = INF;
	speedOfApproachFromBehind[1] = 0;
	speedOfApproachFromBehind[2] = 0;
	speedOfApproachFromBehind[3] = 0;
	speedOfApproachFromFront[1] = INF;
	speedOfApproachFromFront[2] = INF;
	speedOfApproachFromFront[3] = INF;
	// TODO : logic for determining in front and behind does not work for wrap around case
	// e.g.  ego s = 0, cars behind will have s = 6800 > s = 0
	for (auto p: predictions) {
		int laneNum = calcCurrentLane(p[6]);
		if (p[5] > s && p[5] < distToCarAhead[laneNum]) {
			distToCarAhead[laneNum] = p[5];
		}
		if (p[5] < s && p[5] > distToCarBehind[laneNum]) {
			distToCarBehind[laneNum] = p[5];
		}
		const double carV = sqrt(p[3]*p[3] + p[4]*p[4]);
		if ( p[5] > s && carV < speedOfApproachFromFront[laneNum]) {
			speedOfApproachFromFront[laneNum] = carV;
		}
		if ( p[5] < s && carV > speedOfApproachFromBehind[laneNum]) {
			speedOfApproachFromBehind[laneNum] = carV;
		}
	}
	if (distToCarAhead[1] != INF) distToCarAhead[1] -= s;
	if (distToCarAhead[2] != INF) distToCarAhead[2] -= s;
	if (distToCarAhead[3] != INF) distToCarAhead[3] -= s;
	if (distToCarBehind[1] != INF) distToCarBehind[1] = s - distToCarBehind[1];
	if (distToCarBehind[2] != INF) distToCarBehind[2] = s - distToCarBehind[2];
	if (distToCarBehind[3] != INF) distToCarBehind[3] = s - distToCarBehind[3];
	if (speedOfApproachFromBehind[1] != 0) speedOfApproachFromBehind[1] -= mph2ms(v);
	if (speedOfApproachFromBehind[2] != 0) speedOfApproachFromBehind[2] -= mph2ms(v);
	if (speedOfApproachFromBehind[3] != 0) speedOfApproachFromBehind[3] -= mph2ms(v);
	if (PATH_PLANNER_DEBUG) {
		cout << "[PathPlanner] Dist to car front, back Lane 1: " << distToCarAhead[1] << ", " << distToCarBehind[1] << endl;
		cout << "[PathPlanner] Dist to car front, back Lane 2: " << distToCarAhead[2] << ", " << distToCarBehind[2] << endl;
		cout << "[PathPlanner] Dist to car front, back Lane 3: " << distToCarAhead[3] << ", " << distToCarBehind[3] << endl;
		cout << "[PathPlanner] speedOfApproach front, back Lane 1: " << speedOfApproachFromFront[1] << ", " << speedOfApproachFromBehind[1] << endl;
		cout << "[PathPlanner] speedOfApproach front, back Lane 2: " << speedOfApproachFromFront[2] << ", " << speedOfApproachFromBehind[2] << endl;
		cout << "[PathPlanner] speedOfApproach front, back Lane 3: " << speedOfApproachFromFront[3] << ", " << speedOfApproachFromBehind[3] << endl;
	}
}

bool Planner::hasReachedEndOfTrajectory() {
	if (!hasTrajectoryBefore) return false;
	if (endOfCurrentTrajectory.size() == 0) return true;
	return s > endOfCurrentTrajectory[0][0] ? true : false;
}

bool Planner::hasTrajectory() {
	return (endOfCurrentTrajectory.size() > 0);
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

// v, targetSpeed is velocity in mph, appendIdx is idx of prevXYAccelSD to generate new trajectory from
tuple<bool, vector<double>, vector<double> > Planner::generatePath(double targetSpeed, int targetLane, int givenAppendIdx, double timeHorizon) {
	if (timeHorizon < 0 ) timeHorizon = PATH_PLANNING_HORIZON;
	unsigned long long currTimestamp = clock_time_ms();
	bool hasPath = false;
	const double elapsedTime = (currTimestamp - prevTimestamp) / 1000.0;
	const double targetSpeedMs = mph2ms(targetSpeed);
	const double v_ms = mph2ms(v);
	const int size = prevXYAccelSD.size();
	vector<int> appendIndices;
	if (givenAppendIdx != -1) {
		appendIndices = { numOfXYPassed + givenAppendIdx };
	} else {
		if (size == 0) appendIndices = { 0 };
		else appendIndices = { size/5-1, 2*size/5-1, 3*size/5-1, 4*size/5-1, size-1 };
	}

	double start_x, start_y, start_s, start_d, start_yaw, start_speed, start_accel_s, start_accel_d;
	vector<double> next_xs, next_ys, next_ss, next_ds, accelSs, accelDs;
	for (int appendIdx : appendIndices) {
		if (numOfXYPassed > appendIdx) continue;
		if (PATH_PLANNER_DEBUG) cout << "[PathPlanner] appendIdx : " << appendIdx << endl;
		// Specify initial conditions
		start_accel_s = (size == 0) ? 0.0 : get<2>(prevXYAccelSD[appendIdx]);
		start_accel_d = (size == 0) ? 0.0 : get<3>(prevXYAccelSD[appendIdx]);
		start_x = (size == 0) ? x : get<0>(prevXYAccelSD[appendIdx]);
		start_y = (size == 0) ? y : get<1>(prevXYAccelSD[appendIdx]);
		start_s = (size == 0) ? s : get<4>(prevXYAccelSD[appendIdx]);
		start_d = (size == 0) ? d : get<5>(prevXYAccelSD[appendIdx]);
		start_yaw = (size == 0) ? yaw : getYawAtPath(appendIdx);
		start_speed = (size == 0) ? v_ms : getSpeedAtPath(appendIdx);
		
		// work in frenet coordinates
		const vector<double> start_speed_sd = getFrenetSpeed(start_x, start_y, start_yaw, start_speed*cos(start_yaw), start_speed*sin(start_yaw), maps_x, maps_y);

		// s = ut + 1/2 * a * t^2, approx. assuming constant acceleration
		const double ave_accel = min(9.0, (targetSpeedMs - start_speed) / timeHorizon);
		const double displacement = start_speed_sd[0] * timeHorizon + 0.5 * ave_accel * timeHorizon * timeHorizon;

		cout << "[PathPlanner] " << s << ", start_s : " << start_s << " , end_s " << start_s + displacement << " , start_speed "  << start_speed << endl;
		// Generate a bunch of possible end configurations
		vector<vector<double> > possibleSTrajectories;
		vector<vector<double> > possibleDTrajectories;
		vector<double> Ts;
		const vector<tuple<double, double, double> > endConfig = generateEndConfigurations(SAMPLE_SIZE, start_s + displacement, displacement/10, LANE_CENTER[targetLane], -1, timeHorizon, timeHorizon/5);
		
		if (PATH_PLANNER_DEBUG) cout << "[PathPlanner] Time elapsed: " 
			<< (currTimestamp - this->prevTimestamp) << " ms, start_accel_s " 
			<< start_accel_s << ", start_accel_d " << start_accel_d 
			<< " , num of pts passed " << numOfXYPassed <<  endl;

		for (auto p : endConfig) {
			const double startSDot = start_speed_sd[0];
			const double startDDot = start_speed_sd[1];
			const double endS = get<0>(p);
			const double endD = get<1>(p);
			const double T = get<2>(p);
			// cout << "Start S: " << start_s << ", " << startSDot << endl;
			// cout << "End S: " << endS << endl;
			// cout << "T is " << T << ", displacement " << displacement << endl;
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
			if (targetLane != currentLane 
				&& speedOfApproachFromBehind[targetLane] != 0 
				&& distToCarBehind[targetLane] / speedOfApproachFromBehind[targetLane] > LANE_SWITCH_TIME) {
				if (isFeasible(appendIdx, timeHorizon, s_coeffs, d_coeffs, targetLane)) {
					possibleSTrajectories.push_back(s_coeffs);
					possibleDTrajectories.push_back(d_coeffs);
					Ts.push_back(T);
				}
			} else {
				if (isFeasible(appendIdx, timeHorizon, s_coeffs, d_coeffs, targetLane)) {
					possibleSTrajectories.push_back(s_coeffs);
					possibleDTrajectories.push_back(d_coeffs);
					Ts.push_back(T);
				}
			}
		}
		
		// pick trajectory with lowest cost
		if (PATH_PLANNER_DEBUG) cout  << "[PathPlanner] " << possibleSTrajectories.size() << " feasible trajectories." << endl;
		vector<double> s_coeffs;
		vector<double> d_coeffs;
		double T;
		double minCost = 10e10;
		for (int i = 0; i < possibleSTrajectories.size(); ++i) {
			const double cost = calculateCost(possibleSTrajectories[i], possibleDTrajectories[i], targetLane, timeHorizon);
			if (cost < minCost) {
				minCost = cost;
				s_coeffs = possibleSTrajectories[i];
				d_coeffs = possibleDTrajectories[i];
				T = Ts[i];
			}
		}
		
		if (PATH_PLANNER_DEBUG) cout << endl << "[PathPlanner] Min cost " << minCost << endl;
		
		// Generate chosen trajectory
		if (s_coeffs.size() > 0) {
			const double tDelta = 0.02;
			next_xs = vector<double>(prevPathX.begin(), prevPathX.begin()+appendIdx);
			next_ys = vector<double>(prevPathY.begin(), prevPathY.begin()+appendIdx);
			for (int i = numOfXYPassed; i < appendIdx + numOfXYPassed; ++i) {
				accelSs.push_back(get<2>(prevXYAccelSD[i]));
				accelDs.push_back(get<3>(prevXYAccelSD[i]));
				next_ss.push_back(get<4>(prevXYAccelSD[i]));
				next_ds.push_back(get<5>(prevXYAccelSD[i]));
			}
			// for (auto s: s_coeffs) { cout << s << ","; }
			// cout << endl;
			this->prevSCoeffs = s_coeffs;
			this->prevDCoeffs = d_coeffs;
			for (double i = 0; i < T; i+=tDelta) {
				const double next_s = fmod(eval(s_coeffs, i), MAX_S);
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
			}
			// Save sd-coordinates of end of generated trajectory
			endOfCurrentTrajectory.clear();
			endOfCurrentTrajectory.push_back({ next_ss[next_ss.size()-1], next_ds[next_ds.size()-1] });
			if (PATH_PLANNER_DEBUG) cout << "[PathPlanner] End of current trajectory is: (" << next_ss[next_ss.size()-1] << ", " << next_ds[next_ds.size()-1] << ")" << endl;
			hasPath = true;
			hasTrajectoryBefore = true;
			break;
		}
	}
	
	this->prevTimestamp = currTimestamp;
	if (PATH_PLANNER_DEBUG) cout << "[PathPlanner] Time taken : " << (clock_time_ms() - currTimestamp) << " ms" << endl;
	
	if (hasPath) {
		if (next_xs.size() >= 5) {
			pair<vector<double>, vector<double> > smoothed = smoothenPath(0, next_xs, next_ys, next_ss, next_ds, accelSs, accelDs);
			return make_tuple(hasPath, smoothed.first, smoothed.second);
		} else {
			return make_tuple(hasPath, next_xs, next_ys);
		}
	} else {
		// keep previous trajectory
		if (PATH_PLANNER_DEBUG) cout << "[PathPlanner] No trajectory found. Use previous trajectory" << endl;
		next_xs = prevPathX;
		next_ys = prevPathY;
		if (size > 0) {
			this->prevXYAccelSD = vector<tuple<double, double, double, double, double, double> >(prevXYAccelSD.begin()+numOfXYPassed, prevXYAccelSD.end());
		} else {
			this->prevXYAccelSD.clear();
		}
		return make_tuple(hasPath, next_xs, next_ys);
	}
}

// mostly for lane keeping, targetSpeed and v is in mph
pair<vector<double>, vector<double> > Planner::extendPath(double targetSpeed, int targetLane) {
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
	if (numOfXYPassed < 0) numOfXYPassed = 0;

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
	assert(size == next_ss.size());
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
			if ( s + distToCarAhead[targetLane] - prevS < CAR_S_COLLISION_DISTANCE) break;
		}
		return smoothenPath(0, next_xs, next_ys, next_ss, next_ds, accelS, accelD);
	} else {
		return make_pair(next_xs, next_ys);
	}
}

// pathX and pathY must be sorted in ascending X
pair<vector<double>, vector<double> > Planner::smoothenPath(
	int startIdx, 
	const vector<double>& pathX, 
	const vector<double>& pathY, 
	const vector<double>& pathS, 
	const vector<double>& pathD, 
	const vector<double>& accelS, 
	const vector<double>& accelD) 
{
	vector<tuple<double, double, double, double, double, double> > xyAccelSD;
	const int POINTS_TO_FIT = 7;
	if (pathX.size() >= (startIdx + POINTS_TO_FIT)) {
		tk::spline xt, yt;
		const int START = startIdx + 0;
		const int END = pathX.size() - 1;
		const int MID = (int) (END + START) / 2;
		const int FIRST_QUARTILE = (int) (MID + START) / 2;
		const int THIRD_QUARTILE = (int) (MID + END) / 2;
		vector<double> smoothedX, smoothedY;
		xt.set_points( { 0.02*START, 0.02*(START+1), 0.02*(START+2), 0.02*FIRST_QUARTILE, .02*MID, .02*THIRD_QUARTILE, .02*END },
									 { pathX[START], pathX[START+1], pathX[START+2], pathX[FIRST_QUARTILE], pathX[MID], pathX[THIRD_QUARTILE], pathX[END] } );

		yt.set_points( { 0.02*START, 0.02*(START+1), 0.02*(START+2), 0.02*FIRST_QUARTILE, .02*MID, .02*THIRD_QUARTILE, .02*END },
									 { pathY[START], pathY[START+1], pathY[START+2], pathY[FIRST_QUARTILE], pathY[MID], pathY[THIRD_QUARTILE], pathY[END] } );

		for (int i = 0; i < pathX.size(); ++i ) {
			if (i < startIdx) {
				smoothedX.push_back(pathX[i]);
				smoothedY.push_back(pathY[i]);
				xyAccelSD.push_back(make_tuple(pathX[i], pathY[i], accelS[i], accelD[i], pathS[i], pathD[i]));
			} else {
				const double evalX = xt(.02*i);
				const double evalY = yt(.02*i);
				smoothedX.push_back(evalX);
				smoothedY.push_back(evalY);
				// we need to use the smoothed xy values here
				xyAccelSD.push_back(make_tuple(evalX, evalY, accelS[i], accelD[i], pathS[i], pathD[i]));
			}
		}
		this->prevXYAccelSD = xyAccelSD;
		return make_pair(smoothedX, smoothedY);
	} else {
		assert(pathX.size() == pathS.size());
		for (int i = 0; i < pathX.size(); ++i) {
			xyAccelSD.push_back(make_tuple(pathX[i], pathY[i], accelS[i], accelD[i], pathS[i], pathD[i]));
		}
		this->prevXYAccelSD = xyAccelSD;
		return make_pair(pathX, pathY);
	}
}

vector<tuple<double, double, double> > Planner::generateEndConfigurations(int n, double center_s, double sigma_s, double center_d, double sigma_d, double T, double sigma_t) {
	if (sigma_s < 0) sigma_s = SIGMA_S;
	if (sigma_d < 0) sigma_d = SIGMA_D;
	if (sigma_t < 0) sigma_t = SIGMA_T;
	default_random_engine gen;
	std::normal_distribution<double> dist_s(center_s, sigma_s);
	std::normal_distribution<double> dist_d(center_d, sigma_d);
	std::normal_distribution<double> dist_t(T, sigma_t);
	vector<tuple<double, double, double> > results;
	for (int i = 0; i < n; ++i) {
		results.push_back(make_tuple(dist_s(gen), dist_d(gen), dist_t(gen)));
	}
	return results;
}

// startIdx - idx where trajectory to be evaluated will be joined to previous generated trajectory, prevXYAccelSD
bool Planner::isFeasible(int startIdx, double timeHorizon, const vector<double>& s_coeffs, const vector<double>& d_coeffs, int targetLane) {
	if (timeHorizon < 0) timeHorizon = PATH_PLANNING_HORIZON;
	// calculate heading at start of trajectory
	double currentHeading;
	if (prevXYAccelSD.size() != 0) {
		const vector<double> startXY = getXY(get<4>(prevXYAccelSD[startIdx-1]), get<5>(prevXYAccelSD[startIdx-1]), maps_s, maps_x, maps_y);
		const vector<double> nextXY = getXY(get<4>(prevXYAccelSD[startIdx]), get<5>(prevXYAccelSD[startIdx]), maps_s, maps_x, maps_y); 
		currentHeading = atan2( (nextXY[1] - startXY[1]), (nextXY[0] - startXY[0]) ); // angle in X-Y coordinates
	} else {
		currentHeading = yaw;
	}

	// check for safety following distance at end of trajectory
	if (speedOfApproachFromFront[targetLane] * timeHorizon - eval(s_coeffs, timeHorizon) < CAR_S_SAFETY_DISTANCE) {
		cout << "[PathPlanner] infeasible end s position too near car ahead " << eval(s_coeffs, timeHorizon) << endl;
		return false;
	}
	
	const double deltaT = 0.20; // Adjust this value to tradeoff speed vs accuracy
	const int NUM_PTS = (int) floor(timeHorizon / deltaT);
	for (double i = 0; i <= timeHorizon; i+=deltaT) {
		const double ss = eval(s_coeffs, i);
		const double vs = evalV(s_coeffs, i);
		const double as = evalA(s_coeffs, i);
		const double sd = eval(d_coeffs, i);
		const double vd = evalV(d_coeffs, i);
		const double ad = evalA(d_coeffs, i);
		if (sd > 11.0 || sd < 1.0) {
			cout << "[PathPlanner] infeasible sd " << sd << endl;
			return false;
		}
		if (fabs(sqrt(vs*vs + vd*vd)) > MAX_VEL) {
			cout << "[PathPlanner] infeasible velocity " << fabs(sqrt(vs*vs + vd*vd)) << endl;
			return false;
		}
		if (vs < 0) {
			cout << "[PathPlanner] infeasible vs < 0 " << vs << endl;
			return false;
		}
		if (fabs(as) > MAX_ACCEL || (as < 0 && fabs(as) > MAX_S_DECEL)) {
			cout << "[PathPlanner] infeasible accT - as " << as << endl;
			return false;
		}
		if (fabs(ad) > MAX_ACCEL) {
			cout << "[PathPlanner] infeasible accN - ad " << ad << endl;
			return false;
		}
		if (ad < 0 && fabs(ad) > MAX_D_DECEL) {
			cout << "[PathPlanner] infeasible accN deceleration - ad " << ad << endl;
		}
		if (fabs(sqrt(as*as + ad*ad)) > MAX_ACCEL) {
			cout << "[PathPlanner] infeasible accTotal - " << fabs(sqrt(as*as + ad*ad)) << endl;
			return false;
		}

		// TODO: check steering angle
		const double nextSS = eval(s_coeffs, i+deltaT);
		const double nextSD = eval(d_coeffs, i+deltaT);
		const double angleSD = atan2( (nextSD - sd), (nextSS - ss) ); // angle in S-D coordinates
		const vector<double> currXY = getXY(ss, sd, maps_s, maps_x, maps_y);
		const vector<double> nextXY = getXY(nextSS, nextSD, maps_s, maps_x, maps_y);
		const double angleXY = atan2( (nextXY[1] - currXY[1]), (nextXY[0] - currXY[0]) ); // angle in X-Y coordinates
		const double steering_angle = calcAngleDelta(currentHeading, angleXY);
		currentHeading = angleXY;
		if (steering_angle > deg2rad(MAX_STEER_ANGLE) ) {
			// if (PATH_PLANNER_DEBUG) {
			// 	cout << "[PathPlanner] " << "infeasible steering angle "  
			// 		<< currentHeading << ", " 
			// 		<< steering_angle << ", " 
			// 		<< "(" << currXY[0] << "," << currXY[1] << ")" 
			// 		<< " -> " << "(" << nextXY[0] << "," << nextXY[1] << ")" << endl;
			// }
			return false;
		}

		// Check for collision with other cars
		for (auto trajectory: trajectories) {
			const int j = (int) i * (deltaT/0.02) + startIdx;
			if (j < trajectory.size() && fabs(trajectory[j].first - ss) < CAR_S_COLLISION_DISTANCE && fabs(trajectory[j].second - sd) < CAR_D_COLLISION_DISTANCE) {
				if (PATH_PLANNER_DEBUG) 
					cout << "[PathPlanner] infeasible - collision at ("
						 << ss << "," << sd  << ") coordinates" << endl;
				return false;
			}
		}
	}

	// final heading should be parallel to the lane
	const double finalS = eval(s_coeffs, timeHorizon);
	const double finalHeading = angleXYtoFrenet(currentHeading, getLaneCurvature(finalS, maps_s, maps_x, maps_y));
	// cout << "[PathPlanner] final heading " << finalHeading << endl;
	if (finalHeading > 1e-1) {
		if (PATH_PLANNER_DEBUG) {
			cout << "[PathPlanner] infeasible final heading " << finalHeading << ", currentHeading " << currentHeading << ", current yaw " << yaw << endl;
		}
		return false;
	}

	return true;
}

double Planner::calculateCost(const vector<double>& s_coeffs, const vector<double>& d_coeffs, int targetLane, double timeHorizon) {
	if (timeHorizon < 0) timeHorizon = PATH_PLANNING_HORIZON;
	double cost = 0;
	const double deltaT = 0.20;
	const double startS = eval(s_coeffs, 0);
	const double startD = eval(d_coeffs, 0);
	const double endS = eval(s_coeffs, timeHorizon);
	const double endD = eval(d_coeffs, timeHorizon);
	vector<double> acc;
	const int N = (int) ceil(timeHorizon / deltaT);
	// closeness to center of the lane
	cost += fabs(LANE_CENTER[targetLane] - endD);

	// safety distance to car in front
	acc.clear();
	for (double i = 0; i <= timeHorizon; i+=deltaT) {
		const double s = eval(s_coeffs, i);
		const double d = eval(d_coeffs, i);
		const double carS = s + distToCarAhead[targetLane];
		const double carD = LANE_CENTER[targetLane];
		acc.push_back(distance(s, d, carS, carD));
	}
	cost += fmin(1.0, CAR_S_SAFETY_DISTANCE / normalizeValues(acc));

	// safety distance to car behind
	acc.clear();
	for (double i = 0; i <= timeHorizon; i+=deltaT) {
		const double s = eval(s_coeffs, i);
		const double d = eval(d_coeffs, i);
		const double carS = s - distToCarBehind[targetLane];
		const double carD = LANE_CENTER[targetLane];
		acc.push_back(distance(s, d, carS, carD));
	}
	cost += fmin(1.0, CAR_S_SAFETY_DISTANCE / normalizeValues(acc));

	// Time spent in between lanes
	acc.clear();
	for (double i = 0; i <= timeHorizon; i+=deltaT) {
		const double d = eval(d_coeffs, i);
		int lane = calcCurrentLane(d);
		acc.push_back(fabs(d - LANE_CENTER[lane]));
	}
	cost += normalizeValues(acc);

	// No wild swings in velocity
	acc.clear();
	for (double i = 0; i <= timeHorizon; i+=deltaT) {
		acc.push_back(evalV(s_coeffs, i));
	}
	cost += normalizedVariance(acc);

	// comfort - we only count lateral jerk
	acc.clear();
	for (double i = 0; i <= timeHorizon; i+=deltaT) {
		const double jd = evalJ(d_coeffs, i);
		acc.push_back(jd);
	}
	cost += normalizeValues(acc);
	if (PATH_PLANNER_DEBUG) cout << "cost " << cost << ", ";
	return cost;
}

void Planner::generateTrajectoriesForPredictions(double timeHorizon) {
	if (timeHorizon < 0) timeHorizon = PATH_PLANNING_HORIZON; 
	trajectories.clear();
	const double deltaT = 0.02;
	for (auto p: predictions) {
		// assume speed remains constant
		const double thetaXY = angleFrenetToXY(p[1], p[2], 0, yaw, maps_x, maps_y);
		const vector<double> frenetSpeed = getFrenetSpeed(p[1], p[2], thetaXY, p[3], p[4], maps_x, maps_y);
		vector<pair<double, double> > trajectory;
		for (double i = 0; i < timeHorizon; i+=deltaT) {
			const double s = p[5] + i * frenetSpeed[0];
			const double d = p[6] + i * frenetSpeed[1];
			trajectory.push_back(make_pair(s, d));
			// cout << "[PathPlanner] car_id: " << p[0] << " speed_s: " << frenetSpeed[0] << " speed_d: " << frenetSpeed[1]
			// << " s : " << s << " d: " << d << endl;
		}
		trajectories.push_back(trajectory);
	}
}

// run this after generating trajectories
bool Planner::onCollisionCourse() {
	double egoS, egoD;
	for (auto t: trajectories) {
		if (prevXYAccelSD.size() == 0) {
			egoS = s;
			egoD = d;
			for (int i = 0; i < trajectories[i].size(); i++) {
				if (fabs(t[i].first - egoS) < CAR_S_COLLISION_DISTANCE && fabs(t[i].second - egoD) < CAR_D_COLLISION_DISTANCE) {
					return true;
				}
			}
		} else {
			for (int i = 0; i < prevXYAccelSD.size(); ++i) {
				egoS = get<4>(prevXYAccelSD[i]);
				egoD = get<5>(prevXYAccelSD[i]);
				// cout << "(" << fabs(t[i].first - egoS) << ", " << fabs(t[i].second - egoD) << ") ";
				if (fabs(t[i].first - egoS) < CAR_S_COLLISION_DISTANCE && fabs(t[i].second - egoD) < CAR_D_COLLISION_DISTANCE) {
					return true;
				}
			}
			// cout << endl;
		}
	}
	return false;
}

// speed - scalar, ms-1
double Planner::getSpeedAtPath(int idx) {
	if (prevPathX.size() > 1) {
		const double deltaX = prevPathX[idx] - prevPathX[idx-1];
		const double deltaY = prevPathY[idx] - prevPathY[idx-1];
		return sqrt(deltaX * deltaX + deltaY * deltaY) / 0.02;
	} else {
		return 0.0;
	}
}

double Planner::getYawAtPath(int idx) {
	if (prevPathX.size() > 1) {
		const double deltaX = prevPathX[idx] - prevPathX[idx-1];
		const double deltaY = prevPathY[idx] - prevPathY[idx-1];
		return atan2(deltaY, deltaX);
	} else {
		return yaw;
	}
}

// normalize all values between 0 and 1 and returns its average
double Planner::normalizeValues(vector<double>& values) {
	assert(values.size() > 0);
	const double minV = *min_element(values.begin(), values.end());
	const double maxV = *max_element(values.begin(), values.end());
	double sum = 0.0;
	if (fabs(minV - maxV) > 1e-3) {
		for (int i = 0; i < values.size(); ++i) {
			values[i] = (values[i] - minV) / (maxV - minV);
		}
		sum = accumulate(values.begin(), values.end(), 0.0);
	} else {
		sum = accumulate(values.begin(), values.end(), 0.0);
	}
	return sum / values.size();
}

double Planner::normalizedVariance(vector<double>& values) {
	assert(values.size() > 0);
	const double minV = *min_element(values.begin(), values.end());
	const double maxV = *max_element(values.begin(), values.end());
	double sum = 0.0;
	if (fabs(minV - maxV) > 1e-3) {
		for (int i = 0; i < values.size(); ++i) {
			values[i] = (values[i] - minV) / (maxV - minV);
		}
		sum = accumulate(values.begin(), values.end(), 0.0);
	} else {
		sum = accumulate(values.begin(), values.end(), 0.0);
	}
	double mean = sum / values.size();
	double varV = 0.0;
	for (int i = 0; i < values.size(); ++i) {
		varV += fabs(values[i] - mean);
	}
	return varV / values.size();
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