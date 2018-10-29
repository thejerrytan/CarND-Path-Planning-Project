#include <vector>
#include <iostream>
#include <random>
#include <tuple>
#include <map>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include "path_planner.hpp"
#include "fsm.hpp"
#include "utils.hpp"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Planner::Planner(const vector<double>& maps_x, const vector<double>& maps_y, const vector<double>& maps_s) {
	this->maps_x = maps_x;
	this->maps_y = maps_y;
	this->maps_s = maps_s;

	// this->nextAccelS = 0;
	// this->nextAccelD = 0;
	this->prevTimestamp = clock_time_ms();
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

// v, targetSpeed is velocity in mph
pair<vector<double>, vector<double> > Planner::generatePath(double x, double y, double s, double d, double yaw, double v, double targetSpeed, int targetLane) {
	targetLane = 2;
	unsigned long long currTimestamp = clock_time_ms();
	const double ms = mph2ms(targetSpeed);
	const double v_ms = mph2ms(v);

	// work in frenet coordinates
	const vector<double> frenetSpeed = getFrenetSpeed(x, y, yaw, v_ms*cos(yaw), v_ms*sin(yaw), maps_x, maps_y);

	// s = ut + 1/2 * a * t^2, assuming linear kinematics, we apply a correction factor, if not no solution exists
	const double displacement = (frenetSpeed[0] * PATH_PLANNING_HORIZON +
			 												0.5 * TARGET_AVG_ACCEL * PATH_PLANNING_HORIZON * PATH_PLANNING_HORIZON) 
															* NONLINEARITY_CORRECTION_FACTOR;
	
	// Generate a bunch of possible end configurations
	const vector<tuple<double, double, double> > endConfig = generateEndConfigurations(SAMPLE_SIZE, s + displacement, FSM::LANE_CENTER[targetLane], PATH_PLANNING_HORIZON);
	vector<vector<double> > possibleSTrajectories;
	vector<vector<double> > possibleDTrajectories;
	double currAccelS, currAccelD;
	int numOfXYPassed = 0;
	if (this->prevXYAccelSD.size() > 0) {
		for (int i = 0; i < prevXYAccelSD.size(); ++i) {
			if (i == prevXYAccelSD.size()) cout << "Cannot find accelSD" << endl;
			if (distance(x, y, get<0>(prevXYAccelSD[i]), get<1>(prevXYAccelSD[i])) < 1e-3) {
				currAccelS = get<2>(prevXYAccelSD[i]);
				currAccelD = get<3>(prevXYAccelSD[i]);
				numOfXYPassed = i + 1;
				break;
			}
		}
	} else {
		currAccelS = 0.0;
		currAccelD = 0.0;
	}
	cout << "Time elapsed: " << (currTimestamp - this->prevTimestamp) << " ms, currAccelS " 
				<< currAccelS << ", currAccelD " << currAccelD 
				<< " , num of pts passed " << numOfXYPassed <<  endl;

	for (auto p : endConfig) {
		const double startS = s;
		const double startSDot = frenetSpeed[0];
		const double startSDotDot = currAccelS;
		const double startD = d;
		const double startDDot = frenetSpeed[1];
		const double startDDotDot = currAccelD;
		const double endS = get<0>(p);
		const double endSDot = ms;
		const double endSDotDot = 0;
		const double endD = get<1>(p);
		const double endDDot = 0;
		const double endDDotDot = 0;
		const double T = get<2>(p);
		// cout << "Start S: " << startS << ", " << startSDot << ", " << startSDotDot << endl;
		// cout << "End S: " << endS << ", " << endSDot << ", " << endSDotDot << endl;
		// cout << "T is " << T << ", displacement " << displacement << endl;
		const vector<double> startSCoeffs = { startS, startSDot, startSDotDot };
		const vector<double> endSCoeffs = { endS, endSDot, endSDotDot };
		const vector<double> startDCoeffs = { startD, startDDot, startDDotDot };
		const vector<double> endDCoeffs = { endD, endDDot, endDDotDot };

		vector<double> s_coeffs = JMT(startSCoeffs, endSCoeffs, T);
		// for (auto p: s_coeffs) { cout << p << ", "; }
		// cout << endl;
		vector<double> d_coeffs = JMT(startDCoeffs, endDCoeffs, T);
		// cout << "START and END for d: " << end
		if (isFeasible(s_coeffs, d_coeffs)) {
			possibleSTrajectories.push_back(s_coeffs);
			possibleDTrajectories.push_back(d_coeffs);
		}
	}

	// Push back prev trajectory to be considered among the rest
	// Be careful, we need to translate by the S,D travelled
	if (prevXYAccelSD.size() > 0 && prevPathX.size() > 0) {
		const vector<double> deltaSD = calculateDeltaSD(yaw);
		prevSCoeffs[0] += deltaSD[0];
		prevDCoeffs[0] += deltaSD[1];
		possibleSTrajectories.push_back(prevSCoeffs);
		possibleDTrajectories.push_back(prevDCoeffs);
	}

	// pick trajectory with lowest cost
	cout << possibleSTrajectories.size() << " feasible trajectories." << endl;
	vector<double> s_coeffs;
	vector<double> d_coeffs;
	double minCost = 10e10;
	for (int i = 0; i < possibleSTrajectories.size(); ++i) {
		const double cost = calculateCost(possibleSTrajectories[i], possibleDTrajectories[i], targetLane);
		if (cost < minCost) {
			minCost = cost;
			s_coeffs = possibleSTrajectories[i];
			d_coeffs = possibleDTrajectories[i];
		}
	}

	cout << "Min cost " << minCost << endl;
	// Generate chosen trajectory
	vector<double> next_xs, next_ys, accelSs, accelDs;
	// vector<tuple<double, double, double, double> > xyAccelSD;
	if (s_coeffs.size() > 0) {
		const double tDelta = 0.02;
		// for (auto s: s_coeffs) { cout << s << ","; }
		// cout << endl;
		this->prevSCoeffs = s_coeffs;
		this->prevDCoeffs = d_coeffs;
		for (double i = 0; i < SIMULATION_HORIZON; i+=tDelta) {
			const double next_s = eval(s_coeffs, i);
			const double next_d = eval(d_coeffs, i);
			const double s_accel = evalA(s_coeffs, i);
			const double d_accel = evalA(d_coeffs, i);
			const vector<double> xy = getXY(next_s , next_d, maps_s, maps_x, maps_y);
			// cout << next_s << ", " << next_d << ", " << xy[0] << ", " << xy[1] << endl;
		  next_xs.push_back(xy[0]);
		  next_ys.push_back(xy[1]);
		  accelSs.push_back(s_accel);
		  accelDs.push_back(d_accel);
			// xyAccelSD.push_back(make_tuple(xy[0], xy[1], s_accel, d_accel));
		}
	}
	this->prevTimestamp = currTimestamp;
	// this->prevXYAccelSD = xyAccelSD;
	return (next_xs.size() >= 5) ? smoothenPath(next_xs, next_ys, accelSs, accelDs) : make_pair(next_xs, next_ys);
}

// mostly for lane keeping, targetSpeed and v is in mph
pair<vector<double>, vector<double> > Planner::extendPath(double x, double y, double yaw, double v, double targetSpeed, int targetLane) {
	const int size = prevPathX.size();
	const double ms = mph2ms(v);
	const double targetSpeedMs = mph2ms(targetSpeed);
	const double displacement = 0.5 * (targetSpeedMs + ms) * PATH_PLANNING_HORIZON;
	vector<double> next_xs, next_ys, accelS, accelD;
	next_xs = prevPathX;
	next_ys = prevPathY;

	int numOfXYPassed = 0;
	if (this->prevXYAccelSD.size() > 0) {
		for (int i = 0; i < prevXYAccelSD.size(); ++i) {
			if (i == prevXYAccelSD.size()) cout << "Cannot find accelSD" << endl;
			if (distance(x, y, get<0>(prevXYAccelSD[i]), get<1>(prevXYAccelSD[i])) < 1e-3) {
				numOfXYPassed = i + 1;
				break;
			}
		}
	}

	cout << "<ExtendPath> numOfXYPassed " << numOfXYPassed << endl;

	for (int i = numOfXYPassed; i < prevXYAccelSD.size(); ++i) {
		accelS.push_back(get<2>(prevXYAccelSD[i]));
		accelD.push_back(get<3>(prevXYAccelSD[i]));
	}

	int ptsToAdd = (int) ceil(PATH_PLANNING_HORIZON/0.02 - size);
	if (ptsToAdd > 0) {
		const vector<double> sd = getFrenet(prevPathX[size-1], prevPathY[size-1], yaw, maps_x, maps_y);
		const double next_d = FSM::LANE_CENTER[targetLane];
		for (int i = 1; i <= ptsToAdd; ++i) {
			const double next_s = sd[0] + i * targetSpeedMs * 0.02;
			const vector<double> next_xy = getXY(next_s, next_d, maps_s, maps_x, maps_y);
			next_xs.push_back(next_xy[0]);
			next_ys.push_back(next_xy[1]);
			accelS.push_back(0);
			accelD.push_back(0);
		}
		// smoothen trajectory
		return smoothenPath(next_xs, next_ys, accelS, accelD);
	} else {
		return make_pair(next_xs, next_ys);
	}
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
	for (int i = 0; i <= PATH_PLANNING_HORIZON; ++i) {
		const double ss = eval(s_coeffs, i);
		const double vs = evalV(s_coeffs, i);
		const double as = evalA(s_coeffs, i);
		const double sd = eval(d_coeffs, i);
		const double vd = evalV(d_coeffs, i);
		const double ad = evalA(d_coeffs, i);
		if (sd > 12 || sd < -12) {
			// cout << "infeasible sd" << endl;
			return false;
		}
		if (fabs(vs) > MAX_VEL || vs < 0) {
			// cout << "infeasible vs" << endl;;
			return false;
		}
		if (fabs(vd) > MAX_VEL) {
			// cout << "infeasible vd" << endl;
			return false;
		}
		if (fabs(as) > MAX_ACCEL || as < 0) {
			// cout << "infeasible as" << endl;
			return false;
		}
		if (fabs(ad) > MAX_ACCEL) {
			// cout << "infeasible ad" << endl;
			return false;
		}

		// TODO: check steering angle
		double nextSS = eval(s_coeffs, i+1);
		double nextSD = eval(d_coeffs, i+1);
		const double steering_angle = atan2( (nextSD - sd), (nextSS - ss) );
		if (steering_angle > deg2rad(MAX_STEER_ANGLE) ) {
			cout << "infeasible steering angle "  << steering_angle << endl;
			return false;
		}
	}
	return true;
}

double Planner::calculateCost(const vector<double>& s_coeffs, const vector<double>& d_coeffs, int targetLane) {
	// Here we assume T = 10 is the end of the trajectory
	double cost = 0;
	const double startS = eval(s_coeffs, 0);
	const double startD = eval(d_coeffs, 0);
	const double endS = eval(s_coeffs, PATH_PLANNING_HORIZON);
	const double endD = eval(d_coeffs, PATH_PLANNING_HORIZON);

	// closeness to center of the lane
	cost += fabs(FSM::LANE_CENTER[targetLane] - endD);

	// safety distance to car in front
	cost += 1;

	// safety distance to car behind
	cost += 1;

	// closeness to previous trajectory - this could be seen as a feasibility
	// difference is discounted by timesteps into the future
	for (int i = 0; i < prevPathX.size(); ++i) {
		const double s = eval(s_coeffs, i*0.02);
		const double d = eval(d_coeffs, i*0.02);
		const vector<double> xy = getXY(s, d, maps_s, maps_x, maps_y);
		cost += 10.0 * distance(prevPathX[i], prevPathY[i], xy[0], xy[1]) / (i+1 * 1.0);
	}

	// comfort - we only count lateral jerk
	double total_lateral_jerk = 0;
	for (int i = 0; i <= PATH_PLANNING_HORIZON; ++i) {
		const double jd = evalJ(d_coeffs, i);
		total_lateral_jerk += jd;
	}
	cost += (total_lateral_jerk / PATH_PLANNING_HORIZON);
	cout << "cost " << cost << ", ";
	return cost;
}

// pathX and pathY must be sorted in ascending X
pair<vector<double>, vector<double> > Planner::smoothenPath(const vector<double>& pathX, const vector<double>& pathY, const vector<double>& accelS, const vector<double>& accelD) {
	tk::spline s;
	vector<tuple<double, double, double, double> > xyAccelSD;
	const int POINTS_TO_FIT = 5;
	assert(pathX.size() >= POINTS_TO_FIT);
	const int START = 0;
	const int END = pathX.size() - 1;
	const int MID = (int) (END - START) / 2;
	const int FIRST_QUARTILE = (int) (MID - START) / 2;
	const int THIRD_QUARTILE = (int) (MID + END) / 2;
	vector<double> smoothedX, smoothedY;
	s.set_points( { pathX[START], pathX[FIRST_QUARTILE], pathX[MID], pathX[THIRD_QUARTILE], pathX[END] }, 
								{ pathY[START], pathY[FIRST_QUARTILE], pathY[MID], pathY[THIRD_QUARTILE], pathY[END] });

	for (int i = 0; i < pathX.size(); ++i ) {
		smoothedX.push_back(pathX[i]);
		smoothedY.push_back(s(pathX[i]));
		// we need to use the smoothed xy values here
		xyAccelSD.push_back(make_tuple(pathX[i], s(pathX[i]), accelS[i], accelD[i]));
	}
	this->prevXYAccelSD = xyAccelSD;
	return make_pair(smoothedX, smoothedY);
}

vector<double> Planner::calculateDeltaSD(const double theta) {
	const vector<double> initialSD = getFrenet(get<0>(prevXYAccelSD[0]), get<1>(prevXYAccelSD[0]), theta, maps_x, maps_y);
	const vector<double> finalSD = getFrenet(prevPathX[0], prevPathY[0], theta, maps_x, maps_y);
	return { finalSD[0] - initialSD[0], finalSD[1] - initialSD[1] };
}