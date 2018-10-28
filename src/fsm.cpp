#include "fsm.hpp"
#include <map>
#include <math.h>
#include <iostream>
#include "utils.hpp"

#define INF 10E5
#define NINF -10E5

using namespace std;

const double FSM::SPEED_LIMIT = 50;

const map<FSM::STATE, vector<FSM::STATE> > FSM::NEXT_STATE {
		{ FSM::notReady, vector<FSM::STATE> { FSM::notReady, FSM::ready }},
		{ FSM::ready, vector<FSM::STATE> { FSM::ready, FSM::keepLane }},
		{ FSM::keepLane, vector<FSM::STATE> { FSM::keepLane, FSM::laneChangeLeft, FSM::laneChangeRight, FSM::final }},
		{ FSM::laneChangeLeft, vector<FSM::STATE> { FSM::laneChangeLeft, FSM::keepLane }},
		{ FSM::laneChangeRight, vector<FSM::STATE> { FSM::laneChangeRight, FSM::keepLane }},
		{ FSM::final, vector<FSM::STATE> { FSM::final }}
	};
const map<int, double> FSM::LANE_CENTER {
		{NINF, -12 },
		{-3, -10 },
		{-2, -6 },
		{-1, -2 },
		{0, 0 },
		{1, 2 },
		{2, 6 },
		{3, 10 },
		{INF, 12}
};

FSM::FSM(const vector<double>& mapX, const vector<double>& mapY) {
	this->maps_x = mapX;
	this->maps_y = mapY;
	this->x = 0;
	this->y = 0;
	this->s = 0;
	this->d = 0;
	this->yaw = 0;
	this->v = 0;
	this->currentLane = calcCurrentLane(this->d);
	this->targetLane = INF;
	this->currentState = FSM::notReady;
}

FSM::~FSM() {}

void FSM::init(
	const double car_x,
	const double car_y, 
	const double car_s, 
	const double car_d, 
	const double car_yaw, 
	const double car_v, 
	const vector<vector<double> >& newPredictions)
{
	currentState = FSM::ready;
	updateLocalization(car_x, car_y, car_s, car_d, car_yaw, car_v);
	updateCurrentLane();
	updateGoal();
	updatePredictions(newPredictions);
	this->laneSpeeds = map<int, double> {
		{ NINF, -1 *FSM::SPEED_LIMIT },
		{ -3, -1 * FSM::SPEED_LIMIT },
		{ -2, -1 * FSM::SPEED_LIMIT },
		{ -1, -1 * FSM::SPEED_LIMIT },
		{ 0, -1 * FSM::SPEED_LIMIT },
		{ 1, FSM::SPEED_LIMIT },
		{ 2, FSM::SPEED_LIMIT },
		{ 3, FSM::SPEED_LIMIT },
		{ INF, -1 * FSM::SPEED_LIMIT }
	};
	targetLane = this->currentLane;
}

FSM::STATE FSM::getCurrentState() {
	return this->currentState;
}

void FSM::updateCurrentLane() {
	this->currentLane = calcCurrentLane(d);
}

int FSM::getCurrentLane() {
	return currentLane;
}

int FSM::calcCurrentLane(const double d) {
	double minDiff = 9e9;
	int lane = 0;
	for (auto p: LANE_CENTER) {
		const double diff = fabs(d - p.second);
		if (diff < minDiff) {
			lane = p.first;
			minDiff = diff;
		}
	}
	return lane;
}

// outputs the next state
FSM::STATE FSM::run(double x, double y, double s, double d, double yaw, double v, const vector<vector<double> >& newPredictions) {
	updateLocalization(x, y, s, d, yaw, v);
	updateCurrentLane();
	updatePredictions(newPredictions);
	updateLaneSpeeds();
	for (auto p: laneSpeeds) {
		if (p.first == 1 || p.first == 2 || p.first == 3) {
			cout << "lane " << p.first << " -> " << p.second << endl;
		}
	}
	targetLane = currentLane;
	double c = cost();
	cout << c << endl;
	return FSM::keepLane;
}

void FSM::updateLocalization(double x, double y, double s, double d, double yaw, double v) {
	this->x = x;
	this->y = y;
	this->s = s;
	this->d = d;
	this->yaw = yaw;
	this->v = v;
	nearestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);
	nextWaypoint = NextWaypoint(x, y, yaw, maps_x, maps_y);
}

void FSM::updatePredictions(const vector<vector<double> >& newPredictions) {
	this->predictions = newPredictions;
}

void FSM::updateGoal() {
	// if (nextWaypoint > nearestWaypoint) {
	// 	goal = (nextWaypoint + FSM::GOAL_HORIZON) % maps_x.size();
	// } else {
	// 	goal = (nextWaypoint - FSM::GOAL_HORIZON) % maps_x.size();
	// }
	cout << "updated goal!" << endl;
	goalLane = 1;
	goalWaypoint = (nextWaypoint + FSM::GOAL_HORIZON) % maps_x.size();
}

bool cmpPair(pair<double,double> a, pair<double, double> b) { return (a.first < b.first); }
void FSM::updateLaneSpeeds() {
	vector<pair<double, double> > lane1Cars, lane2Cars, lane3Cars; // vector of cars in each lane, to be sorted by increasing s
	for (auto p: predictions) {
		const double d = p[6];
		const double s = p[5];
		if (s + S_BUFFER - this->s > 0) {
			const int lane = calcCurrentLane(d);
			// Calculate frenet s speed given x and y speed
			const vector<double> frenetSpeeds = getFrenetSpeed(x, y, yaw, ms2mph(p[3]), ms2mph(p[4]), maps_x, maps_y);
			const double v = frenetSpeeds[0];
			if (lane == 1) {
				lane1Cars.push_back(make_pair(s, v));
			} else if (lane == 2) {
				lane2Cars.push_back(make_pair(s, v));
			} else if (lane == 3) {
				lane3Cars.push_back(make_pair(s, v));
			}
		}
	}
	sort(lane1Cars.begin(), lane1Cars.end(), cmpPair);
	sort(lane2Cars.begin(), lane2Cars.end(), cmpPair);
	sort(lane3Cars.begin(), lane3Cars.end(), cmpPair);
	if (lane1Cars.size() > 0) {
		laneSpeeds[1] = lane1Cars[0].second;
	} else {
		laneSpeeds[1] = FSM::SPEED_LIMIT;
	}
	if (lane2Cars.size() > 0) {
		laneSpeeds[2] = lane2Cars[0].second;
	} else {
		laneSpeeds[2] = FSM::SPEED_LIMIT;
	}
	if (lane3Cars.size() > 0) {
		laneSpeeds[3] = lane3Cars[0].second;
	} else {
		laneSpeeds[3] = FSM::SPEED_LIMIT;
	}
}

double FSM::cost() {
	double dist = distToGoal(nextWaypoint, goalWaypoint, maps_x, maps_y);
	if (dist < 30) updateGoal();
	cout << "DIST: " << dist << " current " << nearestWaypoint << " next " << nextWaypoint << " goalWaypoint " << goalWaypoint << endl;
	double c = 0;
	if (dist > 0) {
		c = 1 - exp(-(abs( 10 * (2.0*goalLane - targetLane - currentLane) ) / dist));
	} else {
		c = 1;
	}
	return c;
}