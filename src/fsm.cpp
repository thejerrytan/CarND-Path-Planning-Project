#include "fsm.hpp"
#include <map>
#include <math.h>
#include <algorithm>
#include <iostream>
#include "utils.hpp"

#define INF 10E5
#define NINF -10E5
#define DEBUG true

using namespace std;

double FSM::SPEED_LIMIT = 47; // mph

map<FSM::STATE, vector<FSM::STATE> > FSM::NEXT_STATE {
		{ FSM::notReady, vector<FSM::STATE> { FSM::notReady, FSM::ready }},
		{ FSM::ready, vector<FSM::STATE> { FSM::ready, FSM::keepLane }},
		{ FSM::keepLane, vector<FSM::STATE> { FSM::keepLane, FSM::laneChangeLeft, FSM::laneChangeRight }},
		// { FSM::keepLane, vector<FSM::STATE> { FSM::keepLane, FSM::final }},
		{ FSM::laneChangeLeft, vector<FSM::STATE> { FSM::keepLane }},
		{ FSM::laneChangeRight, vector<FSM::STATE> { FSM::keepLane }},
		{ FSM::final, vector<FSM::STATE> { FSM::final }}
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
	this->isInTransit = false;
}

FSM::~FSM() {}

void FSM::init(
	const double car_x,
	const double car_y, 
	const double car_s, 
	const double car_d, 
	const double car_yaw, 
	const double car_v, 
	const vector<vector<double> >& newPredictions,
	Planner &path_planner)
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
	this->planner = &path_planner;
	this->planner->init(x, y, s, d, yaw, v);
}

FSM::STATE FSM::getCurrentState() {
	return this->currentState;
}

FSM::STATE FSM::getNextState() {
	return this->nextState;
}

void FSM::updateCurrentLane() {
	this->currentLane = calcCurrentLane(d);
}

int FSM::getCurrentLane() {
	return currentLane;
}

// outputs the next state
FSM::STATE FSM::run(double x, double y, double s, double d, double yaw, double v, const vector<vector<double> >& newPredictions) {
	updateLocalization(x, y, s, d, yaw, v);
	updateCurrentLane();
	updatePredictions(newPredictions);
	updateLaneSpeeds();
	for (auto p: laneSpeeds) {
		if (p.first == 1 || p.first == 2 || p.first == 3) {
			if (DEBUG) cout << "[FSM] lane speeds " << p.first << " -> " << p.second << endl;
		}
	}

	// check whether path planner has finished executing any pending trajectory to transit to the next state
	if (!planner->hasTrajectory() && planner->hasReachedEndOfTrajectory()) {
		if (DEBUG) cout << "[FSM] Reached next state " << enumToString(nextState) << endl;
		isInTransit = false;
		currentState = nextState;
	}

	// If still in transition, extend current trajectory, return currentState
	if (isInTransit) {
		if (DEBUG) cout << "[FSM] Transiting to next state " << enumToString(nextState) << endl;
		next_paths = planner->extendPath(targetSpeed, targetLane);
		return currentState;
	}

	// we set goalDistance weight to 0 because there is no actual goal here
	vector<double> weight_list = { 0.0, 0.5, 0.5 }; 
	vector<FSM::STATE> nextStates = NEXT_STATE[currentState];
	FSM::STATE chosenState;
	double chosenTargetSpeed = 0;
	int chosenTargetLane = 0;

	double minCost = 10E10;
	if (DEBUG) cout << "[FSM] Calculating costs: " << endl;
	for (auto state: nextStates) {
		const tuple<double, int> config = generateEndConfiguration(state);
		const double targetSpeed = get<0>(config);
		const double targetLane = get<1>(config);
		const double goalCost = weight_list[0] * goalDistanceCost(targetSpeed, targetLane);
		const double speedCost = weight_list[1] * inefficiencyCost(targetSpeed, targetLane);
		const double safety = weight_list[2] * safetyCost(targetSpeed, targetLane);
		const double c = (goalCost + speedCost + safety) / 1.0;
		if (DEBUG) cout << "[FSM]    " << enumToString(state)
		 << " : " << goalCost << ", " << speedCost 
		 << " , " << safety << endl;
		if (c < minCost) {
			chosenState = state;
			minCost = c;
			chosenTargetLane = targetLane;
			chosenTargetSpeed = targetSpeed;
		}
	}
	
	// Next state
	targetLane = chosenTargetLane;
	targetSpeed = max(0.0, min(FSM::SPEED_LIMIT, chosenTargetSpeed));
	nextState = chosenState;

	if (DEBUG) cout << "[FSM] Current state: " 
		<< enumToString(currentState) << " Next state: " << enumToString(chosenState) 
		<< " currentLane : " << currentLane << " targetLane : "  << targetLane 
		<< " currentSpeed : " << v << " targetSpeed : " << targetSpeed << endl;

	// Issue instructions to path planner to transit
	if (nextState == FSM::keepLane && ((targetSpeed - v) > 5 || (v - targetSpeed) > 5)) {
		if ((targetSpeed - v) > 5 && DEBUG) cout << "[FSM] Keep lane but speed up" << endl; 
		if ((v - targetSpeed) > 5 && DEBUG) cout << "[FSM] Keep lane but slow down" << endl;
		pairOfList result = planner->generatePath(targetSpeed, targetLane, -1, -1); // extend path but change final speed
		if (planner->hasTrajectory()) {
			next_paths = result;
			isInTransit = true;
		} else {
			// Do not make the transit
			next_paths = planner->extendPath(targetSpeed, currentLane);
		}
	} else if (currentState != chosenState) {
		pairOfList result = planner->generatePath(targetSpeed, targetLane, -1, -1);
		if (planner->hasTrajectory()) {
			next_paths = result;
			isInTransit = true;
		} else {
			// Do not make the transit
			next_paths = planner->extendPath(targetSpeed, currentLane);
		}
	} else {
		// default, extend path
		next_paths = planner->extendPath(targetSpeed, targetLane);
	}
	return chosenState;
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
	if (DEBUG) cout << "[FSM] Localization: (x:" << x << "), (y:" << y << "), (s:" << s << "), (d:" << d << "), (yaw:" << yaw << "), (v:" << v << "), (currentLane:" << currentLane << ")" << endl;
}

void FSM::updatePredictions(const vector<vector<double> >& newPredictions) {
	this->predictions = newPredictions;
}

void FSM::updateGoal() {
	if (DEBUG) cout << "[FSM] updated goal!" << endl;
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

// returns a <targetSpeed, targetLane> for each currentState -> nextState transition
tuple<double, int> FSM::generateEndConfiguration(FSM::STATE end) {
	tuple<double, int> t;
	int targetLane;
	switch (end) {
		case (FSM::notReady) : 
			t = make_tuple(0.0, 1);
			break;
		case (FSM::ready) : 
			t = make_tuple(0.0, this->currentLane);
			break;
		case (FSM::keepLane) : 
			t = make_tuple(laneSpeeds[currentLane], currentLane);
			break;
		case (FSM::laneChangeLeft) : 
			targetLane = currentLane - 1;
			t = make_tuple(laneSpeeds[targetLane], targetLane);
			break;
		case (FSM::laneChangeRight) :
			targetLane = currentLane + 1;
			t = make_tuple(laneSpeeds[targetLane], targetLane);
			break;
		case (FSM::final) :
			t = make_tuple(laneSpeeds[goalLane], 0);
			break;
	 	default: 
			t = make_tuple(0, 0);
			break;
	}
	return t;
}

double FSM::goalDistanceCost(double targetSpeed, int targetLane) {
	double dist = distToGoal(nextWaypoint, goalWaypoint, maps_x, maps_y);
	if (dist < 30) updateGoal();
	double c = 0;
	if (dist > 0) {
		c = 1 - exp(-(abs( 10 * (2.0*goalLane - targetLane - currentLane) ) / dist));
	} else {
		c = 1;
	}
	return c;
}

double FSM::inefficiencyCost(double targetSpeed, int targetLane) {
	if (targetSpeed == 0) return 1.0; // TODO: need better handling for this case?
	double cost = (FSM::SPEED_LIMIT - targetSpeed) / FSM::SPEED_LIMIT;
	return cost;
}

double FSM::safetyCost(double targetSpeed, int targetLane) {
	if (targetLane <= 0 || targetLane > 3) return INF;
	return (INF - planner->distToCarAhead[targetLane]) / INF + 
		   (INF - planner->distToCarBehind[targetLane]) / INF;
}

string FSM::enumToString(FSM::STATE s) {
	switch (s) {
		case (FSM::notReady): return "NOT_READY";
		case (FSM::ready): return "READY";
		case (FSM::keepLane): return "KEEP_LANE";
		case (FSM::laneChangeLeft): return "LANE_CHANGE_LEFT";
		case (FSM::laneChangeRight): return "LANE_CHANGE_RIGHT";
		case (FSM::final): return "FINAL";
		default: return "NONE";
	}
}