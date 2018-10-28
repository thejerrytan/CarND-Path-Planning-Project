#ifndef FSM_HPP
#define FSM_HPP

#include <vector>
#include <map>

using namespace std;

class FSM {
public:
	enum STATE { notReady, ready, keepLane, laneChangeLeft, laneChangeRight, final };
	FSM(const vector<double>& mapX, const vector<double>& mapY);
	virtual ~FSM();
	void init(double x, double y, double s, double d, double yaw, double v, const vector<vector<double> >& predictions);
	int getCurrentLane();
	STATE getNextState();
	STATE getCurrentState();
	STATE run(double x, double y, double s, double d, double yaw, double v, const vector<vector<double> >& predictions);

private:
	// cars within s_buffer in s coordinates behind us should be considered in lane speed calc
	constexpr static double S_BUFFER = 10;
	constexpr static int GOAL_HORIZON = 10;
	const static double SPEED_LIMIT; //mph
	const static map<int, double> LANE_CENTER;
	const static map<STATE, vector<STATE> > NEXT_STATE;
	map<int, double> laneSpeeds;
	vector<double> maps_x;
	vector<double> maps_y;
	double x, y, yaw, v, s, d;
	int currentLane, targetLane;
	int nearestWaypoint, nextWaypoint, goalWaypoint, goalLane;
	STATE currentState;
	vector<vector<double> > predictions;

	void updateLocalization(double x, double y, double s, double d, double yaw, double v);
	void updatePredictions(const vector<vector<double> >& predictions);
	void updateLaneSpeeds();
	void updateGoal();
	void updateCurrentLane();
	int calcCurrentLane(const double d);
	double cost();



};

#endif