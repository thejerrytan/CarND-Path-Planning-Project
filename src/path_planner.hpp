#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <vector>

typedef std::pair<std::vector<double>, std::vector<double> > pairOfList;
typedef std::vector<std::pair<double, double> > listOfPair;

using namespace std;

class Planner {
	public:
		Planner(const vector<double>& maps_x, const vector<double>& maps_y, const vector<double>& maps_s);
		virtual ~Planner();
		// generate a trajectory by choosing lowest cost among many possible end configuration states
		pairOfList generatePath(double targetSpeed, int targetLane, int appendIdx, double timeHorizon);
		// extends an existing feasible trajectory without changing the final configuration, except for s-coordinate
		pairOfList extendPath(double targetSpeed, int targetLane);
		void init(const double x, const double y, const double s, const double d, const double yaw, const double v);
		void updatePrevPaths(const vector<double>& prevX, const vector<double>& prevY);
		void updateState(double x, double y, double s, double d, double yaw, double v);
		void updatePredictions(const vector<vector<double> >& predictions);
		bool hasReachedEndOfTrajectory();
		bool hasTrajectory();
		bool onCollisionCourse(bool useXY);

		map<int, double> distToCarAhead; 
		map<int, double> distToCarBehind;
		map<int, double> speedOfApproachFromBehind;
		map<int, double> speedOfApproachFromFront;
		int numOfXYPassed; // for current time loop
		int totalXYPassed;
		double distTravelled; // distance travelled since last timestep, in m/s 
		bool isOnCollisionCourse;
		int loopCount;

	private:
		constexpr static double PATH_PLANNING_HORIZON = 5.0; // seconds
		constexpr static double TARGET_AVG_ACCEL = 8.0; // ms-2
		constexpr static double MAX_S = 6945.554;
		constexpr static double MAX_VEL = 21.0109;  // ms-1, <50 mph
		constexpr static double MAX_VEL_ALLOWANCE = 0.9; // ms-1
		constexpr static double MAX_ACCEL = 9.0; // ms-2
		constexpr static double MAX_S_DECEL = 6.0; // ms-2
		constexpr static double MAX_D_DECEL = 2.0; // ms-2
		constexpr static double MAX_JERK = 9.0; // ms-3
		constexpr static double MAX_STEER_ANGLE = 40; // degrees
		constexpr static double SIGMA_D = 0.25;
		constexpr static double SIGMA_S = 15;
		constexpr static double SIGMA_T = 2;
		constexpr static int SAMPLE_SIZE = 50;
		constexpr static double CAR_S_LANE_CHANGE_SAFETY_DISTANCE = 20;
		constexpr static double CAR_S_SAFETY_DISTANCE = 30;
		constexpr static double CAR_D_SAFETY_DISTANCE = 4.00;
		constexpr static double CAR_S_COLLISION_DISTANCE = 5;
		constexpr static double CAR_D_COLLISION_DISTANCE = 3.00;
		constexpr static double CAR_XY_COLLISION_DISTANCE = 3.00;
		constexpr static double LANE_SWITCH_TIME = 5; // ms-1
		unsigned long long prevTimestamp;
		double x,y,s,d,yaw,v;
		int currentLane;
		vector<double> maps_s;
		vector<double> maps_x;
		vector<double> maps_y;
		vector<tuple<double, double, double, double, double, double> > prevXYAccelSD;
		vector<vector<double> > predictions;
		vector<vector<pair<double, double> > > trajectories;
		vector<listOfPair> trajectoriesXY;
		vector<double> prevPathX;
		vector<double> prevPathY;
		vector<double> prevSCoeffs;
		vector<double> prevDCoeffs;
		vector<vector<double> > endOfCurrentTrajectory; // sd-coordinate of end of current trajectory
		bool hasTrajectoryBefore;

		vector<double> JMT(vector<double> start, vector<double> end, double T);
		double eval(const vector<double>& coeffs, double T);
		double evalV(const vector<double>& coeffs, double T);
		double evalA(const vector<double>& coeffs, double T);
		double evalJ(const vector<double>& coeffs, double T);
		vector<tuple<double, double, double> > generateEndConfigurations(int n, double center_s, double sigma_s, double center_d, double sigma_d, double T, double sigma_t);
		bool isFeasible(int startIdx, double timeHorizon, const vector<double>& s_coeffs, const vector<double>& d_coeffs, int targetLane);
		double calculateCost(const vector<double>& s_coeffs, const vector<double>& d_coeffs, int targetLane, double timeHorizon);
		pair<vector<double>, vector<double> > smoothenPath(
			int startIdx,
			const vector<double>& pathX, 
			const vector<double>& pathY,
			const vector<double>& pathD,
			const vector<double>& pathS,
			const vector<double>& accelS,
			const vector<double>& accelD);
		void generateTrajectoriesForPredictions(double T);
		double normalizeValues(vector<double>& values);
		double normalizedVariance(vector<double>& values);
		double getSpeedAtPath(int idx);
		double getYawAtPath(int idx);
};

#endif