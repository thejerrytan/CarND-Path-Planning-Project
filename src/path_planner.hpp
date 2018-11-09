#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <vector>

using namespace std;

class Planner {
	public:
		Planner(const vector<double>& maps_x, const vector<double>& maps_y, const vector<double>& maps_s);
		virtual ~Planner();
		// generate a trajectory by choosing lowest cost among many possible end configuration states
		tuple<bool, vector<double>, vector<double> > generatePath(double targetSpeed, int targetLane, bool appendOnly);
		// extends an existing feasible trajectory without changing the final configuration, except for s-coordinate
		pair<vector<double>, vector<double> > extendPath(double targetSpeed, int targetLane);
		void init(const double x, const double y, const double s, const double d, const double yaw, const double v);
		void updatePrevPaths(const vector<double>& prevX, const vector<double>& prevY);
		void updateState(double x, double y, double s, double d, double yaw, double v);
		void updatePredictions(const vector<vector<double> >& predictions);
		bool hasReachedEndOfTrajectory();
		bool hasTrajectory();

		bool hasBeenTruncated;

	private:
		constexpr static double PATH_PLANNING_HORIZON = 5.0; // seconds
		constexpr static double TARGET_AVG_ACCEL = 8.0; // ms-2
		constexpr static double NONLINEARITY_CORRECTION_FACTOR = 0.6;
		constexpr static double MAX_S = 6945.554;
		constexpr static double MAX_VEL = 22.00;  // ms-1, <50 mph
		constexpr static double MAX_ACCEL = 9.0; // ms-2
		constexpr static double MAX_DECEL = 4.0; // ms-2
		constexpr static double MAX_JERK = 9.0; // ms-3
		constexpr static double MAX_STEER_ANGLE = 60; // degrees
		constexpr static double SIGMA_D = 0.25;
		constexpr static double SIGMA_S = 50;
		constexpr static double SIGMA_T = 2;
		constexpr static int SAMPLE_SIZE = 100;
		constexpr static double CAR_S_SAFETY_DISTANCE = 30;
		constexpr static double CAR_D_SAFETY_DISTANCE = 0.25;
		constexpr static double CAR_S_COLLISION_DISTANCE = 5;
		constexpr static double CAR_D_COLLISION_DISTANCE = 0.25;
		unsigned long long prevTimestamp;
		double x,y,s,d,yaw,v;
		vector<double> maps_s;
		vector<double> maps_x;
		vector<double> maps_y;
		vector<tuple<double, double, double, double, double, double> > prevXYAccelSD;
		vector<vector<double> > predictions;
		vector<vector<pair<double, double> > > trajectories;
		map<int, double> distToCarAhead;
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
		vector<tuple<double, double, double> > generateEndConfigurations(int n, double center_s, double center_d, double T);
		bool isFeasible(const vector<double>& s_coeffs, const vector<double>& d_coeffs);
		double calculateCost(const vector<double>& s_coeffs, const vector<double>& d_coeffs, int targetLane, double elapsedTime);
		pair<vector<double>, vector<double> > smoothenPath(
			const vector<double>& pathX, 
			const vector<double>& pathY,
			const vector<double>& pathD,
			const vector<double>& pathS,
			const vector<double>& accelS,
			const vector<double>& accelD);
		void adjustForLatency(
			int numToPop, 
			vector<double>& xs, 
			vector<double>& ys, 
			vector<double>& ss, 
			vector<double>& ds, 
			vector<double>& as, 
			vector<double>& ad
		);
		void generateTrajectoriesForPredictions(const double T);
		vector<double> calculateDeltaSD(const double theta);
		double getSpeedAtEndOfPath();
		double getYawAtEndOfPath();
};

#endif