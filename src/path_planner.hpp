#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <vector>

using namespace std;

class Planner {
	public:
		Planner(const vector<double>& maps_x, const vector<double>& maps_y, const vector<double>& maps_s);
		virtual ~Planner();
		// generate a trajectory by choosing lowest cost among many possible end configuration states
		pair<vector<double>, vector<double> > generatePath(double x, double y, double s, double d, double yaw, double v, double targetSpeed, int targetLane);
		// extends an existing feasible trajectory without changing the final configuration, except for s-coordinate
		pair<vector<double>, vector<double> > extendPath(double x, double y, double yaw, double v, double targetSpeed, int targetLane);
		
		void updatePrevPaths(const vector<double>& prevX, const vector<double>& prevY);

	private:
		constexpr static double SIMULATION_HORIZON = 5.0; // seconds
		constexpr static double PATH_PLANNING_HORIZON = 5.0; // seconds
		constexpr static double TARGET_AVG_ACCEL = 8; // ms-2
		constexpr static double NONLINEARITY_CORRECTION_FACTOR = 0.6;
		constexpr static double MAX_VEL = 22.352;  // ms-1
		constexpr static double MAX_ACCEL = 10.0; // ms-2
		constexpr static double MAX_JERK = 10.0; // ms-3
		constexpr static double MAX_STEER_ANGLE = 35; // degrees
		constexpr static double SIGMA_D = 1;
		constexpr static double SIGMA_S = 10;
		constexpr static double SIGMA_T = 4;
		constexpr static int SAMPLE_SIZE = 50;
		unsigned long long prevTimestamp;
		vector<double> maps_s;
		vector<double> maps_x;
		vector<double> maps_y;
		vector<tuple<double, double, double, double> > prevXYAccelSD;
		vector<double> prevPathX;
		vector<double> prevPathY;
		vector<double> prevSCoeffs;
		vector<double> prevDCoeffs;

		vector<double> JMT(vector<double> start, vector<double> end, double T);
		double eval(const vector<double>& coeffs, double T);
		double evalV(const vector<double>& coeffs, double T);
		double evalA(const vector<double>& coeffs, double T);
		double evalJ(const vector<double>& coeffs, double T);
		vector<tuple<double, double, double> > generateEndConfigurations(int n, double center_s, double center_d, double T);
		bool isFeasible(const vector<double>& s_coeffs, const vector<double>& d_coeffs);
		double calculateCost(const vector<double>& s_coeffs, const vector<double>& d_coeffs, int targetLane);
		pair<vector<double>, vector<double> > smoothenPath(
			const vector<double>& pathX, 
			const vector<double>& pathY,
			const vector<double>& accelS,
			const vector<double>& accelD);
		vector<double> calculateDeltaSD(const double theta);
};

#endif