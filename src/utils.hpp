#ifndef UTILS_HPP
#define UTILS_HPP

#include <sys/time.h>
#include <math.h>
#include "fsm.hpp"

#define INF 10E5
#define NINF -10E5

using namespace std;

static map<int, double> LANE_CENTER {
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

inline int calcCurrentLane(const double d) {
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

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

inline bool cmp(pair<double, double> a, pair<double, double> b) { return a.first < b.first; }

inline double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
inline double distToGoal(int start, int end, const vector<double>& maps_x, const vector<double>& maps_y) {
	if (start == end) return 0;
	const int size = maps_x.size();
	if (start > end) { start -= size; }
	double distToGoal = 0;
	for (int i = start; i < end; ++i) {
		distToGoal += distance(maps_x[i%size], maps_y[i%size], maps_x[(i+1) % size], maps_y[(i+1) % size]);
	}
	return distToGoal;
}
inline int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

inline int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
inline vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

inline double ms2mph(double x) { return x * 2.237; }
inline double mph2ms(double x) { return x / 2.237; }

// Transform speed from Cartesian x,y coordinates to Frenet s,d coordinates. theta is w.r.t x-axis
inline vector<double> getFrenetSpeed(double x, double y, double theta, double v_x, double v_y, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	// x and y components of the s vector
	double s_x = maps_x[next_wp]-maps_x[prev_wp];
	double s_y = maps_y[next_wp]-maps_y[prev_wp];

	// x and y components of the d vector: d_x * s_x + d_y * s_y = 0
	double d_x = 1.0; // we take direction of increasing x as +ve d
	double d_y = - (d_x * s_x) / s_y;

	// find the projection of v onto s
	double frenet_s_velocity = (v_x * s_x + v_y * s_y) / sqrt(s_x * s_x + s_y * s_y);
	// find the projection of v onto d
	double frenet_d_velocity = (v_x * d_x + v_y * d_y) / sqrt(d_x * d_x + d_y * d_y);

	return {frenet_s_velocity,frenet_d_velocity};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
inline vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

// returns lane curvature at s represented by a unit vector {dx, dy}
inline vector<double> getLaneCurvature(double s, const vector<double>& maps_s, const vector<double>& maps_x, const vector<double>& maps_y) {
	const vector<double> initialXY = getXY(s-1, 0, maps_s, maps_x, maps_y); // evaluate at d = 0
	const vector<double> finalXY = getXY(s+1, 0, maps_s, maps_x, maps_y);
	const double deltaX = finalXY[0] - initialXY[0];
	const double deltaY = finalXY[1] - initialXY[1];
	const double norm = sqrt( deltaX*deltaX + deltaY*deltaY);
	return { deltaX / norm, deltaY / norm };
}

inline double angleXYtoFrenet(double angle, const vector<double>& s_vector) {
	const double norm_h = sqrt(1 + tan(angle)*tan(angle));
	const double heading_projection = (s_vector[0]*1 + s_vector[1]*tan(angle)) / norm_h;
	return acos(heading_projection);
}

// given thetaFrenet angle w.r.t s-axis, transforms to angle w.r.t x-axis
inline double angleFrenetToXY(const double x, const double y, const double thetaFrenet, const double myHeading, const vector<double>& maps_x, const vector<double>& maps_y) {
	int next_wp = NextWaypoint(x,y, myHeading, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}
	const double deltaX = maps_x[next_wp] - maps_x[prev_wp];
	const double deltaY = maps_y[next_wp] - maps_y[prev_wp];
	const double cosTheta = 1 * deltaY * cos(thetaFrenet) / sqrt(deltaX*deltaX + deltaY*deltaY);
	return acos(cosTheta);
}

// Both must be in rad
inline double calcAngleDelta(double a, double b) {
	if (a > b) {
		const double temp = a;
		a = b;
		b = temp;
	}
	while (fabs(b - a) >= pi()) {
		b -= 2*pi();
	}
	return fabs(b - a);
}

inline unsigned long long clock_time_ms(void){
  struct timeval tv;
  gettimeofday(&tv, NULL);
  unsigned long long millisecondsSinceEpoch =
    (unsigned long long)(tv.tv_sec) * 1000 +
    (unsigned long long)(tv.tv_usec) / 1000;
  return millisecondsSinceEpoch;
}

#endif