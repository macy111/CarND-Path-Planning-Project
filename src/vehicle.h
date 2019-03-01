#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>
#include "spline.h"

using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
  // Constructors
  Vehicle();
  Vehicle(double x, double y, double v_x, double v_y, double a, double s, double d, int state=0);
  // Destructor
  virtual ~Vehicle();

  // Vehicle functions
	vector<double> get_predict_position(double t);


	double x, y, v_x, v_y, a, s, d, v;
	int lane;
  int state;
	tk::spline spl;
	vector<double> s_JMT_cos, d_JMT_cos;
	double start_T, T;
};

#endif  // VEHICLE_H
