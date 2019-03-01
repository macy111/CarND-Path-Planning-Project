#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "cost.h"
#include "spline.h"
#include "constants.h"

using std::string;
using std::vector;


// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(double x, double y, double v_x, double v_y, double a, double s, double d, int state) {
	this->x = x;
	this->y = y;
	this->v_x = v_x;
	this->v_y = v_y;
  this->a = a;
  this->s = s;
  this->d = d;
  this->lane = lane;
  this->state = state;
	this->lane = 0;
	if(d>=0&&d<4){
		this->lane = 1;
	}else if(d>=4&&d<8){
		this->lane = 2;
	}else if(d>=8&&d<12){
		this->lane = 3;
	}
	this->v = sqrt(v_x*v_x+v_y*v_y);
  //max_acceleration = -1;
}

Vehicle::~Vehicle() {}

vector<double> Vehicle::get_predict_position(double t){
	vector<double> s_d;
	s_d.push_back(this->s + this->v * t);
	s_d.push_back(this->d);
	return s_d;
}
















