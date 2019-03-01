#ifndef COST_H
#define COST_H

#include "vehicle.h"
#include "constants.h"


using std::map;
using std::string;
using std::vector;



double collision_cost(const Vehicle &ego, const vector<Vehicle> &other_vehicles);

double buffer_cost(const Vehicle &ego, const vector<Vehicle> &other_vehicles);

double exceeds_speed_limit_cost(const Vehicle &ego, const vector<Vehicle> &other_vehicles);

double efficiency_cost(const Vehicle &ego, const vector<Vehicle> &other_vehicles);

double total_accel_cost(const Vehicle &ego, const vector<Vehicle> &other_vehicles);

double max_accel_cost(const Vehicle &ego, const vector<Vehicle> &other_vehicles);

double max_jerk_cost(const Vehicle &ego, const vector<Vehicle> &other_vehicles);

double total_jerk_cost(const Vehicle &ego, const vector<Vehicle> &other_vehicles);


#endif  // COST_H
