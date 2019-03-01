#ifndef HELPER_H
#define HELPER_H

#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

double logistic(double x);

double get_equation_result(vector<double> coefficients, double t);

vector<double> differentiate(vector<double> coefficients);

double nearest_approach_to_any_vehicle(Vehicle ego, vector<Vehicle> other_vehicles);

double nearest_approach(Vehicle ego, Vehicle vehicle);

vector<vector<double>> get_f_and_N_derivatives(vector<double> coeffs, int N=3);

vector<double> JMT(vector<double> &start, vector<double> &end, double T);


#endif  // HELPER_H
