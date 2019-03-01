#include "cost.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "vehicle.h"
#include "helper.h"
#include "spline.h"
#include "constants.h"

using std::string;
using std::vector;



double collision_cost(const Vehicle &ego, const vector<Vehicle> &other_vehicles){
  /**
  Binary cost function which penalizes collisions.
  */
	double nearest = nearest_approach_to_any_vehicle(ego, other_vehicles);
	std::cout << "**nearest:" << nearest <<std::endl;
	if (nearest<5) return 1.0;
	else return 0.0;
}


double buffer_cost(const Vehicle &ego, const vector<Vehicle> &other_vehicles){
  /**
  Penalizes getting close to other vehicles.
  */
	double nearest = nearest_approach_to_any_vehicle(ego, other_vehicles);
	return logistic(3/nearest);
}

double exceeds_speed_limit_cost(const Vehicle &ego, const vector<Vehicle> &other_vehicles){
	vector<double> v_s_JMT_cos = differentiate(ego.s_JMT_cos);
	vector<double> v_d_JMT_cos = differentiate(ego.d_JMT_cos);
	double T = ego.T;
	for(int i=0;i<100;i++){
		//double t = (i*1.0/100) * T;
		double t = i *0.02;
		double cur_v_s = get_equation_result(v_s_JMT_cos, t);
		double cur_v_d = get_equation_result(v_d_JMT_cos, t);
		double v = sqrt(cur_v_s*cur_v_s + cur_v_d*cur_v_d);
		std::cout << "**v:" << v <<std::endl;
		std::cout << "**cur_v_s:" << cur_v_s <<std::endl;
		if(v>=SPEED_LIMIT) return 1.0;
		if(cur_v_s<0) return 1.0;
	}
	return 0.0;
}

double efficiency_cost(const Vehicle &ego, const vector<Vehicle> &other_vehicles){
  /**
  Rewards high average speeds.
  */
	vector<double> s_JMT_cos = ego.s_JMT_cos;
	double T = ego.T;
	double delta_s = get_equation_result(s_JMT_cos, T/5) - get_equation_result(s_JMT_cos, 0);
	double v = delta_s/(T/5);
	return 1 - v/SPEED_LIMIT;

}

double total_accel_cost(const Vehicle &ego, const vector<Vehicle> &other_vehicles){
	return 0.0;
}

double max_accel_cost(const Vehicle &ego, const vector<Vehicle> &other_vehicles){
	vector<double> v_s_JMT_cos = differentiate(ego.s_JMT_cos);
	vector<double> v_d_JMT_cos = differentiate(ego.d_JMT_cos);
	vector<double> a_s_JMT_cos = differentiate(v_s_JMT_cos);
	vector<double> a_d_JMT_cos = differentiate(v_d_JMT_cos);
	double T = ego.T;
	double total = 0.0;
	int number = 100;//int(T/0.02);
	for(int i=0;i<number;i++){
		double t = (i*1.0/number) * T;
		double cur_a_s = get_equation_result(a_s_JMT_cos, t);
		double cur_a_d = get_equation_result(a_d_JMT_cos, t);
		double a = sqrt(cur_a_s*cur_a_s + cur_a_d*cur_a_d);
		total += a;
		if(a>=MAX_ACCEL) return 1.0;
	}
	return logistic(total/number);
}

double max_jerk_cost(const Vehicle &ego, const vector<Vehicle> &other_vehicles){
	vector<double> v_s_JMT_cos = differentiate(ego.s_JMT_cos);
	vector<double> v_d_JMT_cos = differentiate(ego.d_JMT_cos);
	vector<double> a_s_JMT_cos = differentiate(v_s_JMT_cos);
	vector<double> a_d_JMT_cos = differentiate(v_d_JMT_cos);
	vector<double> j_s_JMT_cos = differentiate(a_s_JMT_cos);
	vector<double> j_d_JMT_cos = differentiate(a_d_JMT_cos);
	double T = ego.T;
	double total = 0.0;
	for(int i=0;i<100;i++){
		double t = (i*1.0/100) * T;
		double cur_j_s = get_equation_result(j_s_JMT_cos, t);
		double cur_j_d = get_equation_result(j_d_JMT_cos, t);
		//std::cout << "**cur_j_s:" << cur_j_s <<std::endl;
		//std::cout << "**cur_j_d:" << cur_j_d <<std::endl;
		double j = sqrt(cur_j_s*cur_j_s + cur_j_d*cur_j_d);
		total += j;
		//std::cout << "**j:" << j <<std::endl;
		if(j>=MAX_JERK) {
			//std::cout << "**max_jerk_cost:" << j <<std::endl;
			return 1.0;}
	}
	std::cout << "max_jerk_cost:" << (total/100) <<std::endl;
	return logistic(total/100);
}


double total_jerk_cost(const Vehicle &ego, const vector<Vehicle> &other_vehicles){
	return 0.0;
}








