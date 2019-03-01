#include "vehicle.h"
#include "helper.h"
#include "constants.h"

#include <math.h>
#include "Eigen-3.3/Eigen/Eigen"
#include "spline.h"

using std::map;
using std::string;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

double logistic(double x){
	/**
	A function that returns a value between 0 and 1 for x in the 
	range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

	Useful for cost functions.
	*/
	return 2.0 / (1 + exp(-x)) - 1.0;
}

double get_equation_result(vector<double> coefficients, double t){

	double total = 0.0;
	for(int i=0;i<coefficients.size();i++){
		total += coefficients[i] * pow(t,i);
	}
	return total;

}

vector<double> differentiate(vector<double> coefficients){
	/**
	Calculates the derivative of a polynomial and returns
	the corresponding coefficients.
	*/
	vector<double> new_cos;
	for(int i=1;i<coefficients.size();i++){
		new_cos.push_back(i*coefficients[i]);
	}
	return new_cos;
}

double nearest_approach_to_any_vehicle(Vehicle ego, vector<Vehicle> other_vehicles){
	double closest = 999999;
	for(int i=0;i<other_vehicles.size();i++){
		double d = nearest_approach(ego, other_vehicles[i]);
		if(d<closest) closest=d;
	}
	return closest;
}

double nearest_approach(Vehicle ego, Vehicle vehicle){
	double closest = 999999;
	vector<double> s_JMT_cos = ego.s_JMT_cos;
	vector<double> d_JMT_cos = ego.d_JMT_cos;
	double start_T = ego.start_T;
	//double T = ego.T;
	double T = 0.02;
	//tk::spline spl = ego.spl;
	for(int i=0;i<100;i++){
		double t = (i) * 0.02;
		double cur_s = get_equation_result(s_JMT_cos, t);
		double cur_d = get_equation_result(d_JMT_cos, t);
		vector<double> s_d = vehicle.get_predict_position(start_T+t);
		double diff_s = cur_s - s_d[0];
		double diff_d = cur_d - s_d[1];
		double dist = sqrt(diff_s*diff_s + diff_d*diff_d);
		if(dist<closest && abs(diff_d)<3) closest=dist;
	}
	return closest;
}

vector<vector<double>> get_f_and_N_derivatives(vector<double> coeffs, int N){
	vector<vector<double>> functions;
	functions.push_back(coeffs);
	for(int i=0;i<N;i++){
		coeffs = differentiate(coeffs);
		functions.push_back(coeffs);
	}
	return functions;
}

vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T,5*T*T*T*T,
       6*T, 12*T*T, 20*T*T*T;
    
  MatrixXd B = MatrixXd(3,1);     
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];
          
  MatrixXd Ai = A.inverse();
  
  MatrixXd C = Ai*B;
  
  vector <double> result = {start[0], start[1], .5*start[2]};

  for(int i = 0; i < C.size(); ++i) {
    result.push_back(C.data()[i]);
  }

  return result;
}





