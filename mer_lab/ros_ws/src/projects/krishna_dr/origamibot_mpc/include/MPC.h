#ifndef MPC_H
#define MPC_H
#include <iostream>
#include <math.h>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>

#include <cppad/ipopt/solve.hpp>
#include <cppad/example/cppad_eigen.hpp>



using namespace std;

class MPC {
 public:

  /**
   * Constructor
   */
  MPC(std::unordered_map<string,double>& param);

  /**
   * Destructor
   */
  //virtual ~MPC();
      vector<double> mpc_pixel_x;
      vector<double> mpc_pixel_y;

      double _mpc_totalcost;
      double _mpc_cost_pixel_error_x;
      double _mpc_cost_pixel_error_y;
      double _mpc_cost_j1_vel;
      double _mpc_cost_j2_vel;

  /**
   * Solve the model given an initial state.
   * @param state state vector the the vehicle
   * @return Return the first actuator values, and the predicted state of x,y positions
   */
  vector<float> Solve(Eigen::VectorXf state);

  private:
        // Parameters for mpc solver
        int _dt, _mpc_steps, _pixel_x_start, _pixel_y_start,_pixel_z_start,
        _cable_l1_start,_cable_l2_start,_cable_l3_start,
        _motor1_vel_start,_motor2_vel_start,_motor3_vel_start;
        double _pixel_x_lb,_pixel_x_ub,_pixel_y_lb,_pixel_y_ub,
        _cable_lengths_lb,_cable_lengths_ub,_motor_vel_lb,_motor_vel_ub;
        std::unordered_map<string,double> param_mpc;
        
};

#endif /* MPC_H */
