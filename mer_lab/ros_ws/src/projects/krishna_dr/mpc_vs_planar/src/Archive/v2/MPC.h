#ifndef MPC_H
#define MPC_H

#include <vector>
#include "eigen3/Eigen/Core"

using namespace std;

class MPC {
 public:

  /**
   * Constructor
   */
  MPC();

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
      Eigen::MatrixXf mpc_jacobian_img;
      Eigen::MatrixXf mpc_jacobian_robot;
      Eigen::MatrixXf mpc_input_vector;
      Eigen::MatrixXf mpc_pixel_st_init;
      Eigen::MatrixXf mpc_theta_st_init;
      Eigen::Vector2f mpc_pixel_st_out;
      Eigen::Vector2f mpc_theta_st_out;
  /**
   * Solve the model given an initial state.
   * @param state state vector the the vehicle
   * @return Return the first actuator values, and the predicted state of x,y positions
   */
  vector<float> Solve(Eigen::VectorXf state);
  private:
        // Parameters for mpc solver
        //double _max_angvel, _max_throttle, _bound_value;
        int _dt, _mpc_steps, _pixel_x_start, _pixel_y_start, _theta1_start,_theta2_start,_j1_vel_start,_j2_vel_start;
        

};

#endif /* MPC_H */
