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
  /**
   * Solve the model given an initial state.
   * @param state state vector the the vehicle
   * @return Return the first actuator values, and the predicted state of x,y positions
   */
  vector<float> Solve(Eigen::VectorXf state);
  private:
        // Parameters for mpc solver
        //double _max_angvel, _max_throttle, _bound_value;
        int _mpc_steps, _pixel_x_start, _pixel_y_start, _theta1_start,_theta2_start,_j1_vel_start,_j2_vel_start;
        

};

#endif /* MPC_H */
