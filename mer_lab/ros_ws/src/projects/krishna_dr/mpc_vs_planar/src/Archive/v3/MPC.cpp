#include "MPC.h"

using CppAD::AD;
using Eigen::Matrix;
using Eigen::Dynamic;

typedef Matrix< AD<double>, Dynamic,Dynamic > eg_matrix;
typedef Matrix< AD<double>, Dynamic,1 > eg_vector;
eg_matrix jacobian_robot(6,2);// Robot Jacobian
eg_matrix jacobian_img(2,6);  // Image Jacobian
eg_vector input_vector(2);  // Input vector[w1,w2] at K step
eg_vector pixel_st_init(2); // State vector at time K step
eg_vector pixel_st_out(2);  // State vector
eg_vector theta_st_init(2); // Joint angles at time K step
eg_vector theta_st_out(2);  // Joint angles at K+1 step

class FG_eval
{
    public:
    int _pixel_x_start, _pixel_y_start, _theta1_start,_theta2_start; // state variables index
    int _j1_vel_start,_j2_vel_start; // output variables Index
    // int _j1_accel_start,_j2_accel_start; // output variables Index
    /** Parameters **/
    int _mpc_steps; // Prediction Step size
    double _dt; // Sample time
    double _ref_pixel_pose_x, _ref_pixel_pose_y; // Reference Pose
    double  _w_pixel_error_x,_w_pixel_error_y, _w_j1_vel,_w_j2_vel, _w_j1_acc,_w_j2_acc; // Weights for cost function
    double j1_accl_limit;
    double j2_accl_limit;
    double focus;//Focus from Gazebo Camera info
    double Z;  //depth-2
    double focus_const; // 178.76/2
    double c_intrinsic; //principle point centre offset
    double l1; //link-1 length
    double l2; //link-2 length

    AD<double> cost_pixel_error_x,cost_pixel_error_y, cost_j1_vel,cost_j2_vel;
    //constructor
    FG_eval(unordered_map<string,double>& param_fg) 
    { 
        // Set default value    
        _mpc_steps           = param_fg["mpc_steps"];
        _dt                  = param_fg["mpc_dt"];  // in sec
        _ref_pixel_pose_x    = param_fg["goal_pos_x"];
        _ref_pixel_pose_y    = param_fg["goal_pos_y"]; 
        _w_pixel_error_x     = param_fg["w_pixel_x"];
        _w_pixel_error_y     = param_fg["w_pixel_y"];
        _w_j1_vel            = param_fg["w_j1_vel"];
        _w_j2_vel            = param_fg["w_j2_vel"];
        _w_j1_acc            = param_fg["w_j1_accl"];
        _w_j2_acc            = param_fg["w_j2_accl"];
        j1_accl_limit        = param_fg["j1_accl_limit"];
        j2_accl_limit        = param_fg["j2_accl_limit"];
        focus                = param_fg["focus"];
        Z                    = param_fg["depth"];
        c_intrinsic          = param_fg["c_intrinsic"];
        l1                   = param_fg["l1"];
        l2                   = param_fg["l2"];
        focus_const          = focus/Z;
        
        
        _pixel_x_start       = 0;
        _pixel_y_start       = _pixel_x_start + _mpc_steps;
        _theta1_start        = _pixel_y_start + _mpc_steps;
        _theta2_start        = _theta1_start + _mpc_steps;
        _j1_vel_start        = _theta2_start + _mpc_steps;
        _j2_vel_start        = _j1_vel_start + _mpc_steps - 1;
        // _j1_accel_start      = _j2_vel_start  + _mpc_steps;
        // _j2_accel_start      = _j1_accel_start  + _mpc_steps;
    }
    

    // MPC implementation (cost func & constraints)
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector; 
    // fg: function that evaluates the objective and constraints using the syntax       
    void operator()(ADvector& fg, const ADvector& vars) 
    {
        /************************fg[0] for cost function**************************************/ 
        fg[0] = 0;
        cost_pixel_error_x =  0;
        cost_pixel_error_y =  0;
        int final_index =0;
        for (int i = 0; i < _mpc_steps; i++) 
        {
            fg[0] += _w_pixel_error_x * CppAD::pow(vars[_pixel_x_start + i] - _ref_pixel_pose_x, 2); // Pixel Frame error
            fg[0] += _w_pixel_error_y * CppAD::pow(vars[_pixel_y_start + i] - _ref_pixel_pose_y, 2); // Pixel Frame error
            cost_pixel_error_x +=  _w_pixel_error_x * CppAD::pow(vars[_pixel_x_start + i] - _ref_pixel_pose_x, 2);
            cost_pixel_error_y +=  _w_pixel_error_y * CppAD::pow(vars[_pixel_y_start + i] - _ref_pixel_pose_y, 2);
            //std::cout<<"cost x "<<cost_pixel_error_x<<std::endl;
        }
        std::cout << "-----------------------------------------------" <<std::endl;
        std::cout << "cost_pixel =(x,y): " << cost_pixel_error_x << ", " << cost_pixel_error_y << std::endl;
        

        // Minimize the use of actuators.
        for (int i = 0; i < _mpc_steps - 1; i++) 
        {
            fg[0] += _w_j1_vel * CppAD::pow(vars[_j1_vel_start + i], 2);
            fg[0] += _w_j2_vel * CppAD::pow(vars[_j2_vel_start + i], 2);
            cost_j1_vel += _w_j1_vel * CppAD::pow(vars[_j1_vel_start + i], 2);
            cost_j2_vel += _w_j2_vel * CppAD::pow(vars[_j2_vel_start + i], 2);
            //cout << "velocity  Cost: " << cost_j1_vel <<", "<< cost_j2_vel<< endl;
        }
        //cout << "Total Cost: " << fg[0] << endl; 

        //Minimize the value gap between sequential actuations.
        for (int i = 0; i < _mpc_steps - 2; i++) 
        {
            fg[0] += _w_j1_acc * CppAD::pow(vars[_j1_vel_start + i + 1] - vars[_j1_vel_start + i], 2);
            fg[0] += _w_j2_acc * CppAD::pow(vars[_j2_vel_start + i + 1] - vars[_j2_vel_start + i], 2);
        }
        //cout << "cost of gap: " << fg[0] << endl; 
        

        /************************fg[x] for constraints**************************************/ 
        // Initial constraints
        fg[1 + _pixel_x_start] = vars[_pixel_x_start];
        fg[1 + _pixel_y_start] = vars[_pixel_y_start];
        fg[1 + _theta1_start]  = vars[_theta1_start];
        fg[1 + _theta2_start]  = vars[_theta2_start];
        // Add system dynamic model constraint
        for (int i = 0; i < _mpc_steps - 1; i++)
        {
            // The state at time t+1 .
            AD<double> pixel_x1 = vars[_pixel_x_start + i + 1];
            AD<double> pixel_y1 = vars[_pixel_y_start + i + 1];
            AD<double> theta1_1 = vars[_theta1_start + i + 1];
            AD<double> theta2_1 = vars[_theta2_start + i + 1];

            // The state at time t.
            AD<double> pixel_x0 = vars[_pixel_x_start + i];
            AD<double> pixel_y0 = vars[_pixel_y_start + i];
            AD<double> theta1_0 = vars[_theta1_start + i];
            AD<double> theta2_0 = vars[_theta2_start + i];

            // Only consider the actuation at time t.
            AD<double> j1_vel_0 = vars[_j1_vel_start + i];
            AD<double> j2_vel_0 = vars[_j2_vel_start + i];

            // Here's `x` to get you started.
            // The idea here is to constraint this value to be 0
            // NOTE: The use of `AD<double>` and use of `CppAD`!
            // This is also CppAD can compute derivatives and pass
            // these to the solver.
            // TODO: Setup the rest of the model constraints
            jacobian_img << -focus_const, 0, 0, 0,-focus, (vars[_pixel_y_start + i] - c_intrinsic),
            0, -focus_const , 0 , focus ,0 , (c_intrinsic - vars[_pixel_x_start + i]);

            jacobian_robot << ((l1*CppAD::cos(theta1_0))+(l2*CppAD::cos(theta1_0+theta2_0))), ((l2*CppAD::cos(theta1_0+theta2_0))),
            ((l1*CppAD::sin(theta1_0))+(l2*CppAD::sin(theta1_0+theta2_0))) ,((l2*CppAD::sin(theta1_0+theta2_0))),
            0,0,
            0,0,
            0,0,
            1,1;
            input_vector  << vars[_j1_vel_start + i] , vars[_j2_vel_start + i];
            pixel_st_init << vars[_pixel_x_start + i], vars[_pixel_y_start + i];
            theta_st_init << vars[_theta1_start + i] , vars[_theta2_start + i];
            
            pixel_st_out = pixel_st_init + (((jacobian_img*jacobian_robot)*input_vector) * _dt);
            theta_st_out = theta_st_init + (input_vector * _dt);

            fg[2 + _pixel_x_start + i] = pixel_x1 - pixel_st_out[0];
            fg[2 + _pixel_y_start + i] = pixel_y1 - pixel_st_out[1];
            fg[2 + _theta1_start + i]  = theta1_1 - theta_st_out[0];
            fg[2 + _theta2_start + i]  = theta2_1 - theta_st_out[1]; 

            /**********Acceleration Constraints***********/
            if(vars[_j1_vel_start + i]<0){j1_accl_limit = -j1_accl_limit;}
            fg[9] = vars[_j1_vel_start + i+1] - (vars[_j1_vel_start + i]+ (j1_accl_limit *_dt));
            if(vars[_j2_vel_start + i]<0){j2_accl_limit = -j2_accl_limit;}
            fg[11] = vars[_j2_vel_start + i+1] - (vars[_j2_vel_start + i]+ (j2_accl_limit *_dt));

        }
                  
    }

};

// ====================================
// MPC class definition implementation.
// ====================================
MPC::MPC(std::unordered_map<string,double>& param) 
{
    // Set default value    
    _mpc_steps           = param["mpc_steps"];
    _pixel_x_lb          = param["pixel_x_lb"];
    _pixel_x_ub          = param["pixel_x_ub"];
    _pixel_y_lb          = param["pixel_y_lb"];
    _pixel_y_ub          = param["pixel_y_ub"];
    _theta_1_lb          = param["theta_1_lb"];
    _theta_1_ub          = param["theta_1_ub"];
    _theta_2_lb          = param["theta_2_lb"];
    _theta_2_ub          = param["theta_2_ub"];
    _j1_vel_lb           = param["j1_vel_lb"];
    _j1_vel_ub           = param["j1_vel_ub"];
    _j2_vel_lb           = param["j2_vel_lb"];
    _j2_vel_ub           = param["j2_vel_ub"];
    _pixel_x_start       = 0;
    _pixel_y_start       = _pixel_x_start + _mpc_steps;
    _theta1_start        = _pixel_y_start + _mpc_steps;
    _theta2_start        = _theta1_start  + _mpc_steps;
    _j1_vel_start        = _theta2_start  + _mpc_steps;
    _j2_vel_start        = _j1_vel_start  + _mpc_steps;

    param_mpc = param; 


}


vector<float> MPC::Solve(Eigen::VectorXf state) 
{
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // Set the number of model variables (includes both states and inputs)
    /* State Variables = [px,py,theta_1,theta_2]
       Actuator variables = [j1_vel,j2_vel] */
    size_t n_vars = _mpc_steps * 4 + (_mpc_steps) * 2;
    /****** Set the number of constraints *******/
    /* Pixel model Constraints(px,py) = 2 
    Joint angle Constraints(theta1,theta2) = 2
    Joint acceleration constriants = 2 */
    size_t n_constraints = _mpc_steps * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) 
    {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[_pixel_x_start] = state[0];
    vars[_pixel_y_start] = state[1];
    vars[_theta1_start]  = state[2];
    vars[_theta2_start]  = state[3];
    vars[_j1_vel_start]  = state[4];//0.001
    vars[_j2_vel_start]  = state[5];//0.001
    

    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);


    for (int i = _pixel_x_start; i < _theta1_start; i++) 
    {
        vars_lowerbound[i] = _pixel_x_lb;
        vars_upperbound[i] = _pixel_x_ub;
    }
    for (int i = _theta1_start; i < _j1_vel_start; i++) 
    {
        vars_lowerbound[i] = _theta_1_lb;
        vars_upperbound[i] = _theta_1_ub;
    }

    for (int i = _j1_vel_start; i < _j2_vel_start; i++) 
    {
        vars_lowerbound[i] = _j1_vel_lb;
        vars_upperbound[i] = _j1_vel_ub;
    }
    // Acceleration/decceleration upper and lower limits
    for (int i = _j2_vel_start; i < n_vars; i++)  
    {
        vars_lowerbound[i] = _j2_vel_lb;
        vars_upperbound[i] = _j2_vel_ub;
    }


    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    
    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[_pixel_x_start] = state[0];
    constraints_lowerbound[_pixel_y_start] = state[1];
    constraints_lowerbound[_theta1_start]  = state[2];
    constraints_lowerbound[_theta2_start]  = state[3];
    
    
    constraints_upperbound[_pixel_x_start] = state[0];
    constraints_upperbound[_pixel_y_start] = state[1];
    constraints_upperbound[_theta1_start]  = state[2];
    constraints_upperbound[_theta2_start]  = state[3];

    // object that computes objective and constraints
    FG_eval fg_eval(param_mpc);

    /****** options for IPOPT solver  ******/
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    std::cout<<"Status: "<<solution.status<<std::endl;
    // Cost
    auto cost = solution.obj_value;
    std::cout << "------------ Total Cost(solution): " << cost << "------------" << std::endl;
    cout << "-----------------------------------------------" <<endl;
    _mpc_totalcost = cost;
    _mpc_cost_pixel_error_x = Value(fg_eval.cost_pixel_error_x);
    _mpc_cost_pixel_error_y = Value(fg_eval.cost_pixel_error_y);
    _mpc_cost_j1_vel = Value(fg_eval.cost_j1_vel);
    _mpc_cost_j2_vel = Value(fg_eval.cost_j2_vel);

    this->mpc_pixel_x = {};
    this->mpc_pixel_y = {};

    for (int i = 0; i < _mpc_steps; i++) 
    {
        this->mpc_pixel_x.push_back(solution.x[_pixel_x_start + i]);
        this->mpc_pixel_y.push_back(solution.x[_pixel_y_start + i]);
        //std::cout<<"joint-1 velocity "<<solution.x.size()<<std::endl;

    }
    // for(int i=0;i<n_vars;i++){
    //     std::cout<<"var "<<vars[i]<<std::endl;
    // }
    // for(int i=0;i<solution.x.size();i++){
    //     std::cout<<"x varaibles "<<solution.x[i]<<std::endl;
    // }
    
    vector<float> result;
    result.push_back(solution.x[_j1_vel_start]);
    //result.push_back(solution.x[_j1_vel_start+1]);
    result.push_back(solution.x[_j2_vel_start]);
    //result.push_back(solution.x[_j2_vel_start+1]);
    std::cout<<"j1vel "<<result[0]<<std::endl;
    std::cout<<"j2vel "<<result[1]<<std::endl;
    return result;

}




