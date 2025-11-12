#include "MPC.h"

using CppAD::AD;
using Eigen::Matrix;
using Eigen::Dynamic;

typedef Matrix< AD<double>, Dynamic,Dynamic > eg_matrix;
typedef Matrix< AD<double>, Dynamic,1 > eg_vector;
eg_matrix jacobian_robot(3,3);// Robot Jacobian
eg_vector input_vector(3);  // Input vector[w1,w2,w3] at K step
eg_vector pixel_st_init(3); // Pixel state vector time K step
eg_vector pixel_st_out(3);  // Pixel state vector time K+1 step
eg_vector cable_len_st_init(3); // Cable lengths at time K step
eg_vector cable_len_out(3);  // Cable lengths at K+1 step


/* Class */
class FG_eval
{
    public:
    int _pixel_x_start, _pixel_y_start,_pixel_z_start, _cable_l1_start,_cable_l2_start,_cable_l3_start; // state variables index
    int _motor1_vel_start,_motor2_vel_start,_motor3_vel_start; //control output variables Index
    /** MPC Parameters **/
    int _mpc_steps; // Prediction Step size
    double _dt; // Sample time
    double _ref_pixel_pose_x, _ref_pixel_pose_y,_ref_pixel_pose_z; // Reference Pose
    double  _w_pixel_error_x,_w_pixel_error_y,_w_pixel_error_z,_w_cable_1_accl,_w_cable_2_accl,_w_cable_3_accl; // Weights for cost function; _w_j1_vel,_w_j2_vel
    /** Camera Parameters **/
    double focus; // Camera Focus
    double Z;  // depth-2
    double cx_intrinsic; //principle point centre offset-X
    double cy_intrinsic; //principle point centre offset-Y
    /** Robot Parameters **/
    double module_d; //Centre distance length
    double num_module; //Number of Modules

    AD<double> cost_pixel_error_x,cost_pixel_error_y,cost_pixel_error_z,cost_cable_1_accl,cost_cable_2_accl,cost_cable_3_accl;
    AD<double> limit_12=80;
    AD<double> limit_3=81;
    //constructor
    FG_eval(unordered_map<string,double>& param_fg) 
    { 
        // Set default value
        // MPC params    
        _mpc_steps           = param_fg["mpc_steps"];
        _dt                  = param_fg["mpc_dt"];  // in sec
        _ref_pixel_pose_x    = param_fg["goal_pos_x"];
        _ref_pixel_pose_y    = param_fg["goal_pos_y"]; 
        _ref_pixel_pose_z    = param_fg["goal_pos_z"]; 
        _w_pixel_error_x     = param_fg["w_pixel_x"];
        _w_pixel_error_y     = param_fg["w_pixel_y"];
        _w_pixel_error_z     = param_fg["w_pixel_z"];
        _w_cable_1_accl      = param_fg["w_cable_1_accl"];
        _w_cable_2_accl      = param_fg["w_cable_2_accl"];
        _w_cable_3_accl      = param_fg["w_cable_3_accl"];
        // _w_cable_1_vel       = param_fg["w_cable_1_vel"];
        // _w_cable_2_vel       = param_fg["w_cable_2_vel"];
        // _w_cable_3_vel       = param_fg["w_cable_3_vel"];
        // Camera params 
        focus                = param_fg["focus"];
        Z                    = param_fg["depth"];
        cx_intrinsic         = param_fg["cx_intrinsic"];
        cy_intrinsic         = param_fg["cy_intrinsic"];
        // Robot params 
        module_d             = param_fg["module_d"];
        num_module           = param_fg["num_of_modules"];
        
        _pixel_x_start         = 0;
        _pixel_y_start         = _pixel_x_start + _mpc_steps;
        _pixel_z_start         = _pixel_y_start + _mpc_steps;
        _cable_l1_start        = _pixel_z_start + _mpc_steps;
        _cable_l2_start        = _cable_l1_start + _mpc_steps;
        _cable_l3_start        = _cable_l2_start + _mpc_steps;
        _motor1_vel_start      = _cable_l3_start + _mpc_steps;
        _motor2_vel_start      = _motor1_vel_start + _mpc_steps;
        _motor3_vel_start      = _motor2_vel_start + _mpc_steps - 1;
    }

// void avoid_singularity(AD<double>& l2, AD<double>& l3){
//     if(abs(l2 - l3) < 0.1){
//         l2 =  l2 + 0.2;}
// }
    int update_robot_jacobian(eg_matrix& jacobian_robot, AD<double> l1, AD<double> l2, AD<double> l3, AD<double> d){
        // avoid_singularity(l2,l3);
        l1 = std::max(l1,limit_12);
        l2 = std::max(l2,limit_12);
        l3 = std::max(l3,limit_3);

        jacobian_robot << (1.0/3.0)*d*(-2*l1 + l2 + l3)*(l1 + l2 + l3)*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::pow(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2), 3.0/2.0)*CppAD::pow(l2 - l3, 2)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) + (2*(l1 - 1.0/2.0*l2 - 1.0/2.0*l3)/(d*(l1 + l2 + l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) - 2*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*CppAD::pow(l1 + l2 + l3, 2)))*((1.0/2.0)*d*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)*CppAD::cos(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2))*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) + (-1.0/4.0*CppAD::pow(d, 2)*CppAD::pow(l1 + l2 + l3, 2)*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)) + (1.0/4.0)*d*(CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1)*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)/CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)))*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2))) + (1.0/3.0)*((1.0/2.0)*CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1.0/2.0)*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2)) + (1.0/3.0)*CppAD::cos(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2)),
-1.0/6.0*CppAD::sqrt(3)*d*((1.0/3.0)*CppAD::sqrt(3)/(l2 - l3) - 1.0/3.0*CppAD::sqrt(3)*(-2*l1 + l2 + l3)/CppAD::pow(l2 - l3, 2))*(-2*l1 + l2 + l3)*(l1 + l2 + l3)*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::pow(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2), 3.0/2.0)*(l2 - l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) + (2*(-1.0/2.0*l1 + l2 - 1.0/2.0*l3)/(d*(l1 + l2 + l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) - 2*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*CppAD::pow(l1 + l2 + l3, 2)))*((1.0/2.0)*d*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)*CppAD::cos(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2))*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) + (-1.0/4.0*CppAD::pow(d, 2)*CppAD::pow(l1 + l2 + l3, 2)*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)) + (1.0/4.0)*d*(CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1)*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)/CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)))*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2))) + (1.0/3.0)*((1.0/2.0)*CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1.0/2.0)*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2)) + (1.0/3.0)*CppAD::cos(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2)),
-1.0/6.0*CppAD::sqrt(3)*d*((1.0/3.0)*CppAD::sqrt(3)/(l2 - l3) + (1.0/3.0)*CppAD::sqrt(3)*(-2*l1 + l2 + l3)/CppAD::pow(l2 - l3, 2))*(-2*l1 + l2 + l3)*(l1 + l2 + l3)*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::pow(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2), 3.0/2.0)*(l2 - l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) + (2*(-1.0/2.0*l1 - 1.0/2.0*l2 + l3)/(d*(l1 + l2 + l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) - 2*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*CppAD::pow(l1 + l2 + l3, 2)))*((1.0/2.0)*d*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)*CppAD::cos(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2))*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) + (-1.0/4.0*CppAD::pow(d, 2)*CppAD::pow(l1 + l2 + l3, 2)*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)) + (1.0/4.0)*d*(CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1)*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)/CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)))*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2))) + (1.0/3.0)*((1.0/2.0)*CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1.0/2.0)*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2)) + (1.0/3.0)*CppAD::cos(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2)),
-1.0/3.0*CppAD::sqrt(3)*d*(l1 + l2 + l3)*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::pow(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2), 3.0/2.0)*(l2 - l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) + (2*(l1 - 1.0/2.0*l2 - 1.0/2.0*l3)/(d*(l1 + l2 + l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) - 2*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*CppAD::pow(l1 + l2 + l3, 2)))*((1.0/6.0)*CppAD::sqrt(3)*d*(-2*l1 + l2 + l3)*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)*CppAD::cos(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2))*(l2 - l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) + (1.0/3.0)*CppAD::sqrt(3)*(-1.0/4.0*CppAD::pow(d, 2)*CppAD::pow(l1 + l2 + l3, 2)*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)) + (1.0/4.0)*d*(CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1)*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)/CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)))*(-2*l1 + l2 + l3)*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2))*(l2 - l3))) + (1.0/9.0)*CppAD::sqrt(3)*((1.0/2.0)*CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1.0/2.0)*(-2*l1 + l2 + l3)*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2))*(l2 - l3)) + (1.0/9.0)*CppAD::sqrt(3)*(-2*l1 + l2 + l3)*CppAD::cos(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2))*(l2 - l3)),
(1.0/2.0)*d*((1.0/3.0)*CppAD::sqrt(3)/(l2 - l3) - 1.0/3.0*CppAD::sqrt(3)*(-2*l1 + l2 + l3)/CppAD::pow(l2 - l3, 2))*(l1 + l2 + l3)*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::pow(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2), 3.0/2.0)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) + (2*(-1.0/2.0*l1 + l2 - 1.0/2.0*l3)/(d*(l1 + l2 + l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) - 2*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*CppAD::pow(l1 + l2 + l3, 2)))*((1.0/6.0)*CppAD::sqrt(3)*d*(-2*l1 + l2 + l3)*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)*CppAD::cos(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2))*(l2 - l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) + (1.0/3.0)*CppAD::sqrt(3)*(-1.0/4.0*CppAD::pow(d, 2)*CppAD::pow(l1 + l2 + l3, 2)*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)) + (1.0/4.0)*d*(CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1)*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)/CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)))*(-2*l1 + l2 + l3)*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2))*(l2 - l3))) + (1.0/9.0)*CppAD::sqrt(3)*((1.0/2.0)*CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1.0/2.0)*(-2*l1 + l2 + l3)*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2))*(l2 - l3)) + (1.0/9.0)*CppAD::sqrt(3)*(-2*l1 + l2 + l3)*CppAD::cos(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2))*(l2 - l3)),
(1.0/2.0)*d*((1.0/3.0)*CppAD::sqrt(3)/(l2 - l3) + (1.0/3.0)*CppAD::sqrt(3)*(-2*l1 + l2 + l3)/CppAD::pow(l2 - l3, 2))*(l1 + l2 + l3)*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::pow(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2), 3.0/2.0)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) + (2*(-1.0/2.0*l1 - 1.0/2.0*l2 + l3)/(d*(l1 + l2 + l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) - 2*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*CppAD::pow(l1 + l2 + l3, 2)))*((1.0/6.0)*CppAD::sqrt(3)*d*(-2*l1 + l2 + l3)*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)*CppAD::cos(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2))*(l2 - l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) + (1.0/3.0)*CppAD::sqrt(3)*(-1.0/4.0*CppAD::pow(d, 2)*CppAD::pow(l1 + l2 + l3, 2)*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)) + (1.0/4.0)*d*(CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1)*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)/CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)))*(-2*l1 + l2 + l3)*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2))*(l2 - l3))) + (1.0/9.0)*CppAD::sqrt(3)*((1.0/2.0)*CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1.0/2.0)*(-2*l1 + l2 + l3)*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2))*(l2 - l3)) + (1.0/9.0)*CppAD::sqrt(3)*(-2*l1 + l2 + l3)*CppAD::cos(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::sqrt(1 + (1.0/3.0)*CppAD::pow(-2*l1 + l2 + l3, 2)/CppAD::pow(l2 - l3, 2))*(l2 - l3)),
(2*(l1 - 1.0/2.0*l2 - 1.0/2.0*l3)/(d*(l1 + l2 + l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) - 2*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*CppAD::pow(l1 + l2 + l3, 2)))*(-1.0/4.0*CppAD::pow(d, 2)*CppAD::pow(l1 + l2 + l3, 2)*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)) + (1.0/4.0)*d*(CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1)*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)/CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)) - 1.0/2.0*d*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)) + (-1.0/4.0*CppAD::pow(d, 2)*CppAD::pow(l1 + l2 + l3, 2)*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)) + (1.0/4.0)*d*(CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1)*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)/CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)))*CppAD::cos(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))) + (1.0/3.0)*((1.0/2.0)*CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1.0/2.0)*CppAD::cos(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))) - 1.0/3.0*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))) + (1.0/6.0)*CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1.0/6.0,
(2*(-1.0/2.0*l1 + l2 - 1.0/2.0*l3)/(d*(l1 + l2 + l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) - 2*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*CppAD::pow(l1 + l2 + l3, 2)))*(-1.0/4.0*CppAD::pow(d, 2)*CppAD::pow(l1 + l2 + l3, 2)*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)) + (1.0/4.0)*d*(CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1)*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)/CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)) - 1.0/2.0*d*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)) + (-1.0/4.0*CppAD::pow(d, 2)*CppAD::pow(l1 + l2 + l3, 2)*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)) + (1.0/4.0)*d*(CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1)*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)/CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)))*CppAD::cos(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))) + (1.0/3.0)*((1.0/2.0)*CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1.0/2.0)*CppAD::cos(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))) - 1.0/3.0*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))) + (1.0/6.0)*CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1.0/6.0,
(2*(-1.0/2.0*l1 - 1.0/2.0*l2 + l3)/(d*(l1 + l2 + l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))) - 2*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*CppAD::pow(l1 + l2 + l3, 2)))*(-1.0/4.0*CppAD::pow(d, 2)*CppAD::pow(l1 + l2 + l3, 2)*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)) + (1.0/4.0)*d*(CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1)*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)/CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)) - 1.0/2.0*d*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)) + (-1.0/4.0*CppAD::pow(d, 2)*CppAD::pow(l1 + l2 + l3, 2)*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))/(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)) + (1.0/4.0)*d*(CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1)*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*(l1 + l2 + l3)/CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2)))*CppAD::cos(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))) + (1.0/3.0)*((1.0/2.0)*CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1.0/2.0)*CppAD::cos(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))) - 1.0/3.0*CppAD::sin(2*((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3)))*CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))) + (1.0/6.0)*CppAD::pow(CppAD::tan(((1.0/3.0)*l1 + (1.0/3.0)*l2 + (1.0/3.0)*l3)*CppAD::sqrt(CppAD::pow(l1, 2) - l1*l2 - l1*l3 + CppAD::pow(l2, 2) - l2*l3 + CppAD::pow(l3, 2))/(d*(l1 + l2 + l3))), 2) + 1.0/6.0;

    return 0;
    }

    /* MPC implementation (cost func & constraints) */
    // fg: function that evaluates the objective and constraints using the syntax
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector& fg, const ADvector& vars) 
    {
        /************************ fg[0] for cost function **************************************/ 
        fg[0] = 0;
        cost_pixel_error_x =  0;
        cost_pixel_error_y =  0;
        cost_pixel_error_z =  0;
        for (int i = 0; i < _mpc_steps; i++) 
        {
            fg[0] += _w_pixel_error_x * CppAD::pow(vars[_pixel_x_start + i] - _ref_pixel_pose_x, 2); // Pixel Frame error
            fg[0] += _w_pixel_error_y * CppAD::pow(vars[_pixel_y_start + i] - _ref_pixel_pose_y, 2); // Pixel Frame error
            fg[0] += _w_pixel_error_z * CppAD::pow(vars[_pixel_z_start + i] - _ref_pixel_pose_z, 2); // Pixel Frame error
            cost_pixel_error_x +=  _w_pixel_error_x * CppAD::pow(vars[_pixel_x_start + i] - _ref_pixel_pose_x, 2);
            cost_pixel_error_y +=  _w_pixel_error_y * CppAD::pow(vars[_pixel_y_start + i] - _ref_pixel_pose_y, 2);
        }
        std::cout << "-----------------------------------------------" <<std::endl;
        std::cout << "cost_pixel =(x,y): " << cost_pixel_error_x << ", " << cost_pixel_error_y << std::endl;

        // std::cout << "Mpc steps" << _mpc_steps<<std::endl;

        /* Minimize the use of actuators. */
        // for (int i = 0; i < _mpc_steps - 1; i++) 
        // {
        //     fg[0] += _w_j1_vel * CppAD::pow(vars[_j1_vel_start + i], 2);
        //     fg[0] += _w_j2_vel * CppAD::pow(vars[_j2_vel_start + i], 2);
        //     cost_j1_vel += _w_j1_vel * CppAD::pow(vars[_j1_vel_start + i], 2);
        //     cost_j2_vel += _w_j2_vel * CppAD::pow(vars[_j2_vel_start + i], 2);
        //     //cout << "velocity  Cost: " << cost_j1_vel <<", "<< cost_j2_vel<< endl;
        // }
        //cout << "Total Cost: " << fg[0] << endl; 

        /* Minimize Cable accelerations: the value gap between sequential actuations. */
        // for (int i = 0; i < _mpc_steps - 2; i++) 
        // {
        //     fg[0] += _w_cable_1_accl * CppAD::pow(vars[_motor1_vel_start + i + 1] - vars[_motor1_vel_start + i], 2);
        //     fg[0] += _w_cable_2_accl * CppAD::pow(vars[_motor2_vel_start + i + 1] - vars[_motor2_vel_start + i], 2);
        //     fg[0] += _w_cable_3_accl * CppAD::pow(vars[_motor3_vel_start + i + 1] - vars[_motor3_vel_start + i], 2);
        // }
        // std::cout << "cost of gap: " << fg[0] << std::endl; 
        

        /************************ fg[x] for constraints **************************************/ 
        // Initial constraints
        fg[1 + _pixel_x_start]   = vars[_pixel_x_start];
        fg[1 + _pixel_y_start]   = vars[_pixel_y_start];
        fg[1 + _pixel_z_start]   = vars[_pixel_z_start];
        fg[1 + _cable_l1_start]  = vars[_cable_l1_start];
        fg[1 + _cable_l2_start]  = vars[_cable_l2_start];
        fg[1 + _cable_l3_start]  = vars[_cable_l3_start];
        // Add system dynamic model constraint
        for (int i = 0; i < _mpc_steps - 1; i++)
        {
            // The state at time t+1 .
            AD<double> pixel_x1 = vars[_pixel_x_start + i + 1];
            AD<double> pixel_y1 = vars[_pixel_y_start + i + 1];
            AD<double> pixel_z1 = vars[_pixel_z_start + i + 1];
            AD<double> cl1_1    = vars[_cable_l1_start + i + 1];
            AD<double> cl2_1    = vars[_cable_l2_start + i + 1];
            AD<double> cl3_1    = vars[_cable_l3_start + i + 1];

            // The state at time t.
            AD<double> pixel_x0 = vars[_pixel_x_start + i];
            AD<double> pixel_y0 = vars[_pixel_y_start + i];
            AD<double> pixel_z0 = vars[_pixel_z_start + i];
            AD<double> cl1_0    = vars[_cable_l1_start + i];
            AD<double> cl2_0    = vars[_cable_l2_start + i];
            AD<double> cl3_0    = vars[_cable_l3_start + i];

            // Only consider the actuation at time t.
            AD<double> motor1_vel_0 = vars[_motor1_vel_start + i];
            AD<double> motor2_vel_0 = vars[_motor2_vel_start + i];
            AD<double> motor3_vel_0 = vars[_motor3_vel_start + i];
            // Here's `x` to get you started.
            // The idea here is to constraint this value to be 0
            // NOTE: The use of `AD<double>` and use of `CppAD`!
            // This is also CppAD can compute derivatives and pass
            // these to the solver.
            // TODO: Setup the rest of the model constraints

            // jacobian_robot << ((l1*CppAD::cos(theta1_0))+(l2*CppAD::cos(theta1_0+theta2_0))), ((l2*CppAD::cos(theta1_0+theta2_0))),
            // ((l1*CppAD::sin(theta1_0))+(l2*CppAD::sin(theta1_0+theta2_0))) ,((l2*CppAD::sin(theta1_0+theta2_0)));
            update_robot_jacobian(jacobian_robot, cl1_0, cl2_0, cl3_0, 40);
            
            input_vector  << motor1_vel_0 , motor2_vel_0, motor3_vel_0;
            pixel_st_init << pixel_x0, pixel_z0, pixel_y0;
            
            cable_len_st_init << cl1_0 , cl2_0 , cl3_0;
            
            pixel_st_out = pixel_st_init + ((jacobian_robot * input_vector) * _dt) * 0.92;
            cable_len_out = cable_len_st_init + ((input_vector*0.01) * _dt);

            fg[2 + _pixel_x_start + i]   = pixel_x1 - pixel_st_out[0];
            fg[2 + _pixel_z_start + i]   = pixel_z1 - pixel_st_out[1];
            fg[2 + _pixel_y_start + i]   = pixel_y1 - pixel_st_out[2];
            fg[2 + _cable_l1_start + i]  = cl1_1 - cable_len_out[0];
            fg[2 + _cable_l2_start + i]  = cl2_1 - cable_len_out[1]; 
            fg[2 + _cable_l3_start + i]  = cl3_1 - cable_len_out[2];
            // std::cout << "check point " << std::endl; 

        }
                  
    }

};

// ====================================
// MPC class definition implementation.
// ====================================
MPC::MPC(std::unordered_map<string,double>& param) 
{
    // Set default value    
    _mpc_steps             = param["mpc_steps"];
    _pixel_x_lb            = param["pixel_x_lb"];
    _pixel_x_ub            = param["pixel_x_ub"];
    _pixel_y_lb            = param["pixel_y_lb"];
    _pixel_y_ub            = param["pixel_y_ub"];
    _cable_lengths_lb      = param["cable_lengths_lb"];
    _cable_lengths_ub      = param["cable_lengths_ub"];
    _motor_vel_lb          = param["motor_vel_lb"];
    _motor_vel_ub          = param["motor_vel_ub"];

    _pixel_x_start         = 0;
    _pixel_y_start         = _pixel_x_start + _mpc_steps;
    _pixel_z_start         = _pixel_y_start + _mpc_steps;
    _cable_l1_start        = _pixel_z_start + _mpc_steps;
    _cable_l2_start        = _cable_l1_start + _mpc_steps;
    _cable_l3_start        = _cable_l2_start + _mpc_steps;
    _motor1_vel_start      = _cable_l3_start + _mpc_steps;
    _motor2_vel_start      = _motor1_vel_start + _mpc_steps;
    _motor3_vel_start      = _motor2_vel_start + _mpc_steps - 1;
    param_mpc = param; 

}


vector<float> MPC::Solve(Eigen::VectorXf state) 
{
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    /* Set the number of model variables (includes both states and inputs)
     State Variables = [px,py,pz,l_1,l_2,l_3]
     Actuator variables = [w_1,w_2,w_3] */
    size_t n_vars = _mpc_steps * 6 + (_mpc_steps) * 3;
    /****** Set the number of constraints *******/
    /* Pixel model Constraints(px,py) = 2 
    cable lengths Constraints(l1,l2,l3) = 3 
    */
    size_t n_constraints = _mpc_steps * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) 
    {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[_pixel_x_start]     = state[0];
    vars[_pixel_y_start]     = state[1];
    vars[_pixel_z_start]     = state[2];
    vars[_cable_l1_start]    = state[3];
    vars[_cable_l2_start]    = state[4];
    vars[_cable_l3_start]    = state[5];//0.001
    vars[_motor1_vel_start]  = state[6];//0.001
    vars[_motor2_vel_start]  = state[7];//0.001
    vars[_motor3_vel_start]  = state[8];//0.001
    

    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);


    for (int i = _pixel_x_start; i < _pixel_y_start; i++) 
    {
        vars_lowerbound[i] = _pixel_x_lb;
        vars_upperbound[i] = _pixel_x_ub;
    }
    for (int i = _pixel_y_start; i < _pixel_z_start; i++) 
    {
        vars_lowerbound[i] = _pixel_y_lb;
        vars_upperbound[i] = _pixel_y_ub;
    }
    for (int i = _cable_l1_start; i < _motor1_vel_start; i++) 
    {
        vars_lowerbound[i] = _cable_lengths_lb;
        vars_upperbound[i] = _cable_lengths_ub;
    }
    for (int i = _motor1_vel_start; i < n_vars; i++)  
    {
        vars_lowerbound[i] = _motor_vel_lb;
        vars_upperbound[i] = _motor_vel_ub;
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

    constraints_lowerbound[_pixel_x_start]   = state[0];
    constraints_lowerbound[_pixel_y_start]   = state[1];
    constraints_lowerbound[_pixel_z_start]   = state[2];
    constraints_lowerbound[_cable_l1_start]  = state[3];
    constraints_lowerbound[_cable_l2_start]  = state[4];
    constraints_lowerbound[_cable_l3_start]  = state[5];
    
    
    constraints_upperbound[_pixel_x_start]   = state[0];
    constraints_upperbound[_pixel_y_start]   = state[1];
    constraints_lowerbound[_pixel_z_start]   = state[2];
    constraints_upperbound[_cable_l1_start]  = state[3];
    constraints_upperbound[_cable_l2_start]  = state[4];
    constraints_upperbound[_cable_l3_start]  = state[5];

    // object that computes objective and constraints
    FG_eval fg_eval(param_mpc);
    std::cout << "outer point " << std::endl; 
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
    options += "Numeric max_cpu_time          2\n";

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
    // _mpc_cost_j1_vel = Value(fg_eval.cost_j1_vel);
    // _mpc_cost_j2_vel = Value(fg_eval.cost_j2_vel);

    // this->mpc_pixel_x = {};
    // this->mpc_pixel_y = {};

    // for (int i = 0; i < _mpc_steps; i++) 
    // {
    //     this->mpc_pixel_x.push_back(solution.x[_pixel_x_start + i]);
    //     this->mpc_pixel_y.push_back(solution.x[_pixel_y_start + i]);
    // }

    
    vector<float> result;
    result.push_back(solution.x[_motor1_vel_start]);
    result.push_back(solution.x[_motor2_vel_start]);
    result.push_back(solution.x[_motor3_vel_start]);
    std::cout<<"motor1_vel "<<result[0]<<std::endl;
    std::cout<<"motor2_vel "<<result[1]<<std::endl;
    std::cout<<"motor3_vel "<<result[2]<<std::endl;
    return result;

}


/* ROS -Call back functions*/

std::vector<float> cur_ee_pose;  // Feature cordinates in Pixel frame
std::vector<float> goal_ee_pose;  // Feature cordinates in Pixel frame
std::vector<float> cur_cable_length;
std_msgs::Bool end_flag; // Terminate Control loop
bool servo = false;

/***Callback for End Effector Marker Point Topic***/
void ee_feature_callback(const std_msgs::Float64MultiArray &msg){
    cur_ee_pose.clear();
    for(int i=0;i<3;i++)
    {
        cur_ee_pose.push_back(msg.data.at(i));
    }
    servo = true;
}

/***Callback for Joint angles Topic***/
void cableLengthCallback(const std_msgs::Float64MultiArray &msg){
    for(int i=0; i<3; i++)
    {
        cur_cable_length.push_back(msg.data.at(i));
    }
}

/***Callback for stop Control loop ***/
void controlLoopEndflagCallback(const std_msgs::Bool msg){
     end_flag = msg;
}

/***Callback for Goal Pose ***/
void refCallback(const std_msgs::Float64MultiArray &msg){
    goal_ee_pose.clear();
    for(int i=0; i<3; i++){
        goal_ee_pose.push_back(msg.data.at(i));
    }
}

int main(int argc, char **argv){

    // ROS initialization
    ros::init(argc, argv, "origami_mpc_controller");
    ros::NodeHandle n;
    
    // Initializing ROS publishers
    ros::Publisher cable_vel_pub = n.advertise<std_msgs::Float64MultiArray>("/origami_vs/vel",1);
    ros::Publisher err_pub = n.advertise<std_msgs::Float64MultiArray>("/origami_vs_mpc/servoing_error", 1);
    ros::Publisher end_flag_pub = n.advertise<std_msgs::Bool>("/origami_vs_mpc/control_loop_end_flag", 1);

    // Subscriber Node for End effector feature pose
    ros::Subscriber feature_sub = n.subscribe("origami_vs/features", 1, ee_feature_callback);
    ros::Subscriber goal_sub = n.subscribe("origami_vs/reference", 1, refCallback);
    ros::Subscriber cable_length_sub = n.subscribe("origami_vs/cable_lengths", 1, cableLengthCallback);
    ros::Subscriber end_flag_sub = n.subscribe("/origami_vs_mpc/control_loop_end_flag", 1, controlLoopEndflagCallback);
    // ros::Duration(3).sleep(); 
    
    /** Gazebo Camera Parameters from Intrinsic Matrix **/
    unordered_map<string,double> mpc_param;
    n.getParam("origamibot/gazebo_camera/focus",mpc_param["focus"]); // Focus from Gazebo Camera info
    n.getParam("origamibot/gazebo_camera/cx_intrinsic",mpc_param["cx_intrinsic"]);//principle point centre offset
    n.getParam("origamibot/gazebo_camera/cy_intrinsic",mpc_param["cy_intrinsic"]);//principle point centre offset
    n.getParam("origamibot/gazebo_camera/depth",mpc_param["depth"]);// depth-2
    //float focus_const = focus/Z; // 178.76/1.83

    /** Parameters -Robot **/
    // n.getParam("origami_sim/num_of_modules",mpc_param["num_of_modules"]); // link-1 length

    /** Control loop Parameters **/
    // n.getParam("origamibot/control/goal_pos_x",mpc_param["goal_pos_x"]); //Goal Feature positions  150,136
    // n.getParam("origamibot/control/goal_pos_y",mpc_param["goal_pos_y"]);
    // n.getParam("origamibot/control/goal_pos_z",mpc_param["goal_pos_z"]);
    std::vector<int> target;
    n.getParam("origami_sim/target_feature",target); //Goal Feature positions  150,136
    // n.getParam("origami_sim/control/goal_pos_y",mpc_param["goal_pos_y"]);

    mpc_param["goal_pos_x"] = target[0];
    mpc_param["goal_pos_y"] = target[1];
    mpc_param["goal_pos_z"] = target[2];
    // std::cout<<"waiting to servo "<<target[0]<<std::endl;

    // n.getParam("origami_sim/rate",mpc_param["rate"]); //Control loop Termination Threshold

    /** MPC  Parameters **/
    n.getParam("origamibot/mpc/mpc_steps",mpc_param["mpc_steps"]);
    n.getParam("origamibot/mpc/mpc_sample_time",mpc_param["mpc_dt"]);
    n.getParam("origamibot/mpc/w_pixel_x",mpc_param["w_pixel_x"]);
    n.getParam("origamibot/mpc/w_pixel_y",mpc_param["w_pixel_y"]);
    n.getParam("origamibot/mpc/w_pixel_z",mpc_param["w_pixel_z"]);
    n.getParam("origamibot/mpc/w_cable_1_vel",mpc_param["w_cable_1_vel"]);
    n.getParam("origamibot/mpc/w_cable_2_vel",mpc_param["w_cable_2_vel"]);
    n.getParam("origamibot/mpc/w_cable_3_vel",mpc_param["w_cable_3_vel"]);
    n.getParam("origamibot/mpc/w_cable_1_accl",mpc_param["w_cable_1_accl"]);
    n.getParam("origamibot/mpc/w_cable_2_accl",mpc_param["w_cable_2_accl"]);
    n.getParam("origamibot/mpc/w_cable_3_accl",mpc_param["w_cable_3_accl"]);
    n.getParam("origamibot/mpc/pixel_x_lb",mpc_param["pixel_x_lb"]);
    n.getParam("origamibot/mpc/pixel_x_ub",mpc_param["pixel_x_ub"]);
    n.getParam("origamibot/mpc/pixel_y_lb",mpc_param["pixel_y_lb"]);
    n.getParam("origamibot/mpc/pixel_y_ub",mpc_param["pixel_y_ub"]);
    n.getParam("origamibot/mpc/cable_lengths_lb",mpc_param["cable_lengths_lb"]);
    n.getParam("origamibot/mpc/cable_lengths_ub",mpc_param["cable_lengths_ub"]);
    n.getParam("origamibot/mpc/motor_vel_lb",mpc_param["motor_vel_lb"]);
    n.getParam("origamibot/mpc/motor_vel_ub",mpc_param["motor_vel_ub"]);

    ros::Rate rateController = ros::Rate(10); // 10hz -100ms one loop 
    //Error msg topic
    
    std_msgs::Float64MultiArray err_msg;
    float err = 0;  // Norm of Pixel(x,y) Errors 
    err_msg.data.clear();
    err_msg.data.push_back(err);
    err_pub.publish(err_msg);
    // Declarations
    ros::WallTime start_,end_;
    double execution_time;
    Eigen::VectorXf error_vec(3);
    Eigen::VectorXf cable_vel(3);
    Eigen::MatrixXf rotation_matrix(3,3); // Rotation matrix w.r.t  world frame
    Eigen::MatrixXf k_intrinsic(3,3); // Camera Intrinsic


    /** MPC **/
    Eigen::VectorXf state(9);
    vector<float> mpc_results{0,0,0};


    /** Initialize Zero Joint Velcoities **/
    std_msgs::Float64MultiArray vel_msg;
    vel_msg.data.clear();
    for(int i = 0; i<3; i++){
        vel_msg.data.push_back(0);
    }
    cable_vel_pub.publish(vel_msg);

    MPC _mpc(mpc_param);
    

    while(!servo){
        ros::spinOnce();
        // std::cout<<"waiting to servo"<<std::endl;
        rateController.sleep();
    }

    std::cout<<" Servoing Started "<<std::endl;


    /********** Control loop **************/
    while(int(end_flag.data) == 0){
   
        // update error
        for(int i=0;i<3;i++){ 
            error_vec[i] = (goal_ee_pose[i] - cur_ee_pose[i]);
        }
        err = sqrt((error_vec[0]*error_vec[0]) + (error_vec[1]*error_vec[1]));

        // Publish current error to plot data
        err_msg.data.clear();
        err_msg.data.push_back(err);
        err_pub.publish(err_msg);

        // State update
        state[0] = cur_ee_pose[0]; // Current EE Pose pixel_x
        state[1] = cur_ee_pose[1]; // Current EE Pose pixel_y
        state[2] = 0; // Current EE Pose pixel_z
        state[3] = cur_cable_length[0]; // Cable-1 Length 
        state[4] = cur_cable_length[1]; // Cable-2 Length 
        state[5] = cur_cable_length[2]; // Cable-3 Length 
        state[6] = 0; // Cable-1 velocity 
        state[7] = 0; // Cable-2 velocity 
        state[8] = 0; // Cable-3 velocity 
        // std::cout<<"check point"<<cur_cable_length[0] <<std::endl;
        start_ = ros::WallTime::now();
        // Solve MPC Problem
        mpc_results  = _mpc.Solve(state);

        // Publish Cable velocities
        vel_msg.data.clear();
        for(int i = 0; i<3; i++){
            vel_msg.data.push_back(mpc_results[i]);
        }
        cable_vel_pub.publish(vel_msg);

        end_ = ros::WallTime::now();
        execution_time = (end_ - start_).toNSec() * 1e-6;
        std::cout<<"Exectution time (ms): "<< execution_time<<std::endl;
        
        
        ros::spinOnce();
        rateController.sleep();
    }

    vel_msg.data.clear();
    for(int i = 0; i<3; i++){
        vel_msg.data.push_back(0);
    }
    cable_vel_pub.publish(vel_msg);
    std::cout<<" Servoing Completed "<<std::endl;
    ros::spin();
    return 0;
}


