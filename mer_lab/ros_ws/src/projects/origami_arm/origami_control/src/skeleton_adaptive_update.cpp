#include "ros/ros.h"
#include "origami_control/skeleton_adaptive_update.h"
#include <eigen3/Eigen/Dense>

int window;
float eps;
int num_features;
int num_actuators;

bool update(origami_control::skeleton_adaptive_update::Request &req,
            origami_control::skeleton_adaptive_update::Response &res){

    // std::cout<<"computing Jacobian update"<<std::endl;
    // Assign request data
    float gamma = req.gamma;
    int it = req.it;    
    std_msgs::Float64MultiArray dS = req.dS;
    std_msgs::Float64MultiArray dR = req.dR;
    std_msgs::Float64MultiArray qhat = req.qhat;

    // Convert ROS MSG Arrays to Eigen Matrices
    
    //dS
    std::vector<double> dSdata = dS.data;
    // std::cout<<"size of dSdata: " << dSdata.size()<<std::endl;
    // Declare dS matrix
    Eigen::MatrixXf dSmat(window,num_features);
    
    // Push data to dS matrix
    int row_count = 0;
    int itr = 0;
    Eigen::VectorXf temp_row_ds(num_features);
    while(row_count < window){
        for(int i=0; i<num_features; i++){
            temp_row_ds[i] = dSdata[itr];
            itr = itr + 1;
        }
        dSmat.row(row_count) << temp_row_ds.transpose();
        row_count = row_count + 1;
    }          
        // std::cout<<"Pushing dS data to row:"<<row_count<<std::endl;
        // itr = itr+num_features;
        // dSmat.row(row_count) << dSdata[itr], dSdata[itr+1], dSdata[itr+2], dSdata[itr+3], dSdata[itr+4], dSdata[itr+5];

    //dR
    std::vector<double> dRdata = dR.data;
    // Declare dR matrix
    Eigen::MatrixXf dRmat(window,num_actuators);
    // Push data to dR matrix
    row_count = 0;
    itr = 0;
    Eigen::VectorXf temp_row_dr(num_actuators);
    while(row_count < window){
        for(int i=0; i<num_actuators; i++){
            temp_row_dr[i] = dRdata[itr];
            itr = itr + 1;
        }
        dRmat.row(row_count) << temp_row_dr.transpose();
        row_count = row_count + 1;
    }               
        // dRmat.row(row_count) << dRdata[itr], dRdata[itr+1], dRdata[itr+2], dRdata[itr+3];
        // std::cout<<"Pushing dR data to row:"<<row_count<<std::endl;
        // itr = itr+num_actuators;
    // std::cout<<"Size of dRMat: "<<dRmat.rows()<<","<<dRmat.cols()<<std::endl;

    //qhat
    std::vector<double> qhatdata = qhat.data;
    // Declare qhat matrix
    Eigen::MatrixXf qhatMat(num_features,num_actuators);
    // Push data to qhat matrix
    row_count = 0;
    itr = 0;
    Eigen::VectorXf temp_row_qhat(num_actuators);
    while(row_count < num_features){
        for(int i=0; i<num_actuators; i++){
            temp_row_qhat[i] = qhatdata[itr];
            itr = itr + 1;
            // std::cout<<"pushing: "<<qhatdata[itr];
        }
        // std::cout<<std::endl;
        qhatMat.row(row_count) << temp_row_qhat.transpose();
        row_count = row_count + 1;
    }
        // qhatMat.row(row_count) << qhatdata[itr], qhatdata[itr+1], qhatdata[itr+2], qhatdata[itr+3];
        // std::cout<<"Pushing qhat data to row:"<<row_count<<std::endl;
        // itr = itr + num_actuators;
    // std::cout<<"Size of qhat: "<<qhatMat.rows()<<","<<qhatMat.cols()<<std::endl;
    // std::cout<<"Converted request data to ROS Msg"<<std::endl;

    // Compute Energy Functional
    Eigen::MatrixXf Ji = Eigen::MatrixXf::Zero(1,dSmat.cols());
    // std::cout<<"Declared Ji"<<std::endl;
    
    for(int i=0; i<dSmat.cols();i++){
        // std::cout<<dSmat(it,i)<<std::endl;
        
        // std::cout<<"dr row: " << dRmat.row(it)<<std::endl;
        // std::cout<<"qhat row:"<<qhatMat.row(i)<<std::endl;
        // std::cout<<std::endl;

        float cur_model_err = pow((dRmat.row(it)*qhatMat.row(i).transpose() - dSmat(it,i)),2);
        // std::cout<<"current model error:"<<cur_model_err<<std::endl;
        
        float old_err = pow((dRmat*qhatMat.row(i).transpose() - dSmat.col(i)).norm(),2);
        // std::cout<<"old err:"<<old_err<<std::endl;
        Ji(i) = (cur_model_err + old_err)/2;
        // std::cout<<"Ji:"<<Ji(i)<<std::endl;
    }
    // std::cout<<"computed energy functional"<<std::endl;

    // Updated Jacobian Vectors
    for(int i=0; i<dSmat.cols();i++){
        if(Ji(i) > eps){    // Update Jacobian if error greater than convergence threshold
            Eigen::MatrixXf G1 = dRmat*(qhatMat.row(i).transpose()) - dSmat.col(i);
            // std::cout<<"Size of G1:"<<G1.rows()<<","<<G1.cols()<<std::endl;
            float G2 = dRmat.row(it)*(qhatMat.row(i).transpose()) - dSmat(it,i);
            // std::cout<<"G2:"<<G2<<std::endl;
            Eigen::MatrixXf G ((G1.rows()+1),1);
            G << G1,
                 G2;
            // std::cout<<"Size of G:"<<G.rows()<<","<<G.cols()<<std::endl;
            Eigen::MatrixXf H1 (num_actuators,(window+1));
            H1 << dRmat.transpose(), dRmat.row(it).transpose();
            // std::cout<<"Size of H1:"<<H1.rows()<<","<<H1.cols()<<std::endl;
            Eigen::MatrixXf H = H1.transpose(); 
            qhatMat.row(i) = (-gamma*(H.transpose())*G).transpose();
        }
    }

    // std::cout<<"updated Jacobian vectors"<<std::endl;

    // Convert Eigen::Matrix to ROS MSG Array
        // Declare vector to store qhatMat elements
        std::vector<double> qhatMatVector;

        // Convert matrix to vector
        qhatMatVector.clear();
        for(int i = 0; i<qhatMat.rows(); i++){
            for(int j=0; j<qhatMat.cols(); j++){
                qhatMatVector.push_back(qhatMat(i,j));
            }
        }
            // qhatMatVector.push_back(qhatMat(i,0));
            // qhatMatVector.push_back(qhatMat(i,1));
            // qhatMatVector.push_back(qhatMat(i,2));
            // qhatMatVector.push_back(qhatMat(i,3));
        // std::cout<<"converted qhatmat to vector"<<std::endl;

        // Declare ROS Msg Array
        std_msgs::Float64MultiArray qhat_dotmsg;
        qhat_dotmsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        qhat_dotmsg.layout.dim[0].label = "qhat_elements";
        qhat_dotmsg.layout.dim[0].size = qhatMatVector.size();
        qhat_dotmsg.layout.dim[0].stride = 1;
        qhat_dotmsg.data.clear();
        // std::cout<<"Declared ROS msg"<<std::endl;

        // Push data to ROS Msg Array
        for(std::vector<double>::iterator itr = qhatMatVector.begin(); itr != qhatMatVector.end(); ++itr){
            // std::cout <<*itr<<std::endl;
            qhat_dotmsg.data.push_back(*itr);
        }
        // std::cout<<"Qhat converted to ROS msg"<<std::endl;

    // Jacobian response msg
    res.qhat_dot = qhat_dotmsg;

    // std::cout<<"qhat_dot"<<qhat_dotmsg<<"\n";
    
    // Sum of Jis
    float J = 0.0;
    // std::cout<<"Ji";
    for(int i=0; i<dSmat.cols();i++){
        J = J + Ji(i);
        // std::cout<<Ji(i)<<",";
    }
    // std::cout<<std::endl;
    // Assign Response data
    res.J = J;
    // std::cout<<"J Response "<<J<<"\n";

    return true;
    }


int main(int argc, char **argv){
    ros::init(argc, argv, "adaptive_updates");
    ros::NodeHandle n;

    n.getParam("origami_skeleton_vs/window_size", window);
    n.getParam("origami_skeleton_vs/epsilon", eps);
    n.getParam("origami_skeleton_vs/no_of_features", num_features);
    n.getParam("origami_skeleton_vs/no_of_actuators", num_actuators);

    // Service server
    ros::ServiceServer update_service =  n.advertiseService("origami_skeleton_adaptive_update", update);
    std::cout<<"Update node setup"<<std::endl;
    ros::spin();
    return 0;
}