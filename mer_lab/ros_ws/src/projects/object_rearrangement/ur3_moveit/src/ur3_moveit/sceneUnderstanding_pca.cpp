#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<vector>
#include<geometry_msgs/Pose.h>
#include<string>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include<pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include<ros/package.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl/common/common.h>
#include<ur3_moveit/MoverService.h>
#include<ur3_moveit/MoveRobot.h>
#include<ur3_moveit/getPointCloud.h>
#include<ur3_moveit/resetRobot.h>
#include<pcl/common/pca.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include<ur3_moveit/getrgbimage.h>
// #include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
// #include <opencv2/opencv.hpp>
// #include <Eigen/Quaternion.h>
class sceneUnderstanding_educated{
    public:
    sceneUnderstanding_educated();
    void execute();
    double calcDistance(float x1, float y1,Eigen::Vector4f mean);
    void savergbimage ();
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::ServiceClient gettrajectories;
        ros::ServiceClient moveRobot;
        ros::ServiceClient getCloud;
        ros::ServiceClient resetRobotClient;
        ros::ServiceClient rgbimageclient;
        int counter = 0;
};


sceneUnderstanding_educated::sceneUnderstanding_educated(){
    gettrajectories = nh.serviceClient<ur3_moveit::MoverService>("/ur3_moveit/plantrajectory");
    moveRobot = nh.serviceClient<ur3_moveit::MoveRobot>("/ur3_moveit/MoveRobot");
    getCloud = nh.serviceClient<ur3_moveit::getPointCloud>("/ur3_moveit/pointcloud");
    resetRobotClient = nh.serviceClient<ur3_moveit::resetRobot>("ur3_moveit/ResetRobot");
    rgbimageclient = nh.serviceClient<ur3_moveit::getrgbimage>("/ur3_moveit/getrgbimage");
}

// void sceneUnderstanding_educated::savergbimage() {
//     ur3_moveit::getrgbimage request;
//     ur3_moveit::getrgbimageResponse response;
//     request.request.send = true;
//     rgbimageclient.call(request);
//     cv_bridge::CvImagePtr cv_ptr;
//     sensor_msgs::CompressedImage rgbimage = request.response.rgbimage;
//     cv_ptr = cv_bridge::toCvCopy(rgbimage, "bgr8");
//     cv::Mat current_frame = cv_ptr->image;
//     std::string path = ros::package::getPath("ur3_moveit") + "/output_cloud/Second Phase/img_" + std::to_string(counter) + ".jpg";
//     cv::imwrite(path, current_frame);
//     return;
// }

double sceneUnderstanding_educated::calcDistance(float x1, float y1,Eigen::Vector4f mean){
    return (sqrt((mean(0)-x1)*(mean(0)-x1)+ (mean(1)-y1)*(mean(1)-y1)));
}

void sceneUnderstanding_educated::execute(){
    counter++;
    ros::Duration(3.0).sleep();
    ros::service::waitForService("/ur3_moveit/pointcloud");
    ros::service::waitForService("/ur3_moveit/MoveRobot");
    // std::vector<geometry_msgs::Pose> pose_vector;
    ur3_moveit::getPointCloud cloud_req;
    cloud_req.request.send = true;
    // ur3_moveit::getPointCloudResponse cloud_resp;
    getCloud.call(cloud_req);
    sensor_msgs::PointCloud2 cloud;
    cloud = cloud_req.response.cloud;
    ROS_INFO("Received Cloud");
    ROS_WARN("--------------------------------------------------------");
    // ROS_WARN_STREAM("Cloud: "<<cloud);
    pcl::PCLPointCloud2 temp_cloud;
    pcl_conversions::toPCL(cloud, temp_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(temp_cloud, *pcl_cloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_outlier (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_removal;
    outlier_removal.setInputCloud (pcl_cloud);
    outlier_removal.setMeanK (5);
    outlier_removal.setStddevMulThresh (1.0);
    outlier_removal.filter (*cloud_without_outlier);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (pcl_cloud);
    seg.segment(*inliers, *coefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_table (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (pcl_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_without_table);

    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud_without_outlier);
    seg.segment(*inliers, *coefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_table_or (new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud (cloud_without_outlier);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_without_table_or);

    std::string path = ros::package::getPath("ur3_moveit") + "/output_cloud/Second Phase/";
    pcl::io::savePLYFile(path + "cloud_with_out-Table"+ std::to_string(counter) + ".ply", *cloud_without_table);
    pcl::io::savePLYFile(path + "scenecloud"+  std::to_string(counter) + ".ply", *pcl_cloud);
    pcl::io::savePLYFile(path + "cloud_without_outlier"+  std::to_string(counter) + ".ply", *cloud_without_outlier);

    // DownSampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_table_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_without_outlier);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

    sor.setInputCloud (cloud_without_table_or);
    sor.setLeafSize (0.001f, 0.001f, 0.001f);
    sor.filter (*cloud_without_table_filtered);
    pcl::io::savePLYFile(path + "cloud_without_table_filtered" + std::to_string(counter) + ".ply", *cloud_without_table_filtered);
    pcl::io::savePLYFile(path + "cloud_filtered" +  std::to_string(counter) + ".ply", *cloud_filtered);
    // -------------------------------------------------------------------
    // PCA
    pcl::PCA<pcl::PointXYZ> pca;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pca_input (new pcl::PointCloud<pcl::PointXYZ>);
    *pca_input = *cloud_without_table_filtered;
    for (size_t i = 0; i < pca_input->points.size(); i++){
        pca_input->points[i].z = 0;
    }
    pca.setInputCloud (pca_input);
    Eigen::Vector3f eigen_values = pca.getEigenValues();
    Eigen::Matrix3f eigen_vector = pca.getEigenVectors();
    Eigen::Vector4f mean = pca.getMean();
    ROS_WARN_STREAM("Eigen Values : "<<eigen_values);
    ROS_WARN_STREAM("Eigen vector : "<<eigen_vector);
    ROS_WARN_STREAM("mean : "<<mean);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pca_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pca.project(*pca_input, *pca_cloud);
    pcl::io::savePLYFile(path + "pca_cloud" +  std::to_string(counter) + ".ply", *pca_cloud);

    double n = (-0.2642 - mean(0))/eigen_vector(0,1);
    ROS_WARN_STREAM("N = "<<n);
    double y1 = mean(1) + n * eigen_vector(1,1);
    ROS_WARN_STREAM("Y1 = "<<y1);
    n = (0.2049 - mean(0))/eigen_vector(0,1);
    ROS_WARN_STREAM("N = "<<n);
    double y2 = mean(1) + n * eigen_vector(1,1);
    ROS_WARN_STREAM("Y2 = "<<y2);

    n = 0.0038 - mean(1)/eigen_vector(1,1);
    double x1 = mean(0) + n * eigen_vector(0,1);
    ROS_WARN_STREAM("X1 = "<<x1);
    n = 0.3554 - mean(1)/eigen_vector(1,1);
    double x2 = mean(0) + n * eigen_vector(0,1);
    ROS_WARN_STREAM("X2 = "<<x2);

    double start_x, start_y, end_x, end_y;
    std::vector<double> temp_x, temp_y;
    if (y1 >= 0.0038 && y1<=0.3554){
        temp_x.push_back(-0.2642);
        temp_y.push_back(y1);
    }
    if (y2 >= 0.0038 && y2<=0.3554){
        temp_x.push_back(0.2049);
        temp_y.push_back(y2);
    }
    if (x1 >=-0.2642 && x1<=0.2049){
        temp_x.push_back(x1);
        temp_y.push_back(0.0038);
    }
    if (x2 >=-0.2642 && x2 <=0.2049){
        temp_x.push_back(x2);
        temp_y.push_back(0.3554);
    }
    ROS_WARN_STREAM("temp_x size : " << temp_x.size());
    ROS_WARN_STREAM("temp_y size : " << temp_y.size());

    double dist1 = calcDistance(temp_x.at(0), temp_y.at(0), mean);
    double dist2 = calcDistance(temp_x.at(1), temp_y.at(1), mean);
    if (dist1 < dist2){
    start_x = temp_x.at(0);
    start_y = temp_y.at(0);
    end_x = temp_x.at(1);
    end_y = temp_y.at(1);
    }
    else{
    start_x = temp_x.at(1);
    start_y = temp_y.at(1);
    end_x = temp_x.at(0);
    end_y = temp_y.at(0);
    }
    Eigen::Matrix3f rotation_1;
    rotation_1 <<0,0,1,
                0,1,0,
                -1,0,0;
    
    Eigen::Vector3f euler_angles = eigen_vector.eulerAngles(0,1,2);
    ROS_WARN_STREAM("euler angles : "<< euler_angles);
    Eigen::Matrix3f rotation_2;
    if (euler_angles[2]<0){
        rotation_2 = Eigen::AngleAxisf(euler_angles[2] - 1.5707963, Eigen::Vector3f::UnitX())
                                * Eigen::AngleAxisf(euler_angles[1], Eigen::Vector3f::UnitY())
                                * Eigen::AngleAxisf(euler_angles[0], Eigen::Vector3f::UnitZ());
    }
    else{
        rotation_2 = Eigen::AngleAxisf(euler_angles[2] + 1.5707963, Eigen::Vector3f::UnitX())
                                * Eigen::AngleAxisf(euler_angles[1], Eigen::Vector3f::UnitY())
                                * Eigen::AngleAxisf(euler_angles[0], Eigen::Vector3f::UnitZ());
    }
    ROS_WARN_STREAM("rotation_2 : "<< rotation_2);
    rotation_2 = rotation_1 * rotation_2;
    ROS_WARN_STREAM("rotation_2 : "<< rotation_2);
    Eigen::Quaternionf q(rotation_2);
    
    ROS_WARN_STREAM("Quaternion : x" <<q.x() << " y : "<<q.y() << "z: "<<q.z()<<"w: "<<q.w());
    geometry_msgs::Pose start_pose1, end_pose1;
    start_pose1.position.x = start_x;
    start_pose1.position.y = start_y;
    end_pose1.position.x = end_x;
    end_pose1.position.y = end_y;
    start_pose1.position.z = 1.035;
    start_pose1.orientation.x = q.x();
    start_pose1.orientation.y = q.y();
    start_pose1.orientation.z = q.z();
    start_pose1.orientation.w = q.w();

    end_pose1.position.z = 1.035;
    end_pose1.orientation.x = q.x();
    end_pose1.orientation.y = q.y();
    end_pose1.orientation.z = q.z();
    end_pose1.orientation.w = q.w();

    ur3_moveit::MoverService plan_request;
    ur3_moveit::MoverServiceResponse plan_response;
    plan_request.request.pose_1 = start_pose1;
    plan_request.request.pose_2 = end_pose1;
    gettrajectories.call(plan_request);
    plan_response = plan_request.response;
    ur3_moveit::MoveRobot moverobotsrv;
    moverobotsrv.request.trajectories = plan_response.trajectories;
    moveRobot.call(moverobotsrv);
    ros::Duration(3.0).sleep();
    ur3_moveit::resetRobot resetsrv;
    resetsrv.request.reset = true;
    resetRobotClient.call(resetsrv);
    ros::Duration(3.0).sleep();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "sceneUnderstanding_educated");
    sceneUnderstanding_educated process;
    while (ros::ok()){
        process.execute();
        ros::Duration(5.0).sleep();
    }
    return 0;
}