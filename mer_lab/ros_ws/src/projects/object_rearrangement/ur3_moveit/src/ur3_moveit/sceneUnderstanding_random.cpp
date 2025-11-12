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
#include <cstdlib>
class sceneUnderstanding_random{
    public:
    sceneUnderstanding_random();
    std::vector<geometry_msgs::Pose> getPose(double x1, double y1, double z1, double x2, double y2, double z2, double x_q, double y_q, double z_q, double w_q);
    void execute();
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::ServiceClient gettrajectories;
        ros::ServiceClient moveRobot;
        ros::ServiceClient getCloud;
        ros::ServiceClient resetRobotClient;
        int counter = 0;
};


sceneUnderstanding_random::sceneUnderstanding_random(){
    // sub = nh.subscribe("/unity/pointCloud", 10,&sceneUnderstanding_random::callBack, this);
    gettrajectories = nh.serviceClient<ur3_moveit::MoverService>("/ur3_moveit/plantrajectory");
    moveRobot = nh.serviceClient<ur3_moveit::MoveRobot>("/ur3_moveit/MoveRobot");
    getCloud = nh.serviceClient<ur3_moveit::getPointCloud>("/ur3_moveit/pointcloud");
    resetRobotClient = nh.serviceClient<ur3_moveit::resetRobot>("ur3_moveit/ResetRobot");
}

std::vector<geometry_msgs::Pose> sceneUnderstanding_random::getPose(double x1, double y1, double z1, double x2, double y2, double z2, double x_q, double y_q, double z_q, double w_q){
    std::vector<geometry_msgs::Pose> pose_vector;
    geometry_msgs::Pose start_pose1, end_pose1;
    start_pose1.position.x = x1;
    start_pose1.position.y = y1;
    start_pose1.position.z = z1;
    start_pose1.orientation.x = x_q;
    start_pose1.orientation.y = y_q;
    start_pose1.orientation.z = z_q;
    start_pose1.orientation.w = w_q;
    end_pose1.position.x = x2;
    end_pose1.position.y = y2;
    end_pose1.position.z = z2;
    end_pose1.orientation.x = x_q;
    end_pose1.orientation.y = y_q;
    end_pose1.orientation.z = z_q;
    end_pose1.orientation.w = w_q;
    pose_vector.push_back(start_pose1);
    pose_vector.push_back(end_pose1);
    return pose_vector;
}
void sceneUnderstanding_random::execute(){
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
    std::vector<int> pose_number(8);
    std::iota(std::begin(pose_number), std::end(pose_number), 0);
    srand(time(0));
    int random = rand() % pose_number.size();
    int sel_pose = pose_number[random];
    std::vector<geometry_msgs::Pose> pose_vector;
    switch (sel_pose){
        case 0:
            pose_vector = getPose(-0.2642,0,1.037,0.2049,0,1.037,0,0.7071068,0,0.7071068);
            break;
        case 1:
            pose_vector = getPose(0.2049,0.1,1.037,-0.2642,0.1,1.037,0,0.7071068,0,0.7071068);
            break;
        case 2:
            pose_vector = getPose(-0.2642,0.2,1.037,0.2049,0.2,1.037,0,0.7071068,0,0.7071068);
            break;
        case 3:
            pose_vector = getPose(0.2049,0.3,1.037,-0.2642,0.3,1.037,0,0.7071068,0,0.7071068);
            break;

        case 4:
            pose_vector = getPose(-0.2, -0.0038,1.037,-0.2,0.3554, 1.037,-0.5,-0.5,0.5,-0.5);
            break;

        case 5:
            pose_vector = getPose(-0.1, 0.3554,1.037,-0.1,-0.0038, 1.037,-0.5,-0.5,0.5,-0.5);
            break;

        case 6:
            pose_vector = getPose(0.1, -0.0038,1.037,0.1,0.3554, 1.037,-0.5,-0.5,0.5,-0.5);
            break;

        case 7:
            pose_vector = getPose(0.2, 0.3554,1.037,0.2,-0.0038, 1.037,-0.5,-0.5,0.5,-0.5);
            break;
    }
    ROS_WARN_STREAM("Selected pose : "<<sel_pose << "randomly");
    geometry_msgs::Pose start_pose1, end_pose1;
    start_pose1 = pose_vector.at(0);
    end_pose1 = pose_vector.at(1);
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

    if (moverobotsrv.response.success){
        ROS_INFO("Successfully Moved the robot");
    }
    else{
        ROS_INFO("failed to move the robot");
    }
    ros::Duration(3.0).sleep();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "sceneUnderstanding_random");
    sceneUnderstanding_random process;
    while (ros::ok()){
        process.execute();
        ros::Duration(5.0).sleep();
    }
    return 0;
}