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

class sceneUnderstanding_educated{
    public:
    sceneUnderstanding_educated();
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

sceneUnderstanding_educated::sceneUnderstanding_educated(){
    gettrajectories = nh.serviceClient<ur3_moveit::MoverService>("/ur3_moveit/plantrajectory");
    moveRobot = nh.serviceClient<ur3_moveit::MoveRobot>("/ur3_moveit/MoveRobot");
    getCloud = nh.serviceClient<ur3_moveit::getPointCloud>("/ur3_moveit/pointcloud");
    resetRobotClient = nh.serviceClient<ur3_moveit::resetRobot>("ur3_moveit/ResetRobot");
}
void sceneUnderstanding_educated::execute(){
    counter++;
    ros::Duration(3.0).sleep();
    ros::service::waitForService("/ur3_moveit/pointcloud");
    ros::service::waitForService("/ur3_moveit/MoveRobot");
    ur3_moveit::getPointCloud cloud_req;
    cloud_req.request.send = true;
    getCloud.call(cloud_req);
    sensor_msgs::PointCloud2 cloud;
    cloud = cloud_req.response.cloud;
    ROS_INFO("Received Cloud");
    ROS_WARN("--------------------------------------------------------");
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
    double max_z = 0;
    pcl::PointXYZ max;
    for (auto pt: cloud_without_table_filtered->points){
        if (pt.z > max_z){
            max_z = pt.z;
            max = pt;
        }
    }
    ROS_WARN_STREAM("Max Pt: x: "<<max.x <<"y: "<<max.y<<"z: "<<max.z);
    

    pcl::PointXYZ min_pt1, max_pt1;
    pcl::getMinMax3D(*cloud_filtered, min_pt1, max_pt1);
    double x1,y1,x2,y2;
    x1 = min_pt1.x;
    x2 = max_pt1.x;
    y1 = min_pt1.y;
    y2 = max_pt1.y;
    geometry_msgs::Pose start_pose1, end_pose1, start_pose2, end_pose2;

    double center_x = (x1 + x2)/2;
    double center_y = (y1 + y2)/2;

    if (max.x < center_x && max.y > center_y){
        start_pose1.position.x = x1;
        start_pose1.position.y = max.y;
        end_pose1.position.x = x2;
        end_pose1.position.y = max.y;
    }else if(max.x < center_x && max.y < center_y){
        start_pose1.position.x = x1;
        start_pose1.position.y = max.y;
        end_pose1.position.x = x2;
        end_pose1.position.y = max.y;
    }else if(max.x > center_x && max.y < center_y){
        start_pose1.position.x = x2;
        start_pose1.position.y = max.y;
        end_pose1.position.x = x1;
        end_pose1.position.y = max.y;
    }
    else if(max.x > center_x && max.y > center_y){
        start_pose1.position.x = x1;
        start_pose1.position.y = max.y;
        end_pose1.position.x = x2;
        end_pose1.position.y = max.y;
    }
    start_pose1.position.z = 1.035;
    start_pose1.orientation.x = 0;
    start_pose1.orientation.y = 0.7071068;
    start_pose1.orientation.z = 0;
    start_pose1.orientation.w = 0.7071068;
    end_pose1.position.z = 1.037;
    end_pose1.orientation.x = 0;
    end_pose1.orientation.y = 0.7071068;
    end_pose1.orientation.z = 0;
    end_pose1.orientation.w = 0.7071068;
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

    if (max.x < center_x && max.y > center_y){
        start_pose2.position.x = max.x;
        start_pose2.position.y = y2;
        end_pose2.position.x = max.x;
        end_pose2.position.y = y1;
    }else if(max.x < center_x && max.y < center_y){
        start_pose2.position.x = max.x;
        start_pose2.position.y = y1;
        end_pose2.position.x = max.x;
        end_pose2.position.y = y2;
    }else if(max.x > center_x && max.y < center_y){
        start_pose2.position.x = max.x;
        start_pose2.position.y = y1;
        end_pose2.position.x = max.x;
        end_pose2.position.y = y2;
    }
    else if(max.x > center_x && max.y > center_y){
        start_pose2.position.x = max.x;
        start_pose2.position.y = y2;
        end_pose2.position.x = max.x;
        end_pose2.position.y = y1;
    }
    start_pose2.position.z = 1.037;
    start_pose2.orientation.x = -0.5;
    start_pose2.orientation.y = -0.5;
    start_pose2.orientation.z = 0.5;
    start_pose2.orientation.w = -0.5;
    end_pose2.position.z = 1.035;
    end_pose2.orientation.x = -0.5;
    end_pose2.orientation.y = -0.5;
    end_pose2.orientation.z = 0.5;
    end_pose2.orientation.w = -0.5;
    ur3_moveit::MoverService plan_request_1;
    ur3_moveit::MoverServiceResponse plan_response_1;
    plan_request_1.request.pose_1 = start_pose2;
    plan_request_1.request.pose_2 = end_pose2;
    gettrajectories.call(plan_request_1);
    plan_response_1 = plan_request_1.response;
    moverobotsrv.request.trajectories = plan_response_1.trajectories;
    moveRobot.call(moverobotsrv);

    if (moverobotsrv.response.success){
        ROS_INFO("Successfully Moved the robot");
    }
    else{
        ROS_INFO("failed to move the robot");
    }
    ros::Duration(3.0).sleep();
    ur3_moveit::resetRobot resetsrv_1;
    resetsrv_1.request.reset = true;
    resetRobotClient.call(resetsrv_1);
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