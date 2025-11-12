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
#include<ros/package.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl/common/common.h>
#include<ur3_moveit/MoverService.h>
#include<ur3_moveit/MoveRobot.h>
#include<ur3_moveit/getPointCloud.h>
#include<ur3_moveit/resetRobot.h>

class sceneUnderstanding{
    public:
    sceneUnderstanding();
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

sceneUnderstanding::sceneUnderstanding(){
    // sub = nh.subscribe("/unity/pointCloud", 10,&sceneUnderstanding::callBack, this);
    gettrajectories = nh.serviceClient<ur3_moveit::MoverService>("/ur3_moveit/plantrajectory");
    moveRobot = nh.serviceClient<ur3_moveit::MoveRobot>("/ur3_moveit/MoveRobot");
    getCloud = nh.serviceClient<ur3_moveit::getPointCloud>("/ur3_moveit/pointcloud");
    resetRobotClient = nh.serviceClient<ur3_moveit::resetRobot>("ur3_moveit/ResetRobot");
}
void sceneUnderstanding::execute(){
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

    std::string path = ros::package::getPath("ur3_moveit") + "/output_cloud/Second Phase/";
    pcl::io::savePLYFile(path + "cloud_with_out-Table"+ std::to_string(counter) + ".ply", *cloud_without_table);
    pcl::io::savePLYFile(path + "scenecloud"+  std::to_string(counter) + ".ply", *pcl_cloud);

    // DownSampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_table_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (pcl_cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

    sor.setInputCloud (cloud_without_table);
    sor.setLeafSize (0.001f, 0.001f, 0.001f);
    sor.filter (*cloud_without_table_filtered);
    pcl::io::savePLYFile(path + "cloud_without_table_filtered" + std::to_string(counter) + ".ply", *cloud_without_table_filtered);
    pcl::io::savePLYFile(path + "cloud_filtered" +  std::to_string(counter) + ".ply", *cloud_filtered);
    
    double avg_z1, avg_z2, temp_z;
    temp_z = 0.0;
    for (auto pt : *cloud_filtered){
        temp_z += pt.z; 
    }
    avg_z1 = temp_z/cloud_filtered->points.size();
    temp_z = 0.0;

    for (auto pt : *cloud_without_table_filtered){
        temp_z += pt.z; 
    }
    avg_z2 = temp_z/cloud_without_table_filtered->points.size();

    ROS_WARN_STREAM("Avg_Z1 = "<<avg_z1);
    ROS_WARN_STREAM("Avg_Z2 = "<<avg_z2);

    pcl::PointXYZ min_pt1, max_pt1, min_pt2, max_pt2;
    pcl::getMinMax3D(*cloud_filtered, min_pt1, max_pt1);
    pcl::getMinMax3D(*cloud_without_table_filtered, min_pt2, max_pt2);

    double x1,y1,x2,y2;
    x1 = min_pt1.x;
    x2 = max_pt1.x;
    // x2 = max_pt2.x;
    y1 = min_pt1.y;
    y2 = max_pt1.y;

    geometry_msgs::Pose start_pose1, end_pose1, start_pose2, end_pose2;

    start_pose1.position.x = 0.22;
    start_pose1.position.y = 0.2;
    // start_pose1.position.z = avg_z1 + 0.08;
    start_pose1.position.z = 1.037;
    start_pose1.orientation.x = 0;
    start_pose1.orientation.y = 0.7071068;
    start_pose1.orientation.z = 0;
    start_pose1.orientation.w = 0.7071068;
    end_pose1.position.x = -0.2;
    end_pose1.position.y = 0.2;
    // end_pose1.position.z = avg_z1 + 0.08;
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

    start_pose2.position.x = 0;
    start_pose2.position.y = 0.35;
    // start_pose1.position.z = avg_z1 + 0.08;
    start_pose2.position.z = 1.037;
    start_pose2.orientation.x = -0.5;
    start_pose2.orientation.y = -0.5;
    start_pose2.orientation.z = 0.5;
    start_pose2.orientation.w = -0.5;
    end_pose2.position.x = 0;
    end_pose2.position.y = 0.0;
    // end_pose1.position.z = avg_z1 + 0.08;
    end_pose2.position.z = 1.037;
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

    // ur3_moveit::MoveRobot moverobotsrv;

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
    ros::init(argc, argv, "sceneUnderstanding");
    sceneUnderstanding process;
    while (ros::ok()){
        process.execute();
        ros::Duration(5.0).sleep();
    }
    return 0;
}