/**
 * @file pointcloud_filter.cpp
 * @brief High-performance PointCloud filtering using PCL
 * 高性能点云滤波节点 (Phase 2 优化)
 * 
 * Subscribed Topics:
 *   /adas/lidar (sensor_msgs/PointCloud2)
 * 
 * Published Topics:
 *   /adas/lidar_filtered (sensor_msgs/PointCloud2)
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class PointCloudFilter
{
public:
    PointCloudFilter() : nh_("~")
    {
        // Load parameters
        nh_.param("voxel_size", voxel_size_, 0.1);
        nh_.param("min_z", min_z_, -2.0);
        nh_.param("max_z", max_z_, 10.0);
        nh_.param("max_distance", max_distance_, 50.0);
        nh_.param("ground_threshold", ground_threshold_, 0.3);
        
        // Publishers and subscribers
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/adas/lidar_filtered", 1);
        cloud_sub_ = nh_.subscribe("/adas/lidar", 1, &PointCloudFilter::cloudCallback, this);
        
        ROS_INFO("PointCloud Filter (C++) initialized");
    }
    
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        ros::Time start = ros::Time::now();
        
        // Convert to PCL
        PointCloud::Ptr cloud(new PointCloud);
        pcl::fromROSMsg(*msg, *cloud);
        
        if (cloud->empty())
        {
            return;
        }
        
        // 1. Voxel Grid Downsampling
        PointCloud::Ptr cloud_voxel(new PointCloud);
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel_filter.filter(*cloud_voxel);
        
        // 2. PassThrough Filter (Z-axis)
        PointCloud::Ptr cloud_pass(new PointCloud);
        pcl::PassThrough<PointT> pass_filter;
        pass_filter.setInputCloud(cloud_voxel);
        pass_filter.setFilterFieldName("z");
        pass_filter.setFilterLimits(min_z_, max_z_);
        pass_filter.filter(*cloud_pass);
        
        // 3. Ground Plane Removal using RANSAC
        PointCloud::Ptr cloud_no_ground(new PointCloud);
        removeGroundPlane(cloud_pass, cloud_no_ground);
        
        // Convert back to ROS message
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_no_ground, output);
        output.header = msg->header;
        
        cloud_pub_.publish(output);
        
        // Performance logging
        double elapsed = (ros::Time::now() - start).toSec() * 1000;
        if (elapsed > 15)
        {
            ROS_WARN("PointCloud filtering took %.1f ms (target: <15ms)", elapsed);
        }
    }
    
private:
    void removeGroundPlane(const PointCloud::Ptr& input, PointCloud::Ptr& output)
    {
        // RANSAC plane segmentation
        pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(ground_threshold_);
        
        seg.setInputCloud(input);
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.empty())
        {
            *output = *input;
            return;
        }
        
        // Extract non-ground points
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(input);
        extract.setIndices(inliers);
        extract.setNegative(true);  // Keep points NOT in plane
        extract.filter(*output);
    }
    
    ros::NodeHandle nh_;
    ros::Publisher cloud_pub_;
    ros::Subscriber cloud_sub_;
    
    double voxel_size_;
    double min_z_;
    double max_z_;
    double max_distance_;
    double ground_threshold_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_filter_node");
    PointCloudFilter filter;
    ros::spin();
    return 0;
}
