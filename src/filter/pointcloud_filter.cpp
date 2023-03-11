#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

typedef pcl::PointXYZRGB PointT;

// camera frame right, bottom, forward
constexpr float min_range[3] = {-3.0, -3.0, 0.0};
constexpr float max_range[3] = {3.0, 3.0, 3.0};
constexpr float leaf_size = 0.02;

ros::Subscriber pointcloud_sub;
ros::Publisher pointcloud_pub;

void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &pointcloud_ros) {

    static tf2_ros::Buffer tf_buffer;
    static tf2_ros::TransformListener tfListener(tf_buffer);
    geometry_msgs::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer.lookupTransform("t265_odom_frame", "d435i_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("[FILTER] %s", ex.what());
    }

    // step1: ros->pcl
    pcl::PointCloud<PointT>::Ptr pointcloud_pcl(
            new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*pointcloud_ros, *pointcloud_pcl);

    // step2: bbx filter
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> crop;
    crop.setMin(Eigen::Vector4f(min_range[0], min_range[1], min_range[2], 1.0));
    crop.setMax(Eigen::Vector4f(max_range[0], max_range[1], max_range[2], 1.0));
    crop.setInputCloud(pointcloud_pcl);
    crop.setKeepOrganized(true);
    crop.filter(*pointcloud_pcl);

    // step3: voxelgrid filter
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(pointcloud_pcl);

    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*pointcloud_pcl);

    // step4: transform (camera->lidar)
    // Eigen::Matrix4f transform = Eigen::Matrix4f::Zero();
    // transform(0, 2) = 1;
    // transform(1, 0) = -1;
    // transform(2, 1) = -1;
    // transform(3, 3) = 1;
    // pcl::transformPointCloud(*pointcloud_pcl, *pointcloud_pcl, transform);

    // step5: pcl->ros
    sensor_msgs::PointCloud2 pointcloud_ros_filtered;
    pcl::toROSMsg(*pointcloud_pcl, pointcloud_ros_filtered);
    pointcloud_ros_filtered.header.stamp = transform_stamped.header.stamp;  // ros::Time::now();
    pointcloud_ros_filtered.header.frame_id = pointcloud_ros->header.frame_id;
    pointcloud_pub.publish(pointcloud_ros_filtered);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud_filter");
    ros::NodeHandle nh;

    pointcloud_sub =
            nh.subscribe<sensor_msgs::PointCloud2>("/d435i/depth/color/points", 1, pointcloud_callback);
    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/elevation_map/input", 1);

    ros::spin();
    return 0;
}