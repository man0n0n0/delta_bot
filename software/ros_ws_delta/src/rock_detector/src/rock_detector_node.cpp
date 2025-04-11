#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_ros/transforms.hpp>

class RockDetectorNode : public rclcpp::Node
{
public:
  RockDetectorNode() : Node("rock_detector_node")
  {
    // Subscribe to point cloud data
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_cloud", 10, std::bind(&RockDetectorNode::cloud_callback, this, std::placeholders::_1));

    // Publisher for processed point cloud showing rock surfaces
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("rock_surfaces", 10);

    // Publisher for table plane (optional)
    table_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("table_plane", 10);

    RCLCPP_INFO(this->get_logger(), "Rock detector node has started");
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received point cloud with %d points", cloud_msg->height * cloud_msg->width);

    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Filter point cloud (optional - remove points outside a certain range)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");  // Assuming z is height above ground
    pass.setFilterLimits(0.3, 1.5);  // minimal / maximal included point on the z axis (finetune for each iteration)
    pass.filter(*cloud_filtered);

    // Segment the table plane
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);  // threshold from above the table 
    seg.setInputCloud(cloud_filtered);
    seg.segment(*table_inliers, *coefficients);

    if (table_inliers->indices.size() == 0) {
      RCLCPP_WARN(this->get_logger(), "Could not find a table plane");
      return;
    }

    // Optional: Publish table plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr table_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(table_inliers);
    extract.setNegative(false);
    extract.filter(*table_cloud);

    sensor_msgs::msg::PointCloud2 table_msg;
    pcl::toROSMsg(*table_cloud, table_msg);
    table_msg.header = cloud_msg->header;
    table_publisher_->publish(table_msg);

    // Extract objects above the table
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>());
    extract.setNegative(true);
    extract.filter(*objects);

    // Cluster extraction (to separate individual rocks)
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.05);  // how close points need to be to each other to be considered part of the same cluster. 
    ec.setMinClusterSize(50);     // Minimum points in a cluster
    ec.setMaxClusterSize(20000);   // Maximum points in a cluster
    ec.setInputCloud(objects);
    
    // Create KdTree for search
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(objects);
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);

    // Prepare combined cloud for all rock surfaces
    pcl::PointCloud<pcl::PointXYZ>::Ptr rock_surfaces_combined(new pcl::PointCloud<pcl::PointXYZ>);

    // Process each cluster (rock)
    int cluster_id = 0;
    for (const auto& indices : cluster_indices) {
      RCLCPP_INFO(this->get_logger(), "Processing cluster %d with %ld points", 
                 cluster_id++, indices.indices.size());
      
      // Extract cluster points
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto& idx : indices.indices) {
        cluster->push_back(objects->points[idx]);
      }
      cluster->width = cluster->size();
      cluster->height = 1;
      cluster->is_dense = true;

      // Compute normals
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud(cluster);
      ne.setSearchMethod(tree);
      pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
      ne.setKSearch(20);  // Use nearest neighbors
      ne.compute(*normals);

      // Find surface points using convex hull
      pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::ConvexHull<pcl::PointXYZ> hull;
      hull.setInputCloud(cluster);
      hull.reconstruct(*surface_hull);

      // Add surface points to combined cloud
      *rock_surfaces_combined += *surface_hull;
    }

    // Publish rock surfaces
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*rock_surfaces_combined, output_msg);
    output_msg.header = cloud_msg->header;
    publisher_->publish(output_msg);

    RCLCPP_INFO(this->get_logger(), "Found %ld rock clusters", cluster_indices.size());
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr table_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RockDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}