#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>  // For publishing cluster centers
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64.hpp>  // For publishing plane distance
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/common/centroid.h>  // For computing centroids

class RockDetectorNode : public rclcpp::Node
{
public:
  RockDetectorNode() : Node("rock_detector_node")
  {
    // Declare parameters with default values and descriptions
    auto filter_z_min_desc = rcl_interfaces::msg::ParameterDescriptor{};
    filter_z_min_desc.description = "Minimum Z value for point cloud filtering (meters)";
    this->declare_parameter("filter_z_min", 0.005, filter_z_min_desc);
    
    auto filter_z_max_desc = rcl_interfaces::msg::ParameterDescriptor{};
    filter_z_max_desc.description = "Maximum Z value for point cloud filtering (meters)";
    this->declare_parameter("filter_z_max", 0.4, filter_z_max_desc);
    
    auto cluster_tolerance_desc = rcl_interfaces::msg::ParameterDescriptor{};
    cluster_tolerance_desc.description = "Clustering tolerance for separating objects (meters)";
    this->declare_parameter("cluster_tolerance", 0.004, cluster_tolerance_desc);
    
    auto min_cluster_size_desc = rcl_interfaces::msg::ParameterDescriptor{};
    min_cluster_size_desc.description = "Minimum number of points required for a cluster";
    this->declare_parameter("min_cluster_size", 150, min_cluster_size_desc);
    
    auto max_cluster_size_desc = rcl_interfaces::msg::ParameterDescriptor{};
    max_cluster_size_desc.description = "Maximum number of points allowed for a cluster";
    this->declare_parameter("max_cluster_size", 100000, max_cluster_size_desc);
    
    auto plane_threshold_desc = rcl_interfaces::msg::ParameterDescriptor{};
    plane_threshold_desc.description = "Distance threshold for plane segmentation (meters)";
    this->declare_parameter("plane_threshold", 0.01, plane_threshold_desc);
    
    auto k_neighbors_desc = rcl_interfaces::msg::ParameterDescriptor{};
    k_neighbors_desc.description = "Number of neighbors for normal estimation";
    this->declare_parameter("k_neighbors", 1000, k_neighbors_desc);

    // Subscribe to point cloud data
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "point_cloud", 10, std::bind(&RockDetectorNode::cloud_callback, this, std::placeholders::_1));

    // Publisher for processed point cloud showing rock surfaces
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("rock_surfaces", 10);

    // Publisher for table plane (optional)
    table_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("table_plane", 10);
    
    // Publisher for cluster centers
    centers_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("rock_centers", 10);

    // Publisher for plane distance from sensor
    plane_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("plane_distance", 10);

    RCLCPP_INFO(this->get_logger(), "Rock detector node has started");
    
    // Log all parameter values at startup
    log_parameters();
  }

private:
  void log_parameters()
  {
    double filter_z_min = this->get_parameter("filter_z_min").as_double();
    double filter_z_max = this->get_parameter("filter_z_max").as_double();
    double cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
    int min_cluster_size = this->get_parameter("min_cluster_size").as_int();
    int max_cluster_size = this->get_parameter("max_cluster_size").as_int();
    double plane_threshold = this->get_parameter("plane_threshold").as_double();
    int k_neighbors = this->get_parameter("k_neighbors").as_int();
    
    RCLCPP_INFO(this->get_logger(), "=== Rock Detector Parameters ===");
    RCLCPP_INFO(this->get_logger(), "filter_z_min: %.3f", filter_z_min);
    RCLCPP_INFO(this->get_logger(), "filter_z_max: %.3f", filter_z_max);
    RCLCPP_INFO(this->get_logger(), "cluster_tolerance: %.3f", cluster_tolerance);
    RCLCPP_INFO(this->get_logger(), "min_cluster_size: %d", min_cluster_size);
    RCLCPP_INFO(this->get_logger(), "max_cluster_size: %d", max_cluster_size);
    RCLCPP_INFO(this->get_logger(), "plane_threshold: %.3f", plane_threshold);
    RCLCPP_INFO(this->get_logger(), "k_neighbors: %d", k_neighbors);
    RCLCPP_INFO(this->get_logger(), "===============================");
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received point cloud with %d points", cloud_msg->height * cloud_msg->width);

    // Get parameters (retrieved fresh each time in case they were updated)
    double filter_z_min = this->get_parameter("filter_z_min").as_double();
    double filter_z_max = this->get_parameter("filter_z_max").as_double();
    double cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
    int min_cluster_size = this->get_parameter("min_cluster_size").as_int();
    int max_cluster_size = this->get_parameter("max_cluster_size").as_int();
    double plane_threshold = this->get_parameter("plane_threshold").as_double();
    int k_neighbors = this->get_parameter("k_neighbors").as_int();

    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Filter point cloud (optional - remove points outside a certain range)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");  // Assuming z is height above ground
    pass.setFilterLimits(filter_z_min, filter_z_max);  // minimal / maximal included point on the z axis
    pass.filter(*cloud_filtered);

    // Segment the table plane
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(plane_threshold);  // threshold from above the table 
    seg.setInputCloud(cloud_filtered);
    seg.segment(*table_inliers, *coefficients);

    if (table_inliers->indices.size() == 0) {
      RCLCPP_WARN(this->get_logger(), "Could not find a table plane");
      return;
    }

    // Calculate and publish plane distance from sensor origin (0,0,0)
    double a = coefficients->values[0];
    double b = coefficients->values[1];
    double c = coefficients->values[2];
    double d = coefficients->values[3];
    
    // Distance from origin (sensor position) to plane: |ax + by + cz + d| / sqrt(a² + b² + c²)
    // where (x,y,z) = (0,0,0) for sensor origin
    double plane_distance_from_sensor = std::abs(d) / std::sqrt(a*a + b*b + c*c);
    
    // Publish plane distance
    std_msgs::msg::Float64 distance_msg;
    distance_msg.data = plane_distance_from_sensor;
    plane_distance_publisher_->publish(distance_msg);
    
    RCLCPP_INFO(this->get_logger(), "Plane distance from sensor: %.3f meters", plane_distance_from_sensor);

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
    ec.setClusterTolerance(cluster_tolerance);  // Using parameter
    ec.setMinClusterSize(min_cluster_size);     // Using parameter
    ec.setMaxClusterSize(max_cluster_size);     // Using parameter
    ec.setInputCloud(objects);
    
    // Create KdTree for search
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(objects);
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);

    // Prepare combined cloud for all rock surfaces
    pcl::PointCloud<pcl::PointXYZ>::Ptr rock_surfaces_combined(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Create a message to hold all cluster centers
    geometry_msgs::msg::PoseArray centers_msg;
    centers_msg.header = cloud_msg->header;

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

      // Calculate the centroid of the cluster (X,Y only) and find surface Z (minimum Z)
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cluster, centroid);
      
      // Find the point with minimum Z value (closest to camera/surface)
      float min_z = std::numeric_limits<float>::max();
      for (const auto& point : cluster->points) {
        if (point.z < min_z) {
          min_z = point.z;
        }
      }
      
      // Use average X,Y but surface Z
      float surface_x = centroid[0];  // Average X
      float surface_y = centroid[1];  // Average Y
      float surface_z = min_z;        // Surface Z (minimum Z)
      
      // Check if centroid is below or at the plane level (exclude points above plane)
      // Calculate signed distance to plane using the surface point
      double signed_distance = (a * surface_x + b * surface_y + c * surface_z + d) / 
                              std::sqrt(a*a + b*b + c*c);
      
      // If the plane normal points down (c < 0), flip the sign
      if (c < 0) {
        signed_distance = -signed_distance;
      }
      
      // Skip centroids that are above the plane
      if (signed_distance > 0) {
        continue;
      }
      
      // Create a Pose message for this surface-based centroid
      geometry_msgs::msg::Pose center_pose;
      center_pose.position.x = surface_x;
      center_pose.position.y = surface_y;
      center_pose.position.z = surface_z;
      center_pose.orientation.w = 1.0; // Default orientation (no rotation)
      
      // Add the pose to our PoseArray
      centers_msg.poses.push_back(center_pose);
      
      RCLCPP_INFO(this->get_logger(), "Cluster %d surface center: x=%.3f, y=%.3f, z=%.3f (surface)", 
                  cluster_id-1, surface_x, surface_y, surface_z);

      // Rest of the processing (normals, convex hull, etc.) remains the same
      // Compute normals
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud(cluster);
      ne.setSearchMethod(tree);
      pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
      ne.setKSearch(k_neighbors);  // Using parameter
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
    
    // Publish cluster centers
    centers_publisher_->publish(centers_msg);

    RCLCPP_INFO(this->get_logger(), "Found %ld rock clusters", cluster_indices.size());
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr table_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr centers_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr plane_distance_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RockDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}