#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import sensor_msgs.msg
import subprocess
import time
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
import open3d as o3d
from collections import deque

class ClosestObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('closest_object_detection')
        
        # Declare parameters
        self.declare_parameter('pointcloud_topic', '/point_cloud')
        self.declare_parameter('grbl_action_name', '/delta_marlin/send_gcode_cmd')
        self.declare_parameter('min_distance_threshold', 0.005)  # Minimum distance in meters to trigger action
        self.declare_parameter('num_points_to_process', 20000)  # Max number of points to consider
        self.declare_parameter('x_limit', (-150, 150))  
        self.declare_parameter('y_limit', (-150, 150))  
        self.declare_parameter('z_limit', (0, 110))
        
        # Preprocessing parameters
        self.declare_parameter('voxel_size', 0.005)  # Voxel grid size for downsampling (in meters)
        self.declare_parameter('outlier_std_ratio', 2.0)  # Standard deviation ratio for statistical outlier removal
        self.declare_parameter('outlier_nb_neighbors', 20)  # Number of neighbors for statistical outlier removal
        
        # Segmentation parameters
        self.declare_parameter('plane_distance_threshold', 0.01)  # RANSAC plane distance threshold
        self.declare_parameter('cluster_eps', 0.02)  # DBSCAN clustering epsilon (max distance between points)
        self.declare_parameter('cluster_min_points', 50)  # Minimum number of points to form a cluster
        
        # Verification parameters
        self.declare_parameter('temporal_window_size', 3)  # Number of frames to consider for verification
        self.declare_parameter('consistency_threshold', 10.0)  # Maximum distance in mm between consecutive detections
        
        # Z-height for positioning
        self.declare_parameter('fixed_z_height', 70.0)  # Fixed Z height to move to in mm

        # Get parameters
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.grbl_action_name = self.get_parameter('grbl_action_name').value
        self.min_distance_threshold = self.get_parameter('min_distance_threshold').value
        self.num_points_to_process = self.get_parameter('num_points_to_process').value
        self.x_limit = self.get_parameter('x_limit').value
        self.y_limit = self.get_parameter('y_limit').value
        self.z_limit = self.get_parameter('z_limit').value
        
        # Get preprocessing parameters
        self.voxel_size = self.get_parameter('voxel_size').value
        self.outlier_std_ratio = self.get_parameter('outlier_std_ratio').value
        self.outlier_nb_neighbors = self.get_parameter('outlier_nb_neighbors').value
        
        # Get segmentation parameters
        self.plane_distance_threshold = self.get_parameter('plane_distance_threshold').value
        self.cluster_eps = self.get_parameter('cluster_eps').value
        self.cluster_min_points = self.get_parameter('cluster_min_points').value
        
        # Get verification parameters
        self.temporal_window_size = self.get_parameter('temporal_window_size').value
        self.consistency_threshold = self.get_parameter('consistency_threshold').value
        
        # Get Z height
        self.fixed_z_height = self.get_parameter('fixed_z_height').value

        # Subscribe to point cloud
        self.pointcloud_sub = self.create_subscription(
            sensor_msgs.msg.PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            10)
        
        # Temporal verification buffer
        self.detected_objects_history = deque(maxlen=self.temporal_window_size)
        
        # Send homing command
        self.send_grbl_command("$G28")

        self.previous_position = None
        self.processing_locked = False
        
        self.get_logger().info("Closest Object Detection node initialized!")

    def send_grbl_command(self, command):
        """Helper method to send GRBL commands via ros2 action"""
        ros2_command = [
            "ros2", "action", "send_goal",
            self.grbl_action_name,
            "grbl_msgs/action/SendGcodeCmd",
            f"{{command: {command}}}"
        ]
        
        process = subprocess.Popen(
            ros2_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
    def preprocess_pointcloud(self, points_array):
        """
        Preprocess the point cloud:
        1. Convert to Open3D format
        2. Downsample using voxel grid
        3. Remove statistical outliers
        """
        self.get_logger().info(f"Preprocessing point cloud with {len(points_array)} points")
        
        # Convert numpy array to Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_array)
        
        # Downsample using voxel grid
        downsampled_pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        
        # Remove statistical outliers
        filtered_pcd, _ = downsampled_pcd.remove_statistical_outlier(
            nb_neighbors=self.outlier_nb_neighbors,
            std_ratio=self.outlier_std_ratio
        )
        
        # Convert back to numpy array
        filtered_points = np.asarray(filtered_pcd.points)
        
        self.get_logger().info(f"After preprocessing: {len(filtered_points)} points")
        return filtered_points

    def segment_objects(self, points_array):
        """
        Segment objects in the point cloud:
        1. Remove ground plane using RANSAC
        2. Cluster remaining points using DBSCAN
        """
        self.get_logger().info("Segmenting objects in point cloud")
        
        # Convert numpy array to Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_array)
        
        # Segment plane (table surface)
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=self.plane_distance_threshold,
            ransac_n=3,
            num_iterations=100
        )
        
        # Extract non-plane points (objects)
        object_cloud = pcd.select_by_index(inliers, invert=True)
        
        if len(object_cloud.points) < self.cluster_min_points:
            self.get_logger().warn(f"Not enough non-plane points ({len(object_cloud.points)}) for clustering")
            return []
            
        # Convert to numpy for DBSCAN clustering
        object_points = np.asarray(object_cloud.points)
        
        # Cluster objects using DBSCAN
        dbscan = DBSCAN(eps=self.cluster_eps, min_samples=self.cluster_min_points)
        cluster_labels = dbscan.fit_predict(object_points)
        
        # Extract clusters (ignoring noise label -1)
        clusters = []
        for label in np.unique(cluster_labels):
            if label != -1:  # Skip noise points
                cluster_mask = cluster_labels == label
                cluster_points = object_points[cluster_mask]
                
                # Only add clusters with enough points
                if len(cluster_points) >= self.cluster_min_points:
                    # Calculate centroid and other features
                    centroid = np.mean(cluster_points, axis=0)
                    dimensions = np.max(cluster_points, axis=0) - np.min(cluster_points, axis=0)
                    dist_to_origin = np.linalg.norm(centroid)
                    
                    clusters.append({
                        'points': cluster_points,
                        'centroid': centroid,
                        'dimensions': dimensions,
                        'distance': dist_to_origin,
                        'num_points': len(cluster_points)
                    })
        
        # Sort clusters by distance to origin
        clusters.sort(key=lambda x: x['distance'])
        
        self.get_logger().info(f"Found {len(clusters)} object clusters")
        return clusters

    def verify_detection(self, detected_object):
        """
        Verify object detection using temporal consistency:
        1. Check if object is consistent with previous detections
        2. Update detection history
        """
        if not self.detected_objects_history:
            # First detection, add to history
            self.detected_objects_history.append(detected_object['centroid'])
            return True
            
        # Calculate distance to previous detections
        current_centroid = detected_object['centroid'] * 1000.0  # Convert to mm for comparison
        prev_centroids = np.array(self.detected_objects_history) * 1000.0
        
        # Calculate distances to all previous centroids
        distances = np.linalg.norm(prev_centroids - current_centroid, axis=1)
        
        # Update history
        self.detected_objects_history.append(detected_object['centroid'])
        
        # Check if object is consistent (at least one detection is close enough)
        is_consistent = np.min(distances) < self.consistency_threshold
        
        # Report consistency
        if is_consistent:
            self.get_logger().info(f"Object detection verified (distance to prev: {np.min(distances):.2f}mm)")
        else:
            self.get_logger().warn(f"Inconsistent object detection (distance: {np.min(distances):.2f}mm > threshold)")
            
        return is_consistent

    def move_to_position(self, x_mm, y_mm):
        """Move to the specified X,Y position with fixed Z height"""
        
        # Check if coordinates are within limits
        if (self.x_limit[0] < x_mm < self.x_limit[1] and 
            self.y_limit[0] < y_mm < self.y_limit[1]):
            
            # Send G-code to move to position
            self.get_logger().info(f"Moving to position: X={x_mm:.1f}, Y={y_mm:.1f}, Z={self.fixed_z_height:.1f}")
            self.send_grbl_command(f"$G1X{x_mm:.1f}Y{y_mm:.1f}Z{self.fixed_z_height:.1f}F6000")
            return True
        else:
            self.get_logger().warn(f"Position out of machine physical limits: X={x_mm:.1f}, Y={y_mm:.1f}")
            return False

    def pointcloud_callback(self, pointcloud_msg):
        """Process point cloud and move to closest object position"""
        
        # Skip if already processing
        if self.processing_locked:
            self.get_logger().info("Skipping frame, already processing")
            return
            
        self.processing_locked = True
        
        try:
            # Home the robot to take a clean snapshot
            self.send_grbl_command("$G28")
            time.sleep(3)

            # Convert point cloud to numpy array with explicit coordinate extraction
            points_list = []
            for point in pc2.read_points(pointcloud_msg, skip_nans=True):
                points_list.append((-1 * point[1], -1 * point[0], point[2]))

            total_points = len(points_list)
            
            self.get_logger().info(f"Total points in cloud: {total_points}")

            if total_points == 0:
                self.get_logger().warn("Empty point cloud received")
                self.processing_locked = False
                return
             
            points_array = np.array(points_list, dtype=np.float64)
            
            # Limit number of points for processing efficiency
            if total_points > self.num_points_to_process:
                # Use a stratified sampling approach to maintain structure
                z_sorted_indices = np.argsort(points_array[:, 2])
                stride = max(1, total_points // self.num_points_to_process)
                sampled_indices = z_sorted_indices[::stride]
                points_array = points_array[sampled_indices]
                self.get_logger().info(f"Sampled down to {len(points_array)} points")
            
            # STEP 1: Preprocess the point cloud
            filtered_points = self.preprocess_pointcloud(points_array)
            
            if len(filtered_points) < self.cluster_min_points:
                self.get_logger().warn(f"Not enough points after preprocessing: {len(filtered_points)}")
                self.processing_locked = False
                return
            
            # STEP 2: Segment objects in the point cloud
            object_clusters = self.segment_objects(filtered_points)
            
            if not object_clusters:
                self.get_logger().warn("No objects detected in the point cloud")
                self.processing_locked = False
                return
                
            # STEP 3: Find closest object
            closest_object = object_clusters[0]  # Already sorted by distance
            
            # STEP 4: Verify detection consistency
            if not self.verify_detection(closest_object):
                self.get_logger().warn("Object detection failed verification, skipping this frame")
                self.processing_locked = False
                return
                
            # Get position in mm
            x_mm = closest_object['centroid'][0] * 1000.0  # Convert to mm
            y_mm = closest_object['centroid'][1] * 1000.0  # Convert to mm
            
            self.get_logger().info(f"Found closest object with {closest_object['num_points']} points at position: "
                                 f"X={x_mm:.1f}, Y={y_mm:.1f}")
            
            # STEP 5: Move to the X,Y position
            self.move_to_position(x_mm, y_mm)
            
        finally:
            # Ensure we always unlock processing
            self.processing_locked = False


def main(args=None):
    rclpy.init(args=args)
    node = ClosestObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()