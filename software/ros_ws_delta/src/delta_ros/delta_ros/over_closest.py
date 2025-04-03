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
from scipy.spatial import ConvexHull
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
import math
from collections import deque

class AdvancedPickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('advanced_pick_and_place')
        
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
        
        # Grasp planning parameters
        self.declare_parameter('approach_height', 20.0)  # Height in mm above object to approach
        self.declare_parameter('gripper_width', 60.0)  # Maximum gripper width in mm
        self.declare_parameter('grasp_height_offset', 10.0)  # Height offset for grasping in mm
        
        # Verification parameters
        self.declare_parameter('temporal_window_size', 3)  # Number of frames to consider for verification
        self.declare_parameter('consistency_threshold', 10.0)  # Maximum distance in mm between consecutive detections

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
        
        # Get grasp planning parameters
        self.approach_height = self.get_parameter('approach_height').value
        self.gripper_width = self.get_parameter('gripper_width').value
        self.grasp_height_offset = self.get_parameter('grasp_height_offset').value
        
        # Get verification parameters
        self.temporal_window_size = self.get_parameter('temporal_window_size').value
        self.consistency_threshold = self.get_parameter('consistency_threshold').value

        # Subscribe to point cloud
        self.pointcloud_sub = self.create_subscription(
            sensor_msgs.msg.PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            10)
            
        # Initialize TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Temporal verification buffer
        self.detected_objects_history = deque(maxlen=self.temporal_window_size)
        
        # Send homing command
        self.send_grbl_command("$G28")

        self.previous_grasp_point = None
        self.pick_and_place_state = "IDLE"  # States: IDLE, APPROACHING, GRASPING, LIFTING, PLACING
        
        self.get_logger().info("Advanced Pick and Place node initialized!")

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
                    
                    # Compute convex hull for grasp planning
                    try:
                        hull = ConvexHull(cluster_points[:, 0:2])  # 2D hull (top-down view)
                        hull_points = cluster_points[hull.vertices]
                    except:
                        hull_points = cluster_points  # Fallback if hull computation fails
                    
                    clusters.append({
                        'points': cluster_points,
                        'centroid': centroid,
                        'dimensions': dimensions,
                        'distance': dist_to_origin,
                        'hull_points': hull_points,
                        'num_points': len(cluster_points)
                    })
        
        # Sort clusters by distance to origin
        clusters.sort(key=lambda x: x['distance'])
        
        self.get_logger().info(f"Found {len(clusters)} object clusters")
        return clusters

    def determine_grasp_pose(self, object_cluster):
        """
        Determine optimal grasp pose for the object:
        1. Find principal axes of the object
        2. Calculate grasp point and approach orientation
        """
        points = object_cluster['points']
        centroid = object_cluster['centroid']
        hull_points = object_cluster['hull_points']
        
        # Calculate principal axes using PCA
        cov = np.cov(points.T)
        eigvals, eigvecs = np.linalg.eig(cov)
        
        # Sort eigenvectors by eigenvalues in descending order
        sort_indices = np.argsort(eigvals)[::-1]
        major_axis = eigvecs[:, sort_indices[0]]
        minor_axis = eigvecs[:, sort_indices[1]]
        
        # Ensure Z axis points upward
        if major_axis[2] < 0:
            major_axis = -major_axis
        if minor_axis[2] < 0:
            minor_axis = -minor_axis
            
        # Normalize axes
        major_axis = major_axis / np.linalg.norm(major_axis)
        minor_axis = minor_axis / np.linalg.norm(minor_axis)
            
        # Calculate object dimensions along principal axes
        hull_points_centered = hull_points - centroid
        major_proj = np.abs(np.dot(hull_points_centered, major_axis))
        minor_proj = np.abs(np.dot(hull_points_centered, minor_axis))
        
        major_dim = 2 * np.max(major_proj)
        minor_dim = 2 * np.max(minor_proj)
        
        # Determine approach angle (align with longer dimension if possible)
        approach_angle = 0
        if minor_dim < self.gripper_width and major_dim > minor_dim:
            # Align gripper with minor axis (perpendicular to major axis)
            approach_vector = minor_axis
            approach_angle = math.atan2(approach_vector[1], approach_vector[0])
        
        # Adjust grasp height based on object
        grasp_z = np.min(points[:, 2]) + self.grasp_height_offset / 1000.0
        
        # Convert to mm and create grasp pose
        grasp_pose = {
            'x': centroid[0] * 1000.0,  # Convert to mm
            'y': centroid[1] * 1000.0,  # Convert to mm
            'z': grasp_z * 1000.0,      # Convert to mm
            'angle': approach_angle,    # In radians
            'approach_z': grasp_z * 1000.0 + self.approach_height
        }
        
        return grasp_pose

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

    def execute_pick_and_place(self, grasp_pose):
        """
        Execute the pick and place operation:
        1. Move to approach position
        2. Move to grasp position
        3. Close gripper
        4. Lift object
        5. Move to place position
        6. Open gripper
        """
        self.get_logger().info(f"Executing pick for object at: X={grasp_pose['x']:.1f}, Y={grasp_pose['y']:.1f}, Z={grasp_pose['z']:.1f}")
        
        # Check if coordinates are within limits
        if (self.x_limit[0] < grasp_pose['x'] < self.x_limit[1] and 
            self.y_limit[0] < grasp_pose['y'] < self.y_limit[1] and 
            self.z_limit[0] < grasp_pose['approach_z'] < self.z_limit[1]):
            
            # 1. Move to approach position (above the object)
            self.pick_and_place_state = "APPROACHING"
            x_approach = grasp_pose['x']
            y_approach = grasp_pose['y']
            z_approach = grasp_pose['approach_z']
            
            # Command to move to approach position
            self.send_grbl_command(f"$G1X{x_approach:.1f}Y{y_approach:.1f}Z{z_approach:.1f}F6000")
            time.sleep(3)
            
            # 2. Move to grasp position
            self.pick_and_place_state = "GRASPING"
            z_grasp = grasp_pose['z']
            self.send_grbl_command(f"$G1Z{z_grasp:.1f}F3000")
            time.sleep(2)
            
            # 3. Close gripper (placeholder for actual gripper command)
            self.send_grbl_command("M42P8")  # Example: Activate digital output 8 for gripper
            time.sleep(1)
            
            # 4. Lift object
            self.pick_and_place_state = "LIFTING"
            self.send_grbl_command(f"$G1Z{z_approach:.1f}F3000")
            time.sleep(2)
            
            # 5. Move to place position (predefined location)
            self.pick_and_place_state = "PLACING"
            place_x = 0  # Center position for X
            place_y = 0  # Center position for Y
            self.send_grbl_command(f"$G1X{place_x:.1f}Y{place_y:.1f}F6000")
            time.sleep(3)
            
            # 6. Lower to place position
            place_z = 20  # Slightly above the surface
            self.send_grbl_command(f"$G1Z{place_z:.1f}F3000")
            time.sleep(2)
            
            # 7. Open gripper
            self.send_grbl_command("M43P8")  # Example: Deactivate digital output 8 for gripper
            time.sleep(1)
            
            # 8. Return to home position
            self.send_grbl_command("$G28")
            self.pick_and_place_state = "IDLE"
            
            return True
        else:
            self.get_logger().warn(f"Detected object is out of the machine physical limits")
            return False

    def pointcloud_callback(self, pointcloud_msg):
        """Process point cloud and execute pick and place sequence"""
        
        # Skip if already executing a pick and place operation
        if self.pick_and_place_state != "IDLE":
            self.get_logger().info(f"Skipping frame, current state: {self.pick_and_place_state}")
            return
            
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
            return
        
        # STEP 2: Segment objects in the point cloud
        object_clusters = self.segment_objects(filtered_points)
        
        if not object_clusters:
            self.get_logger().warn("No objects detected in the point cloud")
            return
            
        # STEP 3: Find closest object
        closest_object = object_clusters[0]  # Already sorted by distance
        
        # STEP 4: Verify detection consistency
        if not self.verify_detection(closest_object):
            self.get_logger().warn("Object detection failed verification, skipping this frame")
            return
            
        # STEP 5: Determine grasp pose
        grasp_pose = self.determine_grasp_pose(closest_object)
        
        self.get_logger().info(f"Found closest object with {closest_object['num_points']} points at position: "
                             f"X={grasp_pose['x']:.1f}, Y={grasp_pose['y']:.1f}, Z={grasp_pose['z']:.1f}")
        
        # STEP 6: Execute pick and place operation
        self.execute_pick_and_place(grasp_pose)


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedPickAndPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()