#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from std_msgs.msg import Header
from calibration_loader import load_camera_calibration, pixel_to_camera_frame, estimate_distance_from_bbox

class RadarEmulator(Node):

    def __init__(self):
        super().__init__('radar_emulator')
        
        # Load camera calibration parameters
        self.camera_matrix, self.dist_coeffs, self.transformation_matrix = load_camera_calibration()
        self.get_logger().info("Loaded camera calibration parameters")
        
        # Subscribe to the YOLO detections
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/yolo_detections',
            self.detection_callback,
            10
        )
        
        # Create a publisher for the radar PointCloud2 data
        self.radar_pub = self.create_publisher(PointCloud2, '/radar_points', 10)
        
        # Realistic radar parameters (based on typical automotive radar specs)
        self.range_accuracy = 0.15  # ±15cm
        self.azimuth_accuracy = 0.2  # ±0.2 degrees
        self.velocity_accuracy = 0.1  # ±0.1 m/s
        
        self.get_logger().info("Radar Emulator node started with production-grade calibration")

    def detection_callback(self, msg):
        # Create PointCloud2 message with proper header
        radar_msg = PointCloud2()
        radar_msg.header = msg.header  # Use the same timestamp as the incoming message
        
        # FIX: Use a standard frame ID that RViz2 recognizes
        radar_msg.header.frame_id = "base_link"  # Changed from "radar_frame"
        
        # Define point structure
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='velocity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        radar_msg.fields = fields
        radar_msg.point_step = 20  # 5 fields * 4 bytes each
        radar_msg.is_dense = True
        radar_msg.height = 1

        point_data = []
        
        for detection in msg.detections:
            if not detection.results:
                continue
                
            # Get object class and confidence
            class_name = detection.results[0].hypothesis.class_id
            confidence = detection.results[0].hypothesis.score
            
            # Only process high-confidence detections
            if confidence < 0.5:
                continue
            
            # Get bounding box center
            center_x = detection.bbox.center.position.x
            center_y = detection.bbox.center.position.y
            bbox_width = detection.bbox.size_x
            
            # Production-grade distance estimation
            try:
                # Estimate distance from bounding box size
                distance = estimate_distance_from_bbox(bbox_width, class_name)
                
                # Add realistic noise based on radar specifications
                distance += np.random.normal(0.0, self.range_accuracy)
                
                # Convert pixel coordinates to camera coordinates using proper calibration
                x_cam, y_cam, z_cam = pixel_to_camera_frame(
                    center_x, center_y, distance, self.camera_matrix
                )
                
                # Add angular noise (simplified)
                x_cam += np.random.normal(0.0, self.azimuth_accuracy * distance / 10.0)
                y_cam += np.random.normal(0.0, self.azimuth_accuracy * distance / 10.0)
                
                # Transform from camera to radar coordinates (using calibration)
                # In this simplified version, we assume they're in the same position
                x_radar = x_cam
                y_radar = y_cam
                z_radar = z_cam
                
                # Simulate Doppler velocity with realistic noise
                velocity = np.random.uniform(-15.0, 15.0)  # Typical road speeds
                velocity += np.random.normal(0.0, self.velocity_accuracy)
                
                # Simulate radar reflectivity (intensity) based on object class
                intensity_map = {'car': 0.8, 'truck': 0.9, 'person': 0.3, 'bicycle': 0.4}
                intensity = intensity_map.get(class_name, 0.5)
                
                # Add point to cloud
                point_data.append([x_radar, y_radar, z_radar, velocity, intensity])
                
            except Exception as e:
                self.get_logger().error(f"Error processing detection: {e}")
                continue
        
        # Convert points to byte array
        if point_data:
            radar_msg.data = np.array(point_data, dtype=np.float32).tobytes()
            radar_msg.width = len(point_data)
        else:
            radar_msg.width = 0
            
        self.radar_pub.publish(radar_msg)
        self.get_logger().info(f"Published {radar_msg.width} radar points with production-grade simulation", 
                              throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = RadarEmulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Radar Emulator node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()