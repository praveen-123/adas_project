#!/usr/bin/env python3

import yaml
import numpy as np

def load_camera_calibration(calibration_file='simulations/camera_calibration.yaml'):
    """
    Load camera calibration parameters from a YAML file.
    In production systems, these values would come from physical measurement of the camera.
    """
    with open(calibration_file, 'r') as file:
        calib_data = yaml.safe_load(file)  # Load YAML data into a dictionary
    
    # Convert list data to numpy arrays with proper shape
    camera_matrix = np.array(calib_data['camera_matrix']['data']).reshape(3, 3)
    dist_coeffs = np.array(calib_data['distortion_coefficients']['data'])
    transformation_matrix = np.array(calib_data['transformation_matrix']['data']).reshape(4, 4)
    
    return camera_matrix, dist_coeffs, transformation_matrix

def pixel_to_camera_frame(u, v, depth, camera_matrix):
    """
    Convert pixel coordinates (u,v) to camera coordinates (x,y,z)
    using the pinhole camera model. This is the proper mathematical way to convert
    2D image points to 3D space when depth is known.
    """
    # Extract camera intrinsic parameters
    fx = camera_matrix[0, 0]  # Focal length in x-direction
    fy = camera_matrix[1, 1]  # Focal length in y-direction
    cx = camera_matrix[0, 2]  # Optical center x-coordinate
    cy = camera_matrix[1, 2]  # Optical center y-coordinate
    
    # Convert pixel coordinates to normalized camera coordinates
    # This reverses the perspective projection
    x_normalized = (u - cx) / fx
    y_normalized = (v - cy) / fy
    
    # Convert to 3D camera coordinates using the depth value
    x_cam = depth * x_normalized
    y_cam = depth * y_normalized
    z_cam = depth  # Depth is the distance along the camera's z-axis
    
    return x_cam, y_cam, z_cam

def estimate_distance_from_bbox(bbox_size, object_type='car'):
    """
    Estimate distance based on bounding box size and known object dimensions.
    This implements the basic principle: object size in image is inversely proportional to distance.
    """
    # Known typical real-world dimensions in meters
    object_dimensions = {
        'car': 1.8,       # Typical car width (meters)
        'truck': 2.5,     # Typical truck width
        'person': 0.5,    # Typical person width
        'bicycle': 0.7,   # Typical bicycle width
    }
    
    # Get the expected object width based on its class
    object_width = object_dimensions.get(object_type, 1.8)  # Default to car width if unknown
    
    # Simplified distance estimation formula:
    # distance = (real_world_size * focal_length) / size_in_image
    # We use a typical focal length value here; in production this would come from calibration
    focal_length = 500  # Typical value for many cameras (in pixels)
    distance = (object_width * focal_length) / bbox_size
    
    return distance