#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        
        # Camera intrinsics (real camera calibration)
        self.camera_matrix = np.array([
            [554.254691191187, 0.0, 320.5],
            [0.0, 554.254691191187, 240.5],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros((4,1))
        
        # ArUco dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Set up QoS profile for better image streaming
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers and Publishers
        self.image_sub = self.create_subscription(
            Image,
            '/depth_camera/image_raw',
            self.image_callback,
            qos_profile
        )
        
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/aruco_pose',
            10
        )
        
        self.get_logger().info('ArUco Detector Node has been started')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect ArUco markers
            corners, ids, rejected = cv2.aruco.detectMarkers(
                cv_image,
                self.aruco_dict,
                parameters=self.aruco_params
            )
            
            # If markers are detected
            if ids is not None:
                # Estimate pose for each marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    1,  # Marker size in meters
                    self.camera_matrix,
                    self.dist_coeffs
                )
                
                # Create and publish pose message for each detected marker
                for i in range(len(ids)):
                    pose_msg = PoseStamped()
                    pose_msg.header = msg.header
                    
                    # Convert rotation vector to quaternion
                    rot_matrix, _ = cv2.Rodrigues(rvecs[i])
                    quat = self.rotation_matrix_to_quaternion(rot_matrix)
                    
                    # Set position
                    pose_msg.pose.position.x = float(tvecs[i][0][0])
                    pose_msg.pose.position.y = float(tvecs[i][0][1])
                    pose_msg.pose.position.z = float(tvecs[i][0][2])
                    x1= round(pose_msg.pose.position.x,2)
                    y1= round(pose_msg.pose.position.y,2)
                    z1= round(pose_msg.pose.position.z,2)
                    
                    # Set orientation
                    pose_msg.pose.orientation.x = float(quat[0])
                    pose_msg.pose.orientation.y = float(quat[1])
                    pose_msg.pose.orientation.z = float(quat[2])
                    pose_msg.pose.orientation.w = float(quat[3])
                    x2= round(pose_msg.pose.orientation.x,2)
                    y2= round(pose_msg.pose.orientation.y,2)
                    z2= round(pose_msg.pose.orientation.z,2)
                    w2= round(pose_msg.pose.orientation.w,2)
                    
                    self.pose_pub.publish(pose_msg)
                    self.get_logger().info(f'Published pose for marker ID: {ids[i][0]}')
                    self.get_logger().info(f'Position is (x,y,z): ({x1},{y1},{z1}), Orientation is (x,y,z,w): ({x2},{y2},{z2},{w2})')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def rotation_matrix_to_quaternion(self, rot_matrix):
        tr = np.trace(rot_matrix)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (rot_matrix[2,1] - rot_matrix[1,2]) / S
            qy = (rot_matrix[0,2] - rot_matrix[2,0]) / S
            qz = (rot_matrix[1,0] - rot_matrix[0,1]) / S
        else:
            if rot_matrix[0,0] > rot_matrix[1,1] and rot_matrix[0,0] > rot_matrix[2,2]:
                S = np.sqrt(1.0 + rot_matrix[0,0] - rot_matrix[1,1] - rot_matrix[2,2]) * 2
                qw = (rot_matrix[2,1] - rot_matrix[1,2]) / S
                qx = 0.25 * S
                qy = (rot_matrix[0,1] + rot_matrix[1,0]) / S
                qz = (rot_matrix[0,2] + rot_matrix[2,0]) / S
            elif rot_matrix[1,1] > rot_matrix[2,2]:
                S = np.sqrt(1.0 + rot_matrix[1,1] - rot_matrix[0,0] - rot_matrix[2,2]) * 2
                qw = (rot_matrix[0,2] - rot_matrix[2,0]) / S
                qx = (rot_matrix[0,1] + rot_matrix[1,0]) / S
                qy = 0.25 * S
                qz = (rot_matrix[1,2] + rot_matrix[2,1]) / S
            else:
                S = np.sqrt(1.0 + rot_matrix[2,2] - rot_matrix[0,0] - rot_matrix[1,1]) * 2
                qw = (rot_matrix[1,0] - rot_matrix[0,1]) / S
                qx = (rot_matrix[0,2] + rot_matrix[2,0]) / S
                qy = (rot_matrix[1,2] + rot_matrix[2,1]) / S
                qz = 0.25 * S
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()