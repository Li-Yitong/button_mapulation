#!/usr/bin/env python3
"""ROS2 chessboard pose publisher for eye-in-hand calibration workflows."""
import math
from typing import Optional

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf_transformations import quaternion_from_matrix
from tf2_ros.transform_broadcaster import TransformBroadcaster


class ChessboardPosePublisher(Node):
    """Detects a chessboard in camera images and publishes its pose as TF."""

    def __init__(self) -> None:
        super().__init__('chessboard_pose_publisher')

        # Declare ROS parameters with sensible defaults
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('pattern_rows', 5)
        self.declare_parameter('pattern_cols', 7)
        self.declare_parameter('square_size', 0.03)  # meters
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('board_frame', 'checkerboard')
        self.declare_parameter('publish_visual_pose', True)
        self.declare_parameter('show_image', True)  # 是否显示图像窗口

        self.bridge = CvBridge()
        self.camera_info_msg: Optional[CameraInfo] = None

        # Pre-compute chessboard object points in the board's coordinate system
        self.pattern_rows = int(self.get_parameter('pattern_rows').value)
        self.pattern_cols = int(self.get_parameter('pattern_cols').value)
        self.square_size = float(self.get_parameter('square_size').value)
        self.obj_points = self._create_object_points()

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.board_frame = self.get_parameter('board_frame').value
        self.publish_visual_pose = self.get_parameter('publish_visual_pose').value
        self.show_image = self.get_parameter('show_image').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, 'chessboard_pose', 10)

        self.camera_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_callback, 10)
        self.image_sub = self.create_subscription(Image, image_topic, self._image_callback, 10)
        
        # 日志计数器（用于控制日志频率）
        self._log_counter = 0
        self._log_interval = 30  # 每30帧打印一次
        
        # 图像可视化
        if self.show_image:
            cv2.namedWindow('Chessboard Detection', cv2.WINDOW_AUTOSIZE)

        self.get_logger().info(
            f"Chessboard detector running | pattern {self.pattern_cols}x{self.pattern_rows}, square {self.square_size*1000:.1f} mm"
        )
        if self.show_image:
            self.get_logger().info(f"  图像窗口已启动 - 按 'q' 关闭")


    def _create_object_points(self) -> np.ndarray:
        """Create 3D coordinates for each chessboard corner (z=0 plane)."""
        objp = np.zeros((self.pattern_rows * self.pattern_cols, 3), np.float32)
        grid = np.mgrid[0:self.pattern_cols, 0:self.pattern_rows].T.reshape(-1, 2)
        objp[:, :2] = grid * self.square_size
        return objp

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info_msg = msg

    def _image_callback(self, msg: Image) -> None:
        if self.camera_info_msg is None:
            self._log_counter += 1
            if self._log_counter % self._log_interval == 0:
                self.get_logger().info('Waiting for CameraInfo...')
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  # 改为彩色
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge failed: {exc}')
            return
        
        # 转灰度用于检测
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        pattern_size = (self.pattern_cols, self.pattern_rows)
        found, corners = cv2.findChessboardCorners(
            gray, pattern_size, 
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        
        # 可视化
        if self.show_image:
            vis_img = cv_image.copy()
            if found:
                cv2.drawChessboardCorners(vis_img, pattern_size, corners, found)
                # 添加文本提示
                cv2.putText(vis_img, "Chessboard Detected!", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(vis_img, f"Distance: {self.camera_info_msg.k[0]:.3f}m", (10, 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(vis_img, "Searching for chessboard...", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            cv2.imshow('Chessboard Detection', vis_img)
            key = cv2.waitKey(1)
            if key == ord('q'):
                self.get_logger().info("用户关闭图像窗口")
                cv2.destroyAllWindows()
                self.show_image = False

        if not found:
            self._log_counter += 1
            if self._log_counter % self._log_interval == 0:
                self.get_logger().info('Chessboard not detected')
            return

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        camera_matrix = np.array(self.camera_info_msg.k, dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.array(self.camera_info_msg.d, dtype=np.float64)

        success, rvec, tvec = cv2.solvePnP(self.obj_points, corners, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
        if not success:
            self.get_logger().warn('solvePnP failed')
            return

        rot_matrix, _ = cv2.Rodrigues(rvec)
        quaternion = quaternion_from_matrix(np.vstack((
            np.hstack((rot_matrix, np.array([[0], [0], [0]]))),
            np.array([0, 0, 0, 1])
        )))

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.camera_frame or msg.header.frame_id

        transform = TransformStamped()
        transform.header = header
        transform.child_frame_id = self.board_frame
        transform.transform.translation.x = float(tvec[0])
        transform.transform.translation.y = float(tvec[1])
        transform.transform.translation.z = float(tvec[2])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])

        self.tf_broadcaster.sendTransform(transform)

        if self.publish_visual_pose:
            pose_msg = PoseStamped()
            pose_msg.header = header
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            pose_msg.pose.orientation = transform.transform.rotation
            self.pose_pub.publish(pose_msg)
        
        # 每次成功检测到棋盘格时重置计数器
        self._log_counter = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChessboardPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
