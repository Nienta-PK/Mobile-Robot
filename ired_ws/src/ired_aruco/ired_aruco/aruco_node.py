#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int32MultiArray, String
from cv_bridge import CvBridge
import numpy as np
import cv2

# Map string -> constant (works for both APIs)
DICT_MAP = {name: getattr(cv2.aruco, name) for name in dir(cv2.aruco) if name.startswith('DICT_')}

def rvec_tvec_to_pose(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    tr = np.trace(R)
    qw = np.sqrt(max(0.0, 1.0 + tr)) / 2.0
    if qw == 0.0:
        qw = 1e-9
    qx = (R[2,1] - R[1,2]) / (4.0 * qw)
    qy = (R[0,2] - R[2,0]) / (4.0 * qw)
    qz = (R[1,0] - R[0,1]) / (4.0 * qw)
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, tvec)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = float(qx), float(qy), float(qz), float(qw)
    return p

class IredArucoNode(Node):
    def __init__(self):
        super().__init__('ired_aruco_node')
        # --- Parameters
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('debug_image_topic', '/ired_aruco/image')
        self.declare_parameter('pose_array_topic', '/ired_aruco/poses')
        self.declare_parameter('ids_topic', '/ired_aruco/ids')
        self.declare_parameter('text_topic', '/ired_aruco/pose_text')
        self.declare_parameter('aruco_dictionary', 'DICT_5X5_250')
        self.declare_parameter('marker_length_m', 0.05)  # meters
        self.declare_parameter('draw_axes', True)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('queue_size', 5)

        self.image_topic       = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value
        self.pose_array_topic  = self.get_parameter('pose_array_topic').value
        self.ids_topic         = self.get_parameter('ids_topic').value
        self.text_topic        = self.get_parameter('text_topic').value
        self.marker_length     = float(self.get_parameter('marker_length_m').value)
        dict_name              = self.get_parameter('aruco_dictionary').value
        self.draw_axes         = bool(self.get_parameter('draw_axes').value)
        self.publish_debug     = bool(self.get_parameter('publish_debug_image').value)
        queue_size             = int(self.get_parameter('queue_size').value)

        if dict_name not in DICT_MAP:
            self.get_logger().warn(f"Unknown ArUco dict '{dict_name}', defaulting to DICT_5X5_250")
            dict_name = 'DICT_5X5_250'

        # ---- API compatibility: new (ArucoDetector) vs legacy (detectMarkers)
        self.has_new_api = hasattr(cv2.aruco, "ArucoDetector")
        if self.has_new_api:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_MAP[dict_name])
            self.detector_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)
        else:
            self.aruco_dict = cv2.aruco.Dictionary_get(DICT_MAP[dict_name])
            self.detector_params = cv2.aruco.DetectorParameters_create()
            self.detector = None  # we will call cv2.aruco.detectMarkers directly

        # QoS: Best Effort for images
        qos_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=queue_size
        )
        qos_default = QoSProfile(depth=queue_size)

        # Publishers
        self.pose_pub  = self.create_publisher(PoseArray, self.pose_array_topic, qos_default)
        self.ids_pub   = self.create_publisher(Int32MultiArray, self.ids_topic, qos_default)
        self.text_pub  = self.create_publisher(String, self.text_topic, qos_default)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, qos_img) if self.publish_debug else None

        # Subscribers
        self.bridge = CvBridge()
        self.K = None
        self.D = None

        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, qos_default)
        self.create_subscription(Image, self.image_topic, self._image_cb, qos_img)

        api_str = "new ArucoDetector API" if self.has_new_api else "legacy detectMarkers API"
        self.get_logger().info(f"Using {api_str}")
        self.get_logger().info(f"Subscribing: {self.image_topic} & {self.camera_info_topic}")
        self.get_logger().info(f"Publishing: poses={self.pose_array_topic}, ids={self.ids_topic}, text={self.text_topic}, debug={self.debug_image_topic if self.publish_debug else '(off)'}")
        self.get_logger().info(f"Dict={dict_name}, marker_length={self.marker_length} m")

    def _camera_info_cb(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3,3)
        self.D = np.array(msg.d, dtype=np.float64)

    def _detect(self, img):
        if self.has_new_api:
            return self.detector.detectMarkers(img)
        else:
            # Legacy API
            return cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.detector_params)

    def _image_cb(self, msg: Image):
        if self.K is None or self.D is None:
            return  # wait for intrinsics
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        corners, ids, rejected = self._detect(cv_img)

        poses = PoseArray()
        poses.header = msg.header
        id_list = Int32MultiArray()
        lines = []

        if ids is not None and len(ids) > 0:
            rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.K, self.D
            )
            for i, mid in enumerate(ids.flatten().tolist()):
                pose = rvec_tvec_to_pose(rvecs[i][0], tvecs[i][0])
                poses.poses.append(pose)
                id_list.data.append(int(mid))
                line = (f"[Aruco] id={mid} "
                        f"pos=({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}) "
                        f"quat=({pose.orientation.x:.4f}, {pose.orientation.y:.4f}, {pose.orientation.z:.4f}, {pose.orientation.w:.4f}) "
                        f"frame='{poses.header.frame_id}'")
                lines.append(line)
                self.get_logger().info(line)

                if self.publish_debug:
                    cv2.aruco.drawDetectedMarkers(cv_img, [corners[i]])
                    if self.draw_axes:
                        cv2.drawFrameAxes(cv_img, self.K, self.D, rvecs[i][0], tvecs[i][0], self.marker_length * 0.5)

        if poses.poses:
            self.pose_pub.publish(poses)
            self.ids_pub.publish(id_list)
            self.text_pub.publish(String(data="\n".join(lines)))

        if self.publish_debug:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8'))

def main():
    rclpy.init()
    node = IredArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
