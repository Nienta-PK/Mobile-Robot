#!/usr/bin/env python3
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import message_filters

import numpy as np
import cv2
from aruco_pose_estimation.utils import ARUCO_DICT
from aruco_pose_estimation.pose_estimation import pose_estimation

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseArray
from aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Header, String

# TF
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        self.initialize_parameters()

        # Validate dictionary
        try:
            dictionary_id = cv2.aruco.__getattribute__(self.dictionary_id_name)
            if dictionary_id not in ARUCO_DICT.values():
                raise AttributeError
        except AttributeError:
            self.get_logger().error(f"bad aruco_dictionary_id: {self.dictionary_id_name}")
            options = "\n".join([s for s in ARUCO_DICT])
            self.get_logger().error(f"valid options:\n{options}")

        # Subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo, self.info_topic, self.info_callback, qos_profile_sensor_data
        )

        if bool(self.use_depth_input):
            self.image_sub = message_filters.Subscriber(self, Image, self.image_topic,
                                                        qos_profile=qos_profile_sensor_data)
            self.depth_image_sub = message_filters.Subscriber(self, Image, self.depth_image_topic,
                                                              qos_profile=qos_profile_sensor_data)
            self.synchronizer = message_filters.ApproximateTimeSynchronizer(
                [self.image_sub, self.depth_image_sub], queue_size=30, slop=0.2
            )
            self.synchronizer.registerCallback(self.rgb_depth_sync_callback)
        else:
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self.image_callback, qos_profile_sensor_data
            )

        # QoS: transient local for id/pose topics so late subscribers see latest immediately
        self.latched_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        # Image is usually high-rate sensor data; keep it volatile unless user opts in
        # self.image_qos = self.latched_qos if self.latch_image else qos_profile_sensor_data
        if self.latch_image:
            self.latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Publishers
        self.poses_pub   = self.create_publisher(PoseArray, self.markers_visualization_topic, self.latched_qos)
        self.markers_pub = self.create_publisher(ArucoMarkers, self.detected_markers_topic, self.latched_qos)
        self.image_pub   = self.create_publisher(Image, self.output_image_topic, self.latched_qos)

        # Heartbeat to confirm continuity (once/sec with last seq)
        self.heartbeat_pub = self.create_publisher(String, f"{self.node_namespace}/aruco_heartbeat", self.latched_qos)
        self.last_seq = 0
        self.create_timer(1.0, self._heartbeat_tick)

        # Camera params
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        # ArUco detector (OpenCV 4.7+)
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector   = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)

        self.bridge = CvBridge()

        # TF broadcaster for per-marker frames
        self.tf_broadcaster = TransformBroadcaster(self)

    @property
    def node_namespace(self):
        # make a small, safe namespace for heartbeat topic
        ns = self.get_namespace()
        return ns if ns and ns != "/" else ""

    def _heartbeat_tick(self):
        msg = String()
        msg.data = f"aruco_node alive; last_seq={self.last_seq}"
        self.heartbeat_pub.publish(msg)

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion    = np.array(self.info_msg.d)

        self.get_logger().info("Camera info received.")
        self.get_logger().info(f"Intrinsic matrix:\n{self.intrinsic_mat}")
        self.get_logger().info(f"Distortion coefficients: {self.distortion}")
        self.get_logger().info(f"Camera frame: {self.info_msg.width}x{self.info_msg.height}")

        # Assume camera parameters remain constant
        self.destroy_subscription(self.info_sub)

    def _fill_headers(self, out_msg, stamp, frame_fallback=""):
        if self.camera_frame == "":
            out_msg.header.frame_id = self.info_msg.header.frame_id if self.info_msg else frame_fallback
        else:
            out_msg.header.frame_id = self.camera_frame
        out_msg.header.stamp = stamp

    def _publish_tf_for_markers(self, markers: ArucoMarkers):
        """
        Publish a TF for each detected marker: child_frame_id = <marker_frame_prefix><id>
        Transform is expressed relative to camera_frame (or CameraInfo frame if camera_frame == "").
        """
        parent = self.camera_frame if self.camera_frame != "" else (self.info_msg.header.frame_id if self.info_msg else "camera")
        for i, mid in enumerate(markers.marker_ids):
            if i >= len(markers.poses):
                continue
            p = markers.poses[i].position
            q = markers.poses[i].orientation
            t = TransformStamped()
            t.header.stamp = markers.header.stamp
            t.header.frame_id = parent
            t.child_frame_id = f"{self.marker_frame_prefix}{mid}"
            t.transform.translation.x = p.x
            t.transform.translation.y = p.y
            t.transform.translation.z = p.z
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

    def image_callback(self, img_msg: Image):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")
        markers = ArucoMarkers()
        pose_array = PoseArray()

        self._fill_headers(markers, img_msg.header.stamp, "camera")
        self._fill_headers(pose_array, img_msg.header.stamp, "camera")

        frame, pose_array, markers = pose_estimation(
            rgb_frame=cv_image, depth_frame=None,
            aruco_detector=self.aruco_detector,
            marker_size=self.marker_size,
            matrix_coefficients=self.intrinsic_mat,
            distortion_coefficients=self.distortion,
            pose_array=pose_array, markers=markers
        )

        # Publish id/pose/orientation (Pose contains orientation quaternion)
        if len(markers.marker_ids) > 0:
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)
            # Also publish TF frames so RViz can render axes
            if self.publish_tf:
                self._publish_tf_for_markers(markers)
        else:
            # publish empties so downstream nodes/RViz see continuity
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)
        # Publish annotated image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))
        self.last_seq += 1

    def rgb_depth_sync_callback(self, rgb_msg: Image, depth_msg: Image):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
        cv_image       = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")

        markers = ArucoMarkers()
        pose_array = PoseArray()
        self._fill_headers(markers, rgb_msg.header.stamp, "camera")
        self._fill_headers(pose_array, rgb_msg.header.stamp, "camera")

        frame, pose_array, markers = pose_estimation(
            rgb_frame=cv_image, depth_frame=cv_depth_image,
            aruco_detector=self.aruco_detector,
            marker_size=self.marker_size,
            matrix_coefficients=self.intrinsic_mat,
            distortion_coefficients=self.distortion,
            pose_array=pose_array, markers=markers
        )

        if len(markers.marker_ids) > 0:
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)
            if self.publish_tf:
                self._publish_tf_for_markers(markers)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))
        self.last_seq += 1

    def initialize_parameters(self):
        # Existing params
        self.declare_parameter("marker_size", 0.0625,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description="Size of the markers in meters.")
        )
        self.declare_parameter("aruco_dictionary_id", "DICT_5X5_250",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="Dictionary that was used to generate markers.")
        )
        self.declare_parameter("use_depth_input", True,
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description="Use depth camera input for pose estimation instead of RGB image")
        )
        self.declare_parameter("image_topic", "/camera/image_raw",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="Image topic to subscribe to.")
        )
        self.declare_parameter("depth_image_topic", "/camera/depth/image_raw",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="Depth camera topic to subscribe to.")
        )
        self.declare_parameter("camera_info_topic", "/camera/camera_info",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="Camera info topic to subscribe to.")
        )
        self.declare_parameter("camera_frame", "",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="Camera optical frame to use.")
        )
        self.declare_parameter("detected_markers_topic", "/aruco_markers",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="Topic to publish detected markers as array of marker ids and poses")
        )
        self.declare_parameter("markers_visualization_topic", "/aruco_poses",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="Topic to publish markers as pose array")
        )
        self.declare_parameter("output_image_topic", "/aruco_image",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="Topic to publish annotated images with markers drawn on them")
        )

        # New params
        self.declare_parameter("publish_tf", True,
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description="Publish a TF frame per detected marker")
        )
        self.declare_parameter("marker_frame_prefix", "aruco_",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="Prefix for marker TF child_frame_id")
        )
        self.declare_parameter("latch_image", False,
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description="Use TRANSIENT_LOCAL durability for the annotated image topic")
        )

        # Read back
        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        self.get_logger().info(f"Marker size: {self.marker_size}")

        self.dictionary_id_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        self.get_logger().info(f"Marker type: {self.dictionary_id_name}")

        self.use_depth_input = self.get_parameter("use_depth_input").get_parameter_value().bool_value
        self.get_logger().info(f"Use depth input: {self.use_depth_input}")

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.get_logger().info(f"Input image topic: {self.image_topic}")

        self.depth_image_topic = self.get_parameter("depth_image_topic").get_parameter_value().string_value
        self.get_logger().info(f"Input depth image topic: {self.depth_image_topic}")

        self.info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.get_logger().info(f"Image camera info topic: {self.info_topic}")

        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.get_logger().info(f"Camera frame: {self.camera_frame}")

        self.detected_markers_topic = self.get_parameter("detected_markers_topic").get_parameter_value().string_value
        self.markers_visualization_topic = self.get_parameter("markers_visualization_topic").get_parameter_value().string_value
        self.output_image_topic = self.get_parameter("output_image_topic").get_parameter_value().string_value

        self.publish_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value
        self.marker_frame_prefix = self.get_parameter("marker_frame_prefix").get_parameter_value().string_value
        self.latch_image = self.get_parameter("latch_image").get_parameter_value().bool_value

def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
