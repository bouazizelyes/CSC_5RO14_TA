#!/usr/bin/env python3

import csv
import os
from datetime import datetime

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image


class MappingLoggerNode:
    def __init__(self):
        rospy.init_node("mapping_logger_node", anonymous=False)

        self.image_topic = rospy.get_param("~image_topic", "/tello/camera/image_raw")
        self.pose_topic = rospy.get_param("~pose_topic", "/vrpn_client_node/tello/pose")
        self.output_root = rospy.get_param("~output_root", os.path.join(os.path.expanduser("~"), "tello_mapping_data"))
        self.capture_fps = float(rospy.get_param("~capture_fps", 2.0))
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 90))

        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(self.output_root, now)
        self.images_dir = os.path.join(self.session_dir, "images")
        os.makedirs(self.images_dir, exist_ok=True)

        self.csv_path = os.path.join(self.session_dir, "poses.csv")
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "seq",
                "stamp",
                "image_file",
                "px",
                "py",
                "pz",
                "qx",
                "qy",
                "qz",
                "qw",
            ]
        )

        self.bridge = CvBridge()
        self.latest_pose = None
        self.last_capture = rospy.Time(0)
        self.seq = 0

        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_cb, queue_size=10)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self._image_cb, queue_size=1)

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("mapping_logger_node started")
        rospy.loginfo("image_topic=%s", self.image_topic)
        rospy.loginfo("pose_topic=%s", self.pose_topic)
        rospy.loginfo("session_dir=%s", self.session_dir)

    def _pose_cb(self, msg):
        self.latest_pose = msg

    def _image_cb(self, msg):
        if self.latest_pose is None:
            rospy.logwarn_throttle(5.0, "No pose received yet, skipping image capture")
            return

        period = 1.0 / max(0.1, self.capture_fps)
        if (msg.header.stamp - self.last_capture).to_sec() < period:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "cv_bridge conversion failed: %s", exc)
            return

        filename = f"img_{self.seq:06d}.jpg"
        filepath = os.path.join(self.images_dir, filename)

        ok = cv2.imwrite(filepath, frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            rospy.logwarn("Failed to write image: %s", filepath)
            return

        p = self.latest_pose.pose.position
        q = self.latest_pose.pose.orientation
        stamp = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()

        self.csv_writer.writerow(
            [
                self.seq,
                f"{stamp:.6f}",
                filename,
                f"{p.x:.6f}",
                f"{p.y:.6f}",
                f"{p.z:.6f}",
                f"{q.x:.6f}",
                f"{q.y:.6f}",
                f"{q.z:.6f}",
                f"{q.w:.6f}",
            ]
        )
        self.csv_file.flush()

        self.last_capture = msg.header.stamp
        self.seq += 1

        rospy.loginfo_throttle(2.0, "Captured %d frames", self.seq)

    def _on_shutdown(self):
        try:
            self.csv_file.close()
        except Exception:
            pass

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    node = MappingLoggerNode()
    node.spin()
