#!/usr/bin/env python3

import math
from dataclasses import dataclass

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


@dataclass
class Track:
    cx: int
    cy: int
    last_seen: rospy.Time


class CentroidTracker:
    def __init__(self, max_distance_px=80.0, max_age_sec=1.0):
        self.max_distance_px = float(max_distance_px)
        self.max_age = rospy.Duration.from_sec(float(max_age_sec))
        self.next_id = 1
        self.tracks = {}

    def _distance(self, c1, c2):
        return math.hypot(float(c1[0] - c2[0]), float(c1[1] - c2[1]))

    def update(self, detections, stamp):
        tracked = []
        used_track_ids = set()

        for det in detections:
            x1, y1, x2, y2 = det["bbox"]
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            best_track_id = None
            best_dist = self.max_distance_px

            for track_id, track in self.tracks.items():
                if track_id in used_track_ids:
                    continue
                if stamp - track.last_seen > self.max_age:
                    continue
                dist = self._distance((cx, cy), (track.cx, track.cy))
                if dist <= best_dist:
                    best_dist = dist
                    best_track_id = track_id

            is_new = best_track_id is None
            if is_new:
                best_track_id = self.next_id
                self.next_id += 1

            self.tracks[best_track_id] = Track(cx=cx, cy=cy, last_seen=stamp)
            used_track_ids.add(best_track_id)
            tracked.append(
                {
                    "id": best_track_id,
                    "bbox": det["bbox"],
                    "label": det.get("label", "obj"),
                    "confidence": det.get("confidence", 0.0),
                    "is_new": is_new,
                }
            )

        stale = [track_id for track_id, track in self.tracks.items() if stamp - track.last_seen > self.max_age]
        for track_id in stale:
            self.tracks.pop(track_id, None)

        return tracked


class TelloVisionNode:
    def __init__(self):
        rospy.init_node("tello_vision_node", anonymous=False)

        self.bridge = CvBridge()
        self.show_debug = rospy.get_param("~show_debug", True)
        self.image_topic = rospy.get_param("~image_topic", "/tello/image_raw")
        self.count_topic = rospy.get_param("~count_topic", "/tello/object_count")
        self.detector_backend = rospy.get_param("~detector_backend", "hog").strip().lower()
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.35))
        self.max_track_distance_px = float(rospy.get_param("~max_track_distance_px", 90.0))
        self.track_max_age_sec = float(rospy.get_param("~track_max_age_sec", 1.2))

        self.tracker = CentroidTracker(
            max_distance_px=self.max_track_distance_px,
            max_age_sec=self.track_max_age_sec,
        )

        self.total_count = 0
        self.seen_track_ids = set()
        self.latest_annotated = None

        self.count_pub = rospy.Publisher(self.count_topic, Int32, queue_size=10)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self._image_cb, queue_size=1)

        self.hog = None
        self.yolo_model = None
        self._init_detector()

        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo("tello_vision_node started with backend=%s, image_topic=%s", self.detector_backend, self.image_topic)

    def _init_detector(self):
        if self.detector_backend == "yolo":
            model_path = rospy.get_param("~yolo_model", "yolov8n.pt")
            try:
                from ultralytics import YOLO  # pylint: disable=import-error

                self.yolo_model = YOLO(model_path)
                rospy.loginfo("Loaded YOLO model from '%s'", model_path)
                return
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn("YOLO backend requested but unavailable (%s). Falling back to HOG.", exc)
                self.detector_backend = "hog"

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def _detect_hog(self, frame):
        detections = []
        boxes, _weights = self.hog.detectMultiScale(
            frame,
            winStride=(8, 8),
            padding=(8, 8),
            scale=1.05,
        )
        for (x, y, w, h) in boxes:
            detections.append(
                {
                    "bbox": (int(x), int(y), int(x + w), int(y + h)),
                    "label": "person",
                    "confidence": 1.0,
                }
            )
        return detections

    def _detect_yolo(self, frame):
        detections = []
        results = self.yolo_model(frame, conf=self.min_confidence, verbose=False)
        if not results:
            return detections

        names = results[0].names
        for box in results[0].boxes:
            conf = float(box.conf[0])
            if conf < self.min_confidence:
                continue
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
            cls_idx = int(box.cls[0])
            label = names.get(cls_idx, str(cls_idx)) if isinstance(names, dict) else str(cls_idx)
            detections.append(
                {
                    "bbox": (x1, y1, x2, y2),
                    "label": label,
                    "confidence": conf,
                }
            )
        return detections

    def _detect_objects(self, frame):
        if self.detector_backend == "yolo" and self.yolo_model is not None:
            return self._detect_yolo(frame)
        return self._detect_hog(frame)

    def _draw(self, frame, tracks):
        for track in tracks:
            x1, y1, x2, y2 = track["bbox"]
            color = (0, 220, 0) if not track["is_new"] else (0, 160, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            text = "{}#{} {:.2f}".format(track["label"], track["id"], track["confidence"])
            cv2.putText(frame, text, (x1, max(20, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        status = "Total unique objects: {}".format(self.total_count)
        cv2.putText(frame, status, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "cv_bridge conversion failed: %s", exc)
            return

        detections = self._detect_objects(frame)
        tracks = self.tracker.update(detections, rospy.Time.now())

        for track in tracks:
            if track["is_new"] and track["id"] not in self.seen_track_ids:
                self.seen_track_ids.add(track["id"])
                self.total_count += 1

        self.count_pub.publish(Int32(data=self.total_count))

        self._draw(frame, tracks)
        self.latest_annotated = frame

        rospy.loginfo_throttle(
            2.0,
            "detections=%d tracked=%d total_count=%d",
            len(detections),
            len(tracks),
            self.total_count,
        )

        if self.show_debug:
            cv2.imshow("tello_vision_node", frame)
            cv2.waitKey(1)

    def spin(self):
        rate_hz = float(rospy.get_param("~loop_rate_hz", 30.0))
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            rate.sleep()

    def _on_shutdown(self):
        if self.show_debug:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    node = TelloVisionNode()
    node.spin()
