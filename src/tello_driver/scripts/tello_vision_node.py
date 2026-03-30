#!/usr/bin/env python3

import math
import os
from dataclasses import dataclass
from typing import Dict, Tuple

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


@dataclass
class Track:
    cx: int
    cy: int
    last_seen: rospy.Time


@dataclass
class UniqueObject:
    uid: int
    label: str
    centroid: Tuple[int, int]
    hist: np.ndarray
    first_seen: rospy.Time
    last_seen: rospy.Time
    seen_count: int
    counted: bool


@dataclass
class TrackLink:
    uid: int
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
        self.image_topic = rospy.get_param("~image_topic", "/tello/camera/image_raw")
        self.annotated_topic = rospy.get_param("~annotated_topic", "/tello/camera/detections")
        self.count_topic = rospy.get_param("~count_topic", "/tello/object_count")
        self.detector_backend = rospy.get_param("~detector_backend", "hog").strip().lower()
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.35))
        self.yolo_model = str(rospy.get_param("~yolo_model", "yolov8n.pt")).strip()
        self.yolo_allow_download = bool(rospy.get_param("~yolo_allow_download", False))
        self.yolo_device = str(rospy.get_param("~yolo_device", "auto")).strip().lower()
        self.yolo_imgsz = int(rospy.get_param("~yolo_imgsz", 416))
        self.yolo_max_det = int(rospy.get_param("~yolo_max_det", 20))
        self.yolo_person_only = bool(rospy.get_param("~yolo_person_only", True))
        self.process_every_n_frames = max(1, int(rospy.get_param("~process_every_n_frames", 2)))
        self.max_track_distance_px = float(rospy.get_param("~max_track_distance_px", 90.0))
        self.track_max_age_sec = float(rospy.get_param("~track_max_age_sec", 1.2))
        self.reid_max_age_sec = float(rospy.get_param("~reid_max_age_sec", 120.0))
        self.reid_max_distance_px = float(rospy.get_param("~reid_max_distance_px", 180.0))
        self.reid_max_hist_distance = float(rospy.get_param("~reid_max_hist_distance", 0.35))
        self.min_unique_confirmations = max(1, int(rospy.get_param("~min_unique_confirmations", 2)))

        self.tracker = CentroidTracker(
            max_distance_px=self.max_track_distance_px,
            max_age_sec=self.track_max_age_sec,
        )

        self.total_count = 0
        self.next_unique_id = 1
        self.unique_objects: Dict[int, UniqueObject] = {}
        self.track_links: Dict[int, TrackLink] = {}
        self.latest_annotated = None
        self.frame_index = 0

        self.count_pub = rospy.Publisher(self.count_topic, Int32, queue_size=10)
        self.annotated_pub = rospy.Publisher(self.annotated_topic, Image, queue_size=1)

        self.hog = None
        self.yolo_runtime = None
        self._init_detector()

        # Subscribe only after detector init to avoid early callbacks during startup.
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self._image_cb, queue_size=1)

        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo("tello_vision_node started with backend=%s, image_topic=%s", self.detector_backend, self.image_topic)

    def _init_detector(self):
        if self.detector_backend == "yolo":
            model_path = self._resolve_yolo_model_path(self.yolo_model)
            try:
                from ultralytics import YOLO  # pylint: disable=import-error
                import torch  # pylint: disable=import-error

                if self.yolo_device == "auto":
                    self.yolo_device = "cuda:0" if torch.cuda.is_available() else "cpu"
                if self.yolo_device.startswith("cuda") and not torch.cuda.is_available():
                    rospy.logwarn("CUDA requested but unavailable, using CPU.")
                    self.yolo_device = "cpu"

                self.yolo_runtime = YOLO(model_path)
                rospy.loginfo("Loaded YOLO model from '%s' on device '%s'", model_path, self.yolo_device)
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
        results = self.yolo_runtime.predict(
            source=frame,
            conf=self.min_confidence,
            device=self.yolo_device,
            imgsz=self.yolo_imgsz,
            max_det=self.yolo_max_det,
            classes=[0] if self.yolo_person_only else None,
            verbose=False,
        )
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

    def _crop_for_hist(self, frame, bbox):
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = bbox
        x1 = max(0, min(x1, w - 1))
        y1 = max(0, min(y1, h - 1))
        x2 = max(x1 + 1, min(x2, w))
        y2 = max(y1 + 1, min(y2, h))
        return frame[y1:y2, x1:x2]

    def _compute_hist(self, frame, bbox):
        roi = self._crop_for_hist(frame, bbox)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [16, 16], [0, 180, 0, 256])
        return cv2.normalize(hist, hist).flatten()

    def _register_unique(self, track, frame, stamp):
        x1, y1, x2, y2 = track["bbox"]
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        label = str(track.get("label", "obj"))
        hist = self._compute_hist(frame, track["bbox"])
        track_id = int(track.get("id", -1))

        # First preference: keep the same unique id for an existing tracker id.
        if track_id in self.track_links:
            link = self.track_links[track_id]
            uid = link.uid
            if uid in self.unique_objects:
                obj = self.unique_objects[uid]
                max_age = rospy.Duration.from_sec(self.reid_max_age_sec)
                if (stamp - obj.last_seen) <= max_age and obj.label == label:
                    obj.centroid = (cx, cy)
                    obj.hist = hist
                    obj.last_seen = stamp
                    obj.seen_count += 1
                    incremented_count = False
                    if (not obj.counted) and obj.seen_count >= self.min_unique_confirmations:
                        obj.counted = True
                        self.total_count += 1
                        incremented_count = True
                    self.unique_objects[uid] = obj
                    self.track_links[track_id] = TrackLink(uid=uid, last_seen=stamp)
                    track["unique_id"] = uid
                    track["is_new_unique"] = incremented_count
                    track["counted"] = obj.counted
                    return

        best_uid = None
        best_score = 10.0
        max_age = rospy.Duration.from_sec(self.reid_max_age_sec)

        for uid, obj in self.unique_objects.items():
            if obj.label != label:
                continue
            if stamp - obj.last_seen > max_age:
                continue

            d_centroid = math.hypot(float(cx - obj.centroid[0]), float(cy - obj.centroid[1]))
            if d_centroid > self.reid_max_distance_px:
                continue

            d_hist = float(cv2.compareHist(hist.astype(np.float32), obj.hist.astype(np.float32), cv2.HISTCMP_BHATTACHARYYA))
            if d_hist > self.reid_max_hist_distance:
                continue

            score = d_hist + (d_centroid / max(1.0, self.reid_max_distance_px))
            if score < best_score:
                best_score = score
                best_uid = uid

        is_new_unique = best_uid is None
        incremented_count = False

        if is_new_unique:
            best_uid = self.next_unique_id
            self.next_unique_id += 1
            counted = self.min_unique_confirmations <= 1
            if counted:
                self.total_count += 1
                incremented_count = True

            self.unique_objects[best_uid] = UniqueObject(
                uid=best_uid,
                label=label,
                centroid=(cx, cy),
                hist=hist,
                first_seen=stamp,
                last_seen=stamp,
                seen_count=1,
                counted=counted,
            )
        else:
            obj = self.unique_objects[best_uid]
            obj.centroid = (cx, cy)
            obj.hist = hist
            obj.last_seen = stamp
            obj.seen_count += 1
            if (not obj.counted) and obj.seen_count >= self.min_unique_confirmations:
                obj.counted = True
                self.total_count += 1
                incremented_count = True
            self.unique_objects[best_uid] = obj

        track["unique_id"] = best_uid
        track["is_new_unique"] = incremented_count
        track["counted"] = self.unique_objects[best_uid].counted
        if track_id >= 0:
            self.track_links[track_id] = TrackLink(uid=best_uid, last_seen=stamp)

    def _cleanup_unique(self, stamp):
        max_age = rospy.Duration.from_sec(self.reid_max_age_sec)
        stale = [uid for uid, obj in self.unique_objects.items() if stamp - obj.last_seen > max_age]
        for uid in stale:
            self.unique_objects.pop(uid, None)

        stale_tracks = [tid for tid, link in self.track_links.items() if stamp - link.last_seen > max_age or link.uid not in self.unique_objects]
        for tid in stale_tracks:
            self.track_links.pop(tid, None)

    def _detect_objects(self, frame):
        if self.detector_backend == "yolo" and self.yolo_runtime is not None:
            return self._detect_yolo(frame)
        if self.hog is None:
            rospy.logwarn_throttle(2.0, "Detector not ready yet; skipping frame")
            return []
        return self._detect_hog(frame)

    def _resolve_yolo_model_path(self, model_spec):
        if not model_spec:
            raise ValueError("~yolo_model is empty")

        expanded = os.path.expanduser(model_spec)
        candidates = []

        if os.path.isabs(expanded):
            candidates.append(expanded)
        else:
            candidates.append(os.path.abspath(expanded))
            try:
                import rospkg  # pylint: disable=import-error

                pkg_path = rospkg.RosPack().get_path("tello_driver")
                ws_root = os.path.abspath(os.path.join(pkg_path, "..", ".."))
                candidates.append(os.path.join(ws_root, expanded))
                candidates.append(os.path.join(ws_root, "models", expanded))
                candidates.append(os.path.join(pkg_path, "models", expanded))
            except Exception:
                pass

            candidates.append(os.path.join(os.path.expanduser("~/.config/Ultralytics"), expanded))
            candidates.append(os.path.join(os.path.expanduser("~/.cache/ultralytics"), expanded))
            candidates.append(os.path.join(os.path.expanduser("~/.cache/torch/hub/checkpoints"), expanded))

        for path in candidates:
            if os.path.isfile(path):
                return path

        if self.yolo_allow_download:
            rospy.logwarn("YOLO model '%s' not found locally. Download is enabled; trying Ultralytics fetch.", model_spec)
            return model_spec

        raise FileNotFoundError(
            "YOLO model '%s' not found locally and download disabled. Set ~yolo_model to a local .pt path or set ~yolo_allow_download:=true when internet is available." % model_spec
        )

    def _draw(self, frame, tracks):
        for track in tracks:
            x1, y1, x2, y2 = track["bbox"]
            if not track.get("counted", False):
                color = (0, 220, 220)
            else:
                color = (0, 220, 0) if not track.get("is_new_unique", False) else (0, 160, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            text = "{} u{} t{} {:.2f}".format(
                track["label"],
                track.get("unique_id", -1),
                track["id"],
                track["confidence"],
            )
            cv2.putText(frame, text, (x1, max(20, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        status = "Total unique objects: {}".format(self.total_count)
        cv2.putText(frame, status, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "cv_bridge conversion failed: %s", exc)
            return

        self.frame_index += 1
        if self.process_every_n_frames > 1 and (self.frame_index % self.process_every_n_frames) != 0:
            return

        detections = self._detect_objects(frame)
        stamp = rospy.Time.now()
        tracks = self.tracker.update(detections, stamp)
        for track in tracks:
            self._register_unique(track, frame, stamp)
        self._cleanup_unique(stamp)

        self.count_pub.publish(Int32(data=self.total_count))

        self._draw(frame, tracks)
        self.latest_annotated = frame

        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "cv_bridge annotated conversion failed: %s", exc)

        rospy.loginfo_throttle(
            2.0,
            "backend=%s device=%s detections=%d tracked=%d unique_total=%d",
            self.detector_backend,
            self.yolo_device if self.detector_backend == "yolo" else "n/a",
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
