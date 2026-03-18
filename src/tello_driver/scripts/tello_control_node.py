#!/usr/bin/env python3

from enum import Enum

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Empty, Int32, String


class ControlState(str, Enum):
    TAKEOFF = "TAKEOFF"
    SEARCHING = "SEARCHING"
    COUNTING = "COUNTING"
    AVOIDING_WALL = "AVOIDING_WALL"
    LANDING = "LANDING"


class TelloControlNode:
    def __init__(self):
        rospy.init_node("tello_control_node", anonymous=False)

        self.namespace = rospy.get_param("~namespace", "tello").strip("/")
        self.pose_topic = rospy.get_param("~pose_topic", "/vrpn_client_node/tello/pose")
        self.object_count_topic = rospy.get_param("~object_count_topic", "/tello/object_count")
        self.manual_cmd_topic = rospy.get_param("~manual_cmd_topic", "/tello/cmd_vel_user")

        self.enable_autonomy = bool(rospy.get_param("~enable_autonomy", True))
        self.localization_only = bool(rospy.get_param("~localization_only", False))
        self.takeoff_on_start = bool(rospy.get_param("~takeoff_on_start", False))
        self.allow_manual_input = bool(rospy.get_param("~allow_manual_input", True))

        self.safe_wall_distance = float(rospy.get_param("~safe_wall_distance", 1.0))
        self.clearance_hysteresis = float(rospy.get_param("~clearance_hysteresis", 0.2))
        self.avoid_speed = float(rospy.get_param("~avoid_speed", 0.2))
        self.search_speed_x = float(rospy.get_param("~search_speed_x", 0.15))
        self.search_speed_y = float(rospy.get_param("~search_speed_y", 0.1))
        self.search_segment_duration = float(rospy.get_param("~search_segment_duration", 6.0))
        self.counting_pause_sec = float(rospy.get_param("~counting_pause_sec", 1.0))
        self.manual_cmd_timeout = float(rospy.get_param("~manual_cmd_timeout", 0.4))
        self.takeoff_settle_sec = float(rospy.get_param("~takeoff_settle_sec", 3.0))
        self.target_count = int(rospy.get_param("~target_count", 10))

        self.room_x_min = float(rospy.get_param("~room_x_min", -3.0))
        self.room_x_max = float(rospy.get_param("~room_x_max", 3.0))
        self.room_y_min = float(rospy.get_param("~room_y_min", -3.0))
        self.room_y_max = float(rospy.get_param("~room_y_max", 3.0))

        self.pose = None
        self.latest_manual_cmd = Twist()
        self.latest_manual_stamp = rospy.Time(0)
        self.latest_count = 0
        self.last_count_event_stamp = rospy.Time(0)

        self.state = ControlState.TAKEOFF
        self.state_since = rospy.Time.now()
        self.takeoff_sent = False
        self.land_sent = False
        self.search_phase = 0

        prefix = "/{}".format(self.namespace)
        self.cmd_pub = rospy.Publisher(prefix + "/cmd_vel", Twist, queue_size=10)
        self.takeoff_pub = rospy.Publisher(prefix + "/takeoff", Empty, queue_size=2)
        self.land_pub = rospy.Publisher(prefix + "/land", Empty, queue_size=2)
        self.state_pub = rospy.Publisher(prefix + "/autonomy_state", String, queue_size=10)

        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_cb, queue_size=1)
        self.count_sub = rospy.Subscriber(self.object_count_topic, Int32, self._count_cb, queue_size=10)
        self.manual_cmd_sub = rospy.Subscriber(self.manual_cmd_topic, Twist, self._manual_cmd_cb, queue_size=10)

        rospy.loginfo("tello_control_node started. pose_topic=%s, manual_cmd_topic=%s", self.pose_topic, self.manual_cmd_topic)

    def _set_state(self, new_state):
        if new_state == self.state:
            return
        rospy.loginfo("State transition: %s -> %s", self.state.value, new_state.value)
        self.state = new_state
        self.state_since = rospy.Time.now()

    def _pose_cb(self, msg):
        self.pose = msg.pose
        distances = self._distance_to_walls()
        rospy.loginfo_throttle(
            1.0,
            "wall distances [m] left=%.2f right=%.2f bottom=%.2f top=%.2f",
            distances["left"],
            distances["right"],
            distances["bottom"],
            distances["top"],
        )

    def _count_cb(self, msg):
        if msg.data > self.latest_count:
            self.last_count_event_stamp = rospy.Time.now()
            if self.state == ControlState.SEARCHING:
                self._set_state(ControlState.COUNTING)
        self.latest_count = msg.data

        if self.latest_count >= self.target_count:
            self._set_state(ControlState.LANDING)

    def _manual_cmd_cb(self, msg):
        self.latest_manual_cmd = msg
        self.latest_manual_stamp = rospy.Time.now()

    def _distance_to_walls(self):
        if self.pose is None:
            return {
                "left": float("inf"),
                "right": float("inf"),
                "bottom": float("inf"),
                "top": float("inf"),
            }

        x = self.pose.position.x
        y = self.pose.position.y

        return {
            "left": x - self.room_x_min,
            "right": self.room_x_max - x,
            "bottom": y - self.room_y_min,
            "top": self.room_y_max - y,
        }

    def _too_close_to_wall(self):
        distances = self._distance_to_walls()
        nearest = min(distances.values())
        return nearest < self.safe_wall_distance

    def _is_clear_of_walls(self):
        distances = self._distance_to_walls()
        nearest = min(distances.values())
        return nearest > (self.safe_wall_distance + self.clearance_hysteresis)

    def _compute_avoidance_cmd(self):
        distances = self._distance_to_walls()
        cmd = Twist()

        if distances["left"] < self.safe_wall_distance:
            cmd.linear.y += self.avoid_speed
        if distances["right"] < self.safe_wall_distance:
            cmd.linear.y -= self.avoid_speed
        if distances["bottom"] < self.safe_wall_distance:
            cmd.linear.x += self.avoid_speed
        if distances["top"] < self.safe_wall_distance:
            cmd.linear.x -= self.avoid_speed

        return cmd

    def _compute_search_cmd(self):
        # Simple sweeping pattern: move forward for one segment, then strafe, then reverse.
        elapsed = (rospy.Time.now() - self.state_since).to_sec()
        segment = int(elapsed // self.search_segment_duration)
        self.search_phase = segment % 4

        cmd = Twist()
        if self.search_phase == 0:
            cmd.linear.x = self.search_speed_x
        elif self.search_phase == 1:
            cmd.linear.y = self.search_speed_y
        elif self.search_phase == 2:
            cmd.linear.x = -self.search_speed_x
        else:
            cmd.linear.y = self.search_speed_y
        return cmd

    def _manual_cmd_recent(self):
        return (rospy.Time.now() - self.latest_manual_stamp).to_sec() <= self.manual_cmd_timeout

    def _publish_cmd(self, cmd):
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=self.state.value))

    def _update_takeoff_state(self):
        if not self.takeoff_on_start:
            self._set_state(ControlState.SEARCHING)
            return Twist()

        if not self.takeoff_sent:
            rospy.loginfo("Publishing takeoff")
            self.takeoff_pub.publish(Empty())
            self.takeoff_sent = True

        if (rospy.Time.now() - self.state_since).to_sec() >= self.takeoff_settle_sec:
            self._set_state(ControlState.SEARCHING)

        return Twist()

    def _update_searching_state(self):
        if self._too_close_to_wall():
            self._set_state(ControlState.AVOIDING_WALL)
            return self._compute_avoidance_cmd()

        if self.allow_manual_input and self._manual_cmd_recent():
            return self.latest_manual_cmd

        if self.enable_autonomy:
            return self._compute_search_cmd()

        return Twist()

    def _update_counting_state(self):
        if self._too_close_to_wall():
            self._set_state(ControlState.AVOIDING_WALL)
            return self._compute_avoidance_cmd()

        if (rospy.Time.now() - self.last_count_event_stamp).to_sec() >= self.counting_pause_sec:
            self._set_state(ControlState.SEARCHING)

        return Twist()

    def _update_avoiding_state(self):
        cmd = self._compute_avoidance_cmd()
        if self._is_clear_of_walls():
            self._set_state(ControlState.SEARCHING)
        return cmd

    def _update_landing_state(self):
        if not self.land_sent:
            rospy.loginfo("Publishing land")
            self.land_pub.publish(Empty())
            self.land_sent = True
        return Twist()

    def step(self):
        if self.localization_only:
            self._publish_cmd(Twist())
            return

        if self.state == ControlState.TAKEOFF:
            cmd = self._update_takeoff_state()
        elif self.state == ControlState.SEARCHING:
            cmd = self._update_searching_state()
        elif self.state == ControlState.COUNTING:
            cmd = self._update_counting_state()
        elif self.state == ControlState.AVOIDING_WALL:
            cmd = self._update_avoiding_state()
        else:
            cmd = self._update_landing_state()

        self._publish_cmd(cmd)

    def spin(self):
        rate_hz = float(rospy.get_param("~control_rate_hz", 10.0))
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    node = TelloControlNode()
    node.spin()
