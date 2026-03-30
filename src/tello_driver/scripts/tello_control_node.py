#!/usr/bin/env python3

import math
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
        self.safe_wall_distance_xy = float(rospy.get_param("~safe_wall_distance_xy", self.safe_wall_distance))
        self.safe_wall_distance_z = float(rospy.get_param("~safe_wall_distance_z", 0.35))
        self.clearance_hysteresis = float(rospy.get_param("~clearance_hysteresis", 0.2))
        self.avoid_speed = float(rospy.get_param("~avoid_speed", 0.2))
        self.search_speed_x = float(rospy.get_param("~search_speed_x", 0.15))
        self.search_speed_y = float(rospy.get_param("~search_speed_y", 0.1))
        self.search_speed_z = float(rospy.get_param("~search_speed_z", 0.08))
        self.search_segment_duration = float(rospy.get_param("~search_segment_duration", 6.0))
        self.search_lane_spacing_y = float(rospy.get_param("~search_lane_spacing_y", 0.8))
        self.search_layer_spacing_z = float(rospy.get_param("~search_layer_spacing_z", 0.35))
        self.search_waypoint_tolerance_xy = float(rospy.get_param("~search_waypoint_tolerance_xy", 0.20))
        self.search_waypoint_tolerance_z = float(rospy.get_param("~search_waypoint_tolerance_z", 0.15))
        self.search_kp_xy = float(rospy.get_param("~search_kp_xy", 0.8))
        self.search_kp_z = float(rospy.get_param("~search_kp_z", 0.8))
        self.counting_pause_sec = float(rospy.get_param("~counting_pause_sec", 1.0))
        self.stop_on_count_event = bool(rospy.get_param("~stop_on_count_event", False))
        self.manual_cmd_timeout = float(rospy.get_param("~manual_cmd_timeout", 0.4))
        self.takeoff_settle_sec = float(rospy.get_param("~takeoff_settle_sec", 3.0))
        self.takeoff_retry_period_sec = float(rospy.get_param("~takeoff_retry_period_sec", 1.0))
        self.landing_pre_stop_sec = float(rospy.get_param("~landing_pre_stop_sec", 0.8))
        self.target_count = int(rospy.get_param("~target_count", 10))
        self.land_on_target_count = bool(rospy.get_param("~land_on_target_count", True))

        self.room_x_min = float(rospy.get_param("~room_x_min", -3.0))
        self.room_x_max = float(rospy.get_param("~room_x_max", 3.0))
        self.room_y_min = float(rospy.get_param("~room_y_min", -3.0))
        self.room_y_max = float(rospy.get_param("~room_y_max", 3.0))
        self.room_z_min = float(rospy.get_param("~room_z_min", 0.6))
        self.room_z_max = float(rospy.get_param("~room_z_max", 1.8))

        self.pose = None
        self.latest_manual_cmd = Twist()
        self.latest_manual_stamp = rospy.Time(0)
        self.latest_count = 0
        self.last_count_event_stamp = rospy.Time(0)

        self.state = ControlState.TAKEOFF
        self.state_since = rospy.Time.now()
        self.takeoff_sent = False
        self.last_takeoff_pub_stamp = rospy.Time(0)
        self.land_sent = False
        self.last_land_pub_stamp = rospy.Time(0)
        self.search_phase = 0
        self.search_waypoints = []
        self.search_waypoint_idx = 0

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
            "wall distances [m] left=%.2f right=%.2f bottom=%.2f top=%.2f floor=%.2f ceiling=%.2f",
            distances["left"],
            distances["right"],
            distances["bottom"],
            distances["top"],
            distances["floor"],
            distances["ceiling"],
        )

    def _count_cb(self, msg):
        if msg.data > self.latest_count:
            self.last_count_event_stamp = rospy.Time.now()
            if self.state == ControlState.SEARCHING:
                self._set_state(ControlState.COUNTING)
        self.latest_count = msg.data

        if self.land_on_target_count and self.latest_count >= self.target_count:
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
        z = self.pose.position.z

        return {
            "left": x - self.room_x_min,
            "right": self.room_x_max - x,
            "bottom": y - self.room_y_min,
            "top": self.room_y_max - y,
            "floor": z - self.room_z_min,
            "ceiling": self.room_z_max - z,
        }

    def _too_close_to_wall(self):
        distances = self._distance_to_walls()
        return (
            distances["left"] < self.safe_wall_distance_xy
            or distances["right"] < self.safe_wall_distance_xy
            or distances["bottom"] < self.safe_wall_distance_xy
            or distances["top"] < self.safe_wall_distance_xy
            or distances["floor"] < self.safe_wall_distance_z
            or distances["ceiling"] < self.safe_wall_distance_z
        )

    def _is_clear_of_walls(self):
        distances = self._distance_to_walls()
        return (
            distances["left"] > (self.safe_wall_distance_xy + self.clearance_hysteresis)
            and distances["right"] > (self.safe_wall_distance_xy + self.clearance_hysteresis)
            and distances["bottom"] > (self.safe_wall_distance_xy + self.clearance_hysteresis)
            and distances["top"] > (self.safe_wall_distance_xy + self.clearance_hysteresis)
            and distances["floor"] > (self.safe_wall_distance_z + self.clearance_hysteresis)
            and distances["ceiling"] > (self.safe_wall_distance_z + self.clearance_hysteresis)
        )

    def _compute_avoidance_cmd(self):
        distances = self._distance_to_walls()
        cmd = Twist()

        if distances["left"] < self.safe_wall_distance_xy:
            cmd.linear.y += self.avoid_speed
        if distances["right"] < self.safe_wall_distance_xy:
            cmd.linear.y -= self.avoid_speed
        if distances["bottom"] < self.safe_wall_distance_xy:
            cmd.linear.x += self.avoid_speed
        if distances["top"] < self.safe_wall_distance_xy:
            cmd.linear.x -= self.avoid_speed
        if distances["floor"] < self.safe_wall_distance_z:
            cmd.linear.z += self.avoid_speed
        if distances["ceiling"] < self.safe_wall_distance_z:
            cmd.linear.z -= self.avoid_speed

        return cmd

    def _clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def _axis_points(self, axis_min, axis_max, step):
        if axis_max < axis_min:
            axis_min, axis_max = axis_max, axis_min
        if (axis_max - axis_min) < 1e-3:
            return [0.5 * (axis_min + axis_max)]
        if step <= 0.0:
            return [axis_min, axis_max]

        points = [axis_min]
        cur = axis_min
        while (cur + step) < (axis_max - 1e-6):
            cur += step
            points.append(cur)
        if abs(points[-1] - axis_max) > 1e-6:
            points.append(axis_max)
        return points

    def _build_search_waypoints(self):
        margin_xy = self.safe_wall_distance_xy + 0.10
        margin_z = self.safe_wall_distance_z + 0.10

        x_min = self.room_x_min + margin_xy
        x_max = self.room_x_max - margin_xy
        y_min = self.room_y_min + margin_xy
        y_max = self.room_y_max - margin_xy
        z_min = self.room_z_min + margin_z
        z_max = self.room_z_max - margin_z

        if x_max <= x_min:
            x_min = x_max = 0.5 * (self.room_x_min + self.room_x_max)
        if y_max <= y_min:
            y_min = y_max = 0.5 * (self.room_y_min + self.room_y_max)
        if z_max <= z_min:
            z_min = z_max = 0.5 * (self.room_z_min + self.room_z_max)

        x_points = [x_min] if abs(x_max - x_min) < 1e-3 else [x_min, x_max]
        y_points = self._axis_points(y_min, y_max, self.search_lane_spacing_y)
        z_points = self._axis_points(z_min, z_max, self.search_layer_spacing_z)

        waypoints = []
        for layer_idx, z in enumerate(z_points):
            y_order = y_points if (layer_idx % 2 == 0) else list(reversed(y_points))
            direction = 1
            for y in y_order:
                if len(x_points) == 1:
                    x_order = x_points
                elif direction > 0:
                    x_order = x_points
                else:
                    x_order = list(reversed(x_points))

                for x in x_order:
                    waypoints.append((x, y, z))
                direction *= -1

        return waypoints

    def _compute_search_cmd(self):
        cmd = Twist()
        if self.pose is None:
            return cmd

        if not self.search_waypoints:
            self.search_waypoints = self._build_search_waypoints()
            self.search_waypoint_idx = 0
            rospy.loginfo("Built %d 3D search waypoints for coverage", len(self.search_waypoints))

        if not self.search_waypoints:
            return cmd

        x = self.pose.position.x
        y = self.pose.position.y
        z = self.pose.position.z

        tx, ty, tz = self.search_waypoints[self.search_waypoint_idx]
        dx = tx - x
        dy = ty - y
        dz = tz - z

        if math.hypot(dx, dy) <= self.search_waypoint_tolerance_xy and abs(dz) <= self.search_waypoint_tolerance_z:
            self.search_waypoint_idx = (self.search_waypoint_idx + 1) % len(self.search_waypoints)
            tx, ty, tz = self.search_waypoints[self.search_waypoint_idx]
            dx = tx - x
            dy = ty - y
            dz = tz - z

        cmd.linear.x = self._clamp(self.search_kp_xy * dx, self.search_speed_x)
        cmd.linear.y = self._clamp(self.search_kp_xy * dy, self.search_speed_y)
        cmd.linear.z = self._clamp(self.search_kp_z * dz, self.search_speed_z)

        rospy.loginfo_throttle(
            1.0,
            "search wp=%d/%d target=(%.2f, %.2f, %.2f) err=(%.2f, %.2f, %.2f) cmd=(%.2f, %.2f, %.2f)",
            self.search_waypoint_idx + 1,
            len(self.search_waypoints),
            tx,
            ty,
            tz,
            dx,
            dy,
            dz,
            cmd.linear.x,
            cmd.linear.y,
            cmd.linear.z,
        )

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

        now = rospy.Time.now()
        since_last = (now - self.last_takeoff_pub_stamp).to_sec()
        should_retry = self.takeoff_sent and since_last >= self.takeoff_retry_period_sec

        if (not self.takeoff_sent) or should_retry:
            rospy.loginfo("Publishing takeoff%s", " (retry)" if self.takeoff_sent else "")
            self.takeoff_pub.publish(Empty())
            self.takeoff_sent = True
            self.last_takeoff_pub_stamp = now

        if (now - self.state_since).to_sec() >= self.takeoff_settle_sec:
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

        if not self.stop_on_count_event:
            # Keep covering the room while counting events are processed.
            self._set_state(ControlState.SEARCHING)
            return self._compute_search_cmd()

        if (rospy.Time.now() - self.last_count_event_stamp).to_sec() >= self.counting_pause_sec:
            self._set_state(ControlState.SEARCHING)

        return Twist()

    def _update_avoiding_state(self):
        cmd = self._compute_avoidance_cmd()
        if self._is_clear_of_walls():
            self._set_state(ControlState.SEARCHING)
        return cmd

    def _update_landing_state(self):
        now = rospy.Time.now()
        since_state = (now - self.state_since).to_sec()
        if since_state < self.landing_pre_stop_sec:
            # Brief zero-velocity phase before land command reduces pre-land spin.
            return Twist()

        if not self.land_sent or (now - self.last_land_pub_stamp).to_sec() >= 1.0:
            rospy.loginfo("Publishing land%s", " (retry)" if self.land_sent else "")
            self.land_pub.publish(Empty())
            self.land_sent = True
            self.last_land_pub_stamp = now
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
