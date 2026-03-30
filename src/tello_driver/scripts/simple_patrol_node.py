#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Empty
from tello_driver.msg import TelloStatus


class SimplePatrolNode:
    def __init__(self):
        rospy.init_node("simple_patrol_node", anonymous=False)

        self.namespace = rospy.get_param("~namespace", "tello").strip("/")
        self.pose_topic = rospy.get_param("~pose_topic", "/vrpn_client_node/Telloyes/pose")

        self.room_x_min = float(rospy.get_param("~room_x_min", -1.56))
        self.room_x_max = float(rospy.get_param("~room_x_max", 1.68))
        self.room_y_min = float(rospy.get_param("~room_y_min", -1.01))
        self.room_y_max = float(rospy.get_param("~room_y_max", 2.06))
        self.room_z_min = float(rospy.get_param("~room_z_min", 0.06))
        self.room_z_max = float(rospy.get_param("~room_z_max", 1.8))

        self.boundary_margin_xy = float(rospy.get_param("~boundary_margin_xy", 0.25))
        self.boundary_margin_z = float(rospy.get_param("~boundary_margin_z", 0.20))

        self.move_speed = float(rospy.get_param("~move_speed", 0.9))
        self.pause_sec = float(rospy.get_param("~pause_sec", 1.2))
        self.loop_forever = bool(rospy.get_param("~loop_forever", True))
        self.goal_tolerance_xy = float(rospy.get_param("~goal_tolerance_xy", 0.18))
        self.goal_timeout_sec = float(rospy.get_param("~goal_timeout_sec", 18.0))
        self.xy_kp = float(rospy.get_param("~xy_kp", 0.9))
        self.near_wall_slowdown_dist = float(rospy.get_param("~near_wall_slowdown_dist", 0.45))
        self.near_wall_stop_dist = float(rospy.get_param("~near_wall_stop_dist", 0.12))
        self.near_wall_min_speed_scale = float(rospy.get_param("~near_wall_min_speed_scale", 0.25))
        self.near_wall_retreat_speed = float(rospy.get_param("~near_wall_retreat_speed", 0.18))
        self.near_wall_retreat_sec = float(rospy.get_param("~near_wall_retreat_sec", 0.8))
        self.control_mode = str(rospy.get_param("~control_mode", "world_direct")).strip().lower()
        self.require_fresh_pose = bool(rospy.get_param("~require_fresh_pose", True))
        self.pose_timeout_sec = float(rospy.get_param("~pose_timeout_sec", 0.25))
        self.pose_wait_before_takeoff_sec = float(rospy.get_param("~pose_wait_before_takeoff_sec", 8.0))
        self.abort_takeoff_if_no_pose = bool(rospy.get_param("~abort_takeoff_if_no_pose", True))
        self.stagnation_window_sec = float(rospy.get_param("~stagnation_window_sec", 2.0))
        self.min_motion_for_progress = float(rospy.get_param("~min_motion_for_progress", 0.05))

        self.hold_altitude = bool(rospy.get_param("~hold_altitude", True))
        self.target_z = float(rospy.get_param("~target_z", 1.0))
        self.z_kp = float(rospy.get_param("~z_kp", 0.7))
        self.max_z_speed = float(rospy.get_param("~max_z_speed", 0.12))

        self.takeoff_on_start = bool(rospy.get_param("~takeoff_on_start", False))
        self.takeoff_settle_sec = float(rospy.get_param("~takeoff_settle_sec", 3.0))
        self.takeoff_retry_period_sec = float(rospy.get_param("~takeoff_retry_period_sec", 1.0))
        self.takeoff_confirm_timeout_sec = float(rospy.get_param("~takeoff_confirm_timeout_sec", 10.0))
        self.land_on_shutdown = bool(rospy.get_param("~land_on_shutdown", False))

        prefix = f"/{self.namespace}"
        self.cmd_pub = rospy.Publisher(prefix + "/cmd_vel", Twist, queue_size=10)
        self.takeoff_pub = rospy.Publisher(prefix + "/takeoff", Empty, queue_size=2)
        self.land_pub = rospy.Publisher(prefix + "/land", Empty, queue_size=2)

        self.pose = None
        self.pose_last_rx = rospy.Time(0)
        self.stagnation_ref_pose = None
        self.stagnation_ref_time = rospy.Time(0)
        self.is_flying = False
        self.status_last_rx = rospy.Time(0)
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_cb, queue_size=1)
        self.status_sub = rospy.Subscriber(prefix + "/status", TelloStatus, self._status_cb, queue_size=10)

        self.sequence = []

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("simple_patrol_node started. pose_topic=%s", self.pose_topic)

    def _pose_cb(self, msg):
        self.pose = msg.pose
        self.pose_last_rx = rospy.Time.now()

    def _status_cb(self, msg):
        self.is_flying = bool(msg.is_flying)
        self.status_last_rx = rospy.Time.now()

    def _pose_is_fresh(self):
        if self.pose is None:
            return False
        if self.pose_last_rx == rospy.Time(0):
            return False
        age = (rospy.Time.now() - self.pose_last_rx).to_sec()
        return age <= self.pose_timeout_sec

    @staticmethod
    def _clamp(value, max_abs):
        return max(-max_abs, min(max_abs, value))

    @staticmethod
    def _yaw_from_quat(qx, qy, qz, qw):
        # ZYX yaw extraction from quaternion.
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def _build_extremity_targets(self):
        x_low = self.room_x_min + self.boundary_margin_xy
        x_high = self.room_x_max - self.boundary_margin_xy
        y_low = self.room_y_min + self.boundary_margin_xy
        y_high = self.room_y_max - self.boundary_margin_xy
        z_mid = max(self.room_z_min + self.boundary_margin_z, min(self.target_z, self.room_z_max - self.boundary_margin_z))

        x_mid = 0.5 * (x_low + x_high)
        y_mid = 0.5 * (y_low + y_high)

        return [
            ("front", x_high, y_mid, z_mid),
            ("back", x_low, y_mid, z_mid),
            ("right", x_mid, y_low, z_mid),
            ("left", x_mid, y_high, z_mid),
        ]

    def _apply_boundary_guards(self, cmd):
        if self.pose is None:
            return cmd

        x = self.pose.position.x
        y = self.pose.position.y
        z = self.pose.position.z

        x_low = self.room_x_min + self.boundary_margin_xy
        x_high = self.room_x_max - self.boundary_margin_xy
        y_low = self.room_y_min + self.boundary_margin_xy
        y_high = self.room_y_max - self.boundary_margin_xy
        z_low = self.room_z_min + self.boundary_margin_z
        z_high = self.room_z_max - self.boundary_margin_z

        if x <= x_low and cmd.linear.x < 0.0:
            cmd.linear.x = 0.0
        if x >= x_high and cmd.linear.x > 0.0:
            cmd.linear.x = 0.0
        if y <= y_low and cmd.linear.y < 0.0:
            cmd.linear.y = 0.0
        if y >= y_high and cmd.linear.y > 0.0:
            cmd.linear.y = 0.0

        if self.hold_altitude:
            z_err = self.target_z - z
            cmd.linear.z = self._clamp(self.z_kp * z_err, self.max_z_speed)
        else:
            cmd.linear.z = 0.0

        # Hard z guard.
        if z <= z_low and cmd.linear.z < 0.0:
            cmd.linear.z = 0.0
        if z >= z_high and cmd.linear.z > 0.0:
            cmd.linear.z = 0.0

        return cmd

    def _xy_clearance_to_guard(self):
        if self.pose is None:
            return float("inf")

        x = self.pose.position.x
        y = self.pose.position.y

        x_low = self.room_x_min + self.boundary_margin_xy
        x_high = self.room_x_max - self.boundary_margin_xy
        y_low = self.room_y_min + self.boundary_margin_xy
        y_high = self.room_y_max - self.boundary_margin_xy

        cx = min(x - x_low, x_high - x)
        cy = min(y - y_low, y_high - y)
        return min(cx, cy)

    def _retreat_from_guarded_wall(self):
        if self.pose is None:
            return

        x = self.pose.position.x
        y = self.pose.position.y
        x_low = self.room_x_min + self.boundary_margin_xy
        x_high = self.room_x_max - self.boundary_margin_xy
        y_low = self.room_y_min + self.boundary_margin_xy
        y_high = self.room_y_max - self.boundary_margin_xy

        d_left = abs(x - x_low)
        d_right = abs(x_high - x)
        d_back = abs(y - y_low)
        d_front = abs(y_high - y)

        nearest = min(d_left, d_right, d_back, d_front)
        cmd = Twist()
        if nearest == d_left:
            cmd.linear.x = self.near_wall_retreat_speed
        elif nearest == d_right:
            cmd.linear.x = -self.near_wall_retreat_speed
        elif nearest == d_back:
            cmd.linear.y = self.near_wall_retreat_speed
        else:
            cmd.linear.y = -self.near_wall_retreat_speed

        retreat_end = rospy.Time.now() + rospy.Duration(max(0.0, self.near_wall_retreat_sec))
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and rospy.Time.now() < retreat_end:
            self.cmd_pub.publish(cmd)
            rate.sleep()
        self._publish_zero()

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())

    def _perform_takeoff(self):
        if not self.takeoff_on_start:
            return True

        rospy.loginfo("Takeoff enabled: retry_period=%.2fs confirm_timeout=%.1fs", self.takeoff_retry_period_sec, self.takeoff_confirm_timeout_sec)
        t0 = rospy.Time.now()
        last_pub = rospy.Time(0)
        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self.is_flying:
                rospy.loginfo("Takeoff confirmed from /%s/status", self.namespace)
                rospy.sleep(max(0.0, self.takeoff_settle_sec))
                return True

            if (now - t0).to_sec() > self.takeoff_confirm_timeout_sec:
                rospy.logerr("Takeoff not confirmed after %.1fs; aborting patrol", self.takeoff_confirm_timeout_sec)
                return False

            if (now - last_pub).to_sec() >= self.takeoff_retry_period_sec:
                rospy.loginfo("Publishing takeoff")
                self.takeoff_pub.publish(Empty())
                last_pub = now

            rate.sleep()

        return False

    def _compute_cmd_to_target(self, tx, ty):
        cmd = Twist()
        if self.pose is None:
            return cmd

        if self.require_fresh_pose and not self._pose_is_fresh():
            return cmd

        px = self.pose.position.x
        py = self.pose.position.y
        q = self.pose.orientation
        yaw = self._yaw_from_quat(q.x, q.y, q.z, q.w)

        dx_w = tx - px
        dy_w = ty - py

        if self.control_mode == "body_from_yaw":
            # Convert world-frame XY error to body-frame command using OptiTrack yaw.
            cmd_x = math.cos(yaw) * dx_w + math.sin(yaw) * dy_w
            cmd_y = -math.sin(yaw) * dx_w + math.cos(yaw) * dy_w
        else:
            # Robust default for this setup: command directly in tracked XY frame.
            cmd_x = dx_w
            cmd_y = dy_w

        cmd.linear.x = self._clamp(self.xy_kp * cmd_x, self.move_speed)
        cmd.linear.y = self._clamp(self.xy_kp * cmd_y, self.move_speed)
        cmd = self._apply_boundary_guards(cmd)

        # Slow down and eventually stop as we get too close to the guarded room limits.
        clearance = self._xy_clearance_to_guard()
        if clearance <= self.near_wall_stop_dist:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
        elif clearance < self.near_wall_slowdown_dist:
            band = max(1e-3, self.near_wall_slowdown_dist - self.near_wall_stop_dist)
            t = (clearance - self.near_wall_stop_dist) / band
            t = max(0.0, min(1.0, t))
            scale = self.near_wall_min_speed_scale + (1.0 - self.near_wall_min_speed_scale) * t
            cmd.linear.x *= scale
            cmd.linear.y *= scale

        return cmd

    def _run_segment(self, name, tx, ty):
        start = rospy.Time.now()
        rate = rospy.Rate(20.0)

        while not rospy.is_shutdown():
            if self.pose is None:
                rospy.logwarn_throttle(1.0, "Patrol waiting: no OptiTrack pose on %s", self.pose_topic)
                self._publish_zero()
                rate.sleep()
                continue

            if self.require_fresh_pose and not self._pose_is_fresh():
                rospy.logwarn_throttle(1.0, "Patrol paused: OptiTrack pose is stale on topic %s", self.pose_topic)
                self._publish_zero()
                rate.sleep()
                continue

            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed >= self.goal_timeout_sec:
                rospy.logwarn("Timeout reaching %s extremity", name)
                break

            if self.pose is not None:
                err = math.hypot(tx - self.pose.position.x, ty - self.pose.position.y)
                if err <= self.goal_tolerance_xy:
                    break

            cmd = self._compute_cmd_to_target(tx, ty)
            if abs(cmd.linear.x) < 1e-3 and abs(cmd.linear.y) < 1e-3 and self._xy_clearance_to_guard() <= self.near_wall_stop_dist:
                rospy.logwarn("Near guarded wall while tracking %s target; stopping this segment for safety", name)
                self._retreat_from_guarded_wall()
                break
            self.cmd_pub.publish(cmd)

            if self.pose is not None:
                px = self.pose.position.x
                py = self.pose.position.y
                pz = self.pose.position.z
                err = math.hypot(tx - px, ty - py)
                rospy.loginfo_throttle(
                    0.5,
                    "patrol=%s pose=(%.2f, %.2f, %.2f) target=(%.2f, %.2f) err_xy=%.2f cmd=(%.2f, %.2f, %.2f)",
                    name,
                    px,
                    py,
                    pz,
                    tx,
                    ty,
                    err,
                    cmd.linear.x,
                    cmd.linear.y,
                    cmd.linear.z,
                )

                planar_cmd_mag = math.hypot(cmd.linear.x, cmd.linear.y)
                if planar_cmd_mag > 0.15:
                    now = rospy.Time.now()
                    if self.stagnation_ref_pose is None or self.stagnation_ref_time == rospy.Time(0):
                        self.stagnation_ref_pose = (px, py)
                        self.stagnation_ref_time = now
                    elif (now - self.stagnation_ref_time).to_sec() >= self.stagnation_window_sec:
                        moved = math.hypot(px - self.stagnation_ref_pose[0], py - self.stagnation_ref_pose[1])
                        if moved < self.min_motion_for_progress:
                            rospy.logwarn_throttle(
                                1.0,
                                "No XY progress while commanding motion: moved=%.3fm in %.1fs. Check OptiTrack tracker binding/frame alignment.",
                                moved,
                                self.stagnation_window_sec,
                            )
                        self.stagnation_ref_pose = (px, py)
                        self.stagnation_ref_time = now
                else:
                    self.stagnation_ref_pose = (px, py)
                    self.stagnation_ref_time = rospy.Time.now()
            rate.sleep()

        rospy.loginfo("Completed segment: %s", name)

        pause_start = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - pause_start).to_sec() < self.pause_sec:
            self._publish_zero()
            rate.sleep()

    def spin(self):
        wait_start = rospy.Time.now()
        while not rospy.is_shutdown() and self.pose is None and (rospy.Time.now() - wait_start).to_sec() < 10.0:
            rospy.sleep(0.1)

        if self.require_fresh_pose:
            wait_fresh_start = rospy.Time.now()
            while not rospy.is_shutdown() and not self._pose_is_fresh() and (rospy.Time.now() - wait_fresh_start).to_sec() < self.pose_wait_before_takeoff_sec:
                rospy.logwarn_throttle(1.0, "Waiting for fresh OptiTrack pose on %s", self.pose_topic)
                rospy.sleep(0.1)

            if not self._pose_is_fresh() and self.abort_takeoff_if_no_pose:
                rospy.logerr("No fresh OptiTrack pose available after %.1fs. Refusing takeoff.", self.pose_wait_before_takeoff_sec)
                return

        if not self._perform_takeoff():
            return

        self.sequence = self._build_extremity_targets()
        rospy.loginfo(
            "Extremity targets: %s",
            ", ".join(["%s(%.2f,%.2f,%.2f)" % (n, x, y, z) for n, x, y, z in self.sequence]),
        )

        while not rospy.is_shutdown():
            for name, tx, ty, _tz in self.sequence:
                self._run_segment(name, tx, ty)
                if rospy.is_shutdown():
                    return
            if not self.loop_forever:
                break

        self._publish_zero()

    def _on_shutdown(self):
        self._publish_zero()
        if self.land_on_shutdown:
            self.land_pub.publish(Empty())


if __name__ == "__main__":
    node = SimplePatrolNode()
    node.spin()
