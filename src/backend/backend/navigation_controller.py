"""
Short-segment navigation from the local terrain OccupancyGrid.

Publishes proposed ``geometry_msgs/Twist`` on ``/autonomy/navigation_twist`` for a mux
or operator tooling. Optional direct ``cmd/velocity`` when ``claim_cmd_vel`` is true (used by ``lunar run nav`` /
``nav-dig`` so Sabertooth ``drive_motors`` receives twists). Values on ``cmd/velocity`` are scaled to the
same ±100 **percent** convention as ``dig_sequence`` / joystick (see ``cmd_vel_*_full_percent`` params).
While navigation is disarmed or in mission handoff, this node skips ``cmd/velocity`` so ``dig_sequence`` can own the topic.
"""

from __future__ import annotations

import json
import math
import time
from typing import Any, Dict, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from backend.navigation_controller_pure import (
    navigation_status_dict,
    plan_navigation_step,
    scale_twist,
    heading_error_deg,
    steering_bearing_from_heading_error,
    zone_odom_xy,
)


class NavigationController(Node):
    def __init__(self) -> None:
        super().__init__("navigation_controller")
        self.declare_parameter("look_rows", 8)
        self.declare_parameter("unknown_ratio_max", 0.45)
        self.declare_parameter("v_max", 0.25)
        self.declare_parameter("w_max", 0.45)
        self.declare_parameter("grid_max_age_sec", 0.5)
        self.declare_parameter("claim_cmd_vel", False)
        self.declare_parameter("use_flag_bearing", True)
        self.declare_parameter("bearing_weight", 0.35)
        self.declare_parameter("min_flag_confidence", 0.2)
        self.declare_parameter("flag_max_age_sec", 1.5)
        self.declare_parameter("use_zone_goal", False)
        self.declare_parameter("zone_goal_id", "dig")
        self.declare_parameter("zone_goal_weight", 0.45)
        self.declare_parameter("min_goal_distance_m", 0.12)
        self.declare_parameter("max_goal_distance_m", 40.0)
        self.declare_parameter("goal_preference", "auto")
        self.declare_parameter("goal_slow_radius_m", 0.35)
        self.declare_parameter("nav_mission_state_timeout_sec", 0.9)
        # When claim_cmd_vel is true, drive_motors expects linear.x / angular.z as Sabertooth
        # percent (-100..100); planner uses normalized * v_max / w_max (~m/s style magnitudes).
        self.declare_parameter("cmd_vel_linear_full_percent", 35.0)
        self.declare_parameter("cmd_vel_angular_full_percent", 50.0)

        self._grid: Optional[np.ndarray] = None
        self._grid_mono: Optional[float] = None
        self._terrain_status: Optional[Dict[str, Any]] = None
        self._terrain_mono: Optional[float] = None
        self._autonomy_state: Optional[Dict[str, Any]] = None
        self._nav_active = False
        self._flag_candidates: Optional[Dict[str, Any]] = None
        self._flag_mono: Optional[float] = None
        self._odom: Optional[Odometry] = None
        self._nav_mission: Optional[Dict[str, Any]] = None
        self._nav_mission_mono: Optional[float] = None
        self._nav_mission_twist = Twist()

        self.create_subscription(OccupancyGrid, "/autonomy/local_terrain_grid", self._grid_cb, 10)
        self.create_subscription(String, "/autonomy/terrain_status", self._terrain_cb, 10)
        self.create_subscription(String, "/autonomy/state", self._state_cb, 10)
        self.create_subscription(String, "/perception/flag_candidates", self._flags_cb, 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(Bool, "/autonomy/navigation_active", self._active_cb, 10)
        self.create_subscription(String, "/autonomy/nav_mission/state", self._nav_mission_state_cb, 10)
        self.create_subscription(Twist, "/autonomy/nav_mission_twist", self._nav_mission_twist_cb, 10)

        self.pub_twist = self.create_publisher(Twist, "/autonomy/navigation_twist", 10)
        self.pub_status = self.create_publisher(String, "/autonomy/navigation_status", 10)
        self._pub_cmd: Optional[Any] = None
        if bool(self.get_parameter("claim_cmd_vel").value):
            self._pub_cmd = self.create_publisher(Twist, "cmd/velocity", 10)
            self.get_logger().info(
                "claim_cmd_vel is true: publishing cmd/velocity (Sabertooth % scale) when armed; "
                "releasing topic during handoff / navigation_active false (dig may take over)."
            )

        self.create_timer(0.1, self._tick)
        self.get_logger().info("navigation_controller: publishes /autonomy/navigation_twist (gated)")

    def _grid_cb(self, msg: OccupancyGrid) -> None:
        if msg.info.width <= 0 or msg.info.height <= 0:
            return
        try:
            arr = np.asarray(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width), order="C")
        except ValueError:
            return
        self._grid = arr
        self._grid_mono = time.monotonic()

    def _terrain_cb(self, msg: String) -> None:
        try:
            self._terrain_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self._terrain_status = None
        self._terrain_mono = time.monotonic()

    def _state_cb(self, msg: String) -> None:
        try:
            self._autonomy_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self._autonomy_state = None

    def _active_cb(self, msg: Bool) -> None:
        self._nav_active = bool(msg.data)

    def _flags_cb(self, msg: String) -> None:
        try:
            self._flag_candidates = json.loads(msg.data)
        except json.JSONDecodeError:
            self._flag_candidates = None
        self._flag_mono = time.monotonic()

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom = msg

    def _nav_mission_state_cb(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
            self._nav_mission = parsed if isinstance(parsed, dict) else None
            self._nav_mission_mono = time.monotonic()
        except json.JSONDecodeError:
            self._nav_mission = None
            self._nav_mission_mono = None

    def _nav_mission_twist_cb(self, msg: Twist) -> None:
        self._nav_mission_twist = msg

    def _best_bearing(self, now: float) -> tuple[Optional[float], Optional[float]]:
        if not bool(self.get_parameter("use_flag_bearing").value):
            return None, None
        mono = self._flag_mono
        if self._flag_candidates is None or mono is None:
            return None, None
        if (now - mono) > float(self.get_parameter("flag_max_age_sec").value):
            return None, None
        cands = self._flag_candidates.get("candidates") or []
        if not cands:
            return None, None
        best = cands[0]
        conf = float(best.get("confidence", 0.0))
        if conf < float(self.get_parameter("min_flag_confidence").value):
            return None, None
        b = best.get("bearing_deg")
        if b is None:
            return None, None
        try:
            return float(b), conf
        except (TypeError, ValueError):
            return None, None

    def _odom_yaw(self, msg: Odometry) -> float:
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return float(math.atan2(siny_cosp, cosy_cosp))

    def _try_zone_goal(self) -> tuple[Optional[float], float, Optional[float], Optional[float]]:
        """
        Returns (steering_bearing_deg, weight, heading_error_deg, distance_m) for zone goal, or (None,0,...).
        """
        if not bool(self.get_parameter("use_zone_goal").value):
            return None, 0.0, None, None
        odom = self._odom
        state = self._autonomy_state
        if odom is None or state is None:
            return None, 0.0, None, None
        zones = state.get("zones")
        if not isinstance(zones, dict):
            return None, 0.0, None, None
        zid = str(self.get_parameter("zone_goal_id").value)
        xy = zone_odom_xy(zones, zid)
        if xy is None:
            return None, 0.0, None, None
        rx = float(odom.pose.pose.position.x)
        ry = float(odom.pose.pose.position.y)
        yaw = self._odom_yaw(odom)
        he = heading_error_deg(rx, ry, yaw, xy[0], xy[1])
        dist = float(math.hypot(xy[0] - rx, xy[1] - ry))
        d_min = float(self.get_parameter("min_goal_distance_m").value)
        d_max = float(self.get_parameter("max_goal_distance_m").value)
        if dist <= d_min or dist >= d_max:
            return None, 0.0, he, dist
        steer = steering_bearing_from_heading_error(he)
        w = float(self.get_parameter("zone_goal_weight").value)
        return steer, w, he, dist

    def _select_goal_bearing(
        self, now: float
    ) -> tuple[
        Optional[float],
        float,
        Optional[str],
        Optional[float],
        Optional[float],
        Optional[float],
    ]:
        """Returns (bearing_deg, weight, goal_source, heading_err_deg, goal_distance_m, flag_confidence)."""
        pref = str(self.get_parameter("goal_preference").value).strip().lower()
        z_bear, z_w, z_he, z_dist = self._try_zone_goal()
        f_bear, f_conf = self._best_bearing(now)
        f_w = float(self.get_parameter("bearing_weight").value) if f_bear is not None else 0.0

        if pref == "zone":
            if z_bear is not None:
                return z_bear, z_w, "zone", z_he, z_dist, None
            return None, 0.0, None, None, None, None
        if pref == "flag":
            if f_bear is not None:
                return f_bear, f_w, "flag", None, None, f_conf
            return None, 0.0, None, None, None, None
        if z_bear is not None:
            return z_bear, z_w, "zone", z_he, z_dist, None
        if f_bear is not None:
            return f_bear, f_w, "flag", None, None, f_conf
        return None, 0.0, None, None, None, None

    def _tick(self) -> None:
        now = time.monotonic()
        twist = Twist()
        plan_reason = "idle"

        if self._autonomy_state and bool(self._autonomy_state.get("estop")):
            self._publish_outputs(twist, active=False, gated_reason="estop", plan_reason=plan_reason)
            return
        if self._autonomy_state and str(self._autonomy_state.get("mode", "")) == "Paused":
            self._publish_outputs(twist, active=False, gated_reason="paused", plan_reason=plan_reason)
            return

        terrain_ok = bool(self._terrain_status and self._terrain_status.get("ok"))
        terrain_fresh = self._terrain_mono is not None and (now - self._terrain_mono) <= 1.0
        gated_reason = "ok"
        if not terrain_ok:
            gated_reason = "terrain_not_ok"
        elif not terrain_fresh:
            gated_reason = "terrain_stale"

        grid = self._grid
        grid_mono = self._grid_mono
        max_age = float(self.get_parameter("grid_max_age_sec").value)
        if grid is None or grid_mono is None:
            gated_reason = "no_grid"
        elif (now - grid_mono) > max_age:
            gated_reason = "grid_stale"

        if gated_reason != "ok":
            self._publish_outputs(twist, active=False, gated_reason=gated_reason, plan_reason=plan_reason)
            return

        assert grid is not None

        mission_timeout = float(self.get_parameter("nav_mission_state_timeout_sec").value)
        m_age = (now - self._nav_mission_mono) if self._nav_mission_mono is not None else 999.0
        mission = self._nav_mission or {}
        cm = str(mission.get("controller_mode", "idle"))
        m_phase = str(mission.get("phase", ""))

        if m_age < mission_timeout and cm == "none":
            self._publish_outputs(
                twist,
                active=False,
                gated_reason="nav_mission_handoff",
                plan_reason="nav_mission",
                mission_phase=m_phase,
                mission_controller_mode=cm,
            )
            return
        if m_age < mission_timeout and cm == "mission_twist":
            mt = self._nav_mission_twist
            v_max = float(self.get_parameter("v_max").value)
            w_max = float(self.get_parameter("w_max").value)
            vx = float(mt.linear.x)
            wz = float(mt.angular.z)
            twist.linear.x = max(-v_max, min(v_max, vx))
            twist.angular.z = max(-w_max, min(w_max, wz))
            self._publish_outputs(
                twist,
                active=True,
                gated_reason="ok",
                plan_reason="nav_mission_twist",
                mission_phase=m_phase,
                mission_controller_mode=cm,
            )
            return

        if not self._nav_active:
            self._publish_outputs(
                twist,
                active=False,
                gated_reason="navigation_active_false",
                plan_reason=plan_reason,
                mission_phase=m_phase if m_age < mission_timeout else None,
                mission_controller_mode=cm if m_age < mission_timeout else None,
            )
            return

        look = int(self.get_parameter("look_rows").value)
        ur = float(self.get_parameter("unknown_ratio_max").value)
        bearing_deg, bw, goal_src, he_odom, goal_dist_m, flag_conf = self._select_goal_bearing(now)
        ln, an, plan_reason = plan_navigation_step(
            grid,
            look_rows=look,
            unknown_ratio_max=ur,
            bearing_deg=bearing_deg,
            bearing_weight=bw,
        )
        v_max = float(self.get_parameter("v_max").value)
        w_max = float(self.get_parameter("w_max").value)
        twist.linear.x, twist.angular.z = scale_twist(ln, an, v_max=v_max, w_max=w_max)
        r_slow = float(self.get_parameter("goal_slow_radius_m").value)
        if (
            goal_dist_m is not None
            and math.isfinite(goal_dist_m)
            and r_slow > 0.0
            and goal_dist_m < r_slow
        ):
            twist.linear.x *= max(0.15, float(goal_dist_m) / r_slow)
        self._publish_outputs(
            twist,
            active=True,
            gated_reason="ok",
            plan_reason=plan_reason,
            bearing_deg=bearing_deg,
            bearing_confidence=flag_conf if goal_src == "flag" else None,
            goal_source=goal_src,
            goal_distance_m=goal_dist_m,
            heading_error_deg_odom=he_odom if goal_src == "zone" else None,
            mission_phase=m_phase if m_age < mission_timeout and cm == "corridor_follow" else None,
            mission_controller_mode=cm if m_age < mission_timeout and cm == "corridor_follow" else None,
        )

    def _publish_outputs(
        self,
        twist: Twist,
        *,
        active: bool,
        gated_reason: str,
        plan_reason: str,
        bearing_deg: Optional[float] = None,
        bearing_confidence: Optional[float] = None,
        goal_source: Optional[str] = None,
        goal_distance_m: Optional[float] = None,
        heading_error_deg_odom: Optional[float] = None,
        mission_phase: Optional[str] = None,
        mission_controller_mode: Optional[str] = None,
    ) -> None:
        status = navigation_status_dict(
            active=active,
            gated_reason=gated_reason,
            plan_reason=plan_reason,
            linear_x=float(twist.linear.x),
            angular_z=float(twist.angular.z),
            bearing_deg=bearing_deg,
            bearing_confidence=bearing_confidence,
            goal_source=goal_source,
            goal_distance_m=goal_distance_m,
            heading_error_deg_odom=heading_error_deg_odom,
            mission_phase=mission_phase,
            mission_controller_mode=mission_controller_mode,
        )
        s = String()
        s.data = json.dumps(status)
        self.pub_status.publish(s)
        self.pub_twist.publish(twist)
        if self._pub_cmd is not None:
            # Avoid flooding cmd/velocity with zeros while disarmed or at dig handoff — dig_sequence
            # publishes the same topic during autonomous dig.
            if gated_reason in ("navigation_active_false", "nav_mission_handoff"):
                return
            self._pub_cmd.publish(self._twist_to_drive_motors(twist))

    def _twist_to_drive_motors(self, twist: Twist) -> Twist:
        """Map planner twist (linear ~v_max, angular ~w_max) to drive_motors percent input."""
        v_max = float(self.get_parameter("v_max").value)
        w_max = float(self.get_parameter("w_max").value)
        p_lin = float(self.get_parameter("cmd_vel_linear_full_percent").value)
        p_ang = float(self.get_parameter("cmd_vel_angular_full_percent").value)
        g_lin = (p_lin / v_max) if abs(v_max) > 1e-9 else 0.0
        g_ang = (p_ang / w_max) if abs(w_max) > 1e-9 else 0.0
        out = Twist()
        out.linear.x = max(-100.0, min(100.0, float(twist.linear.x) * g_lin))
        out.angular.z = max(-100.0, min(100.0, float(twist.angular.z) * g_ang))
        return out


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = NavigationController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
