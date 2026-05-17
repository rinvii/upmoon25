"""
Autonomous dig sequence for lunar `run dig`.

Runs once after IR/bucket calibration, then executes exactly three simple
drive-forward → drive-back cycles by default. The bucket position bumps between
cycles. Conveyor output is explicitly held off for this profile.

When ``use_local_terrain_grid`` is true (default), drive phases consult the same
``/autonomy/local_terrain_grid`` OccupancyGrid as short-segment nav (forward / rear
corridor slices + ``plan_corridor_step``). Dig does not turn the robot; it only
holds ``cmd/velocity`` when the map says the commanded direction is unsafe.

If IR reaches the target first during setup, the linear bucket pose still stops stepping, but the
dig belt (``cmd/bucket_vel`` / bucket chain) **stays commanded** through the three forward/back
cycles until the sequence finishes or aborts. If setup exits on the bucket position safety cap
without IR, the same belt behavior applies.

**Setup vs. drive:** In ``SETUP_IR`` the controller does not command wheel motion; it only steps
``cmd/bucket_pos`` until IR is in range or ``bucket_safety_stop`` is hit, then switches to
``DRIVE_FORWARD``. If the bucket count keeps changing but wheels stay still, check that phase in
``/autonomy/dig_sequence/state``.

**Terrain gating:** When the local grid stops updating (age ``> grid_max_age_sec``), drive legs
still use the **last** grid for corridor checks so encoder-mode digs do not freeze; a throttled
warn is logged. Increase ``grid_max_age_sec`` if your grid publishes slowly.

By default ``ir_setup_mode`` is ``le``: IR is expected to **decrease** toward ``ir_target``
(e.g. from ~70 while high to 17 at depth). The setup phase stops when IR is **less than or equal
to** ``ir_target``, after it has been **above** ``ir_target`` or the bucket has stepped past
``bucket_start_pos``. Use ``ir_setup_mode:=eq`` for the legacy exact IR match.

During setup, optional ``ir_bucket_gate_min_ir_drop`` (default ``2``, ``0`` disables) **gates**
each successive ``cmd/bucket_pos`` step: after a step, publishing the next increment waits until IR
drops by at least that amount vs. the reading **before** the step, or ``ir_bucket_gate_timeout_sec``
elapses.
Optional ``end_cycle_conveyor_seconds`` adds a conveyor-on window at the end of each cycle.
Default is ``0`` (conveyor held off). ``dig-backup`` enables this to run the conveyor for ~5s
after each cycle before repeating drive-forward.
Set ``timed_drive_ms`` > 0 to run forward and backward drive legs for the same duration (ms)
without wheel encoders. Otherwise forward stops at ``calibrated_rotary`` ticks and backward
when the encoder reads ~zero.

Prerequisite: frontend stack publishing /sensor/ir and
subscribed to cmd/velocity, cmd/bucket_pos, cmd/bucket_vel, cmd/conveyor.
``cmd/conveyor`` is only published as ``0`` by this node.
Encoder topics are only needed when ``timed_drive_ms`` is 0.
For terrain gating, run ``local_terrain_grid`` (e.g. ``lunar run nav``) so the grid topic exists.
"""

from __future__ import annotations

import json
import time
from enum import Enum

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, Int16, Int32, String

from backend.dig_sequence_params import (
    encoder_forward_target_reached,
    encoder_returned_home,
    ir_bucket_step_gate_released,
    ir_setup_stop_eq,
    ir_setup_stop_le,
    merge_timed_drive_ms,
    ros_param_non_negative_int,
    timed_leg_complete,
)
from backend.navigation_controller_pure import plan_corridor_step

DEFAULT_DIG_CYCLES = 8
DIG_BUCKET_POS_MAX = 50


class DigState(Enum):
    """When ``wait_for_nav_dig_arm`` is true, sequence begins in ``WAIT_NAV_ARM`` until ``/autonomy/dig_arm`` is true."""

    WAIT_NAV_ARM = 0
    SETUP_IR = 1
    DRIVE_FORWARD = 2
    DRIVE_BACK = 3
    CYCLE_END_CONVEYOR = 4
    DONE = 5


class DigSequenceController(Node):
    def __init__(self):
        super().__init__("dig_sequence")

        self.declare_parameter("calibrated_rotary", 0)
        # When > 0, forward and backward drive legs each run for this duration (encoder unused).
        self.declare_parameter("timed_drive_ms", 0)
        self.declare_parameter("forward_drive_ms", 0)  # deprecated; same meaning as timed_drive_ms if timed_drive_ms unset
        self.declare_parameter("encoder_side", "left")
        self.declare_parameter("encoder_tolerance", 2)
        self.declare_parameter("forward_encoder_increases", True)
        self.declare_parameter("ir_setup_mode", "le")
        self.declare_parameter("ir_target", 17)
        self.declare_parameter("bucket_start_pos", 20)
        self.declare_parameter("bucket_safety_stop", DIG_BUCKET_POS_MAX)
        self.declare_parameter("bucket_chain_speed", 40)
        self.declare_parameter("max_cycles_le", DEFAULT_DIG_CYCLES)
        self.declare_parameter("forward_linear", 35.0)
        self.declare_parameter("backward_linear", -35.0)
        self.declare_parameter("control_dt", 0.05)
        self.declare_parameter("ir_bucket_step_every_sec", 0.2)
        # After each bucket_pos increment in SETUP_IR, wait for IR to drop by this much (vs reading
        # before the step) before allowing the next increment. 0 disables the gate.
        self.declare_parameter("ir_bucket_gate_min_ir_drop", 2)
        self.declare_parameter("ir_bucket_gate_timeout_sec", 25.0)
        self.declare_parameter("conveyor_seconds", 0.0)
        self.declare_parameter("end_cycle_conveyor_seconds", 0.0)
        self.declare_parameter("phase_timeout_sec", 180.0)
        self.declare_parameter("wait_for_nav_dig_arm", False)
        # Same local traversability map as short-segment nav (`local_terrain_grid` → OccupancyGrid).
        self.declare_parameter("use_local_terrain_grid", True)
        self.declare_parameter("grid_topic", "/autonomy/local_terrain_grid")
        self.declare_parameter("grid_max_age_sec", 0.6)
        self.declare_parameter("terrain_look_rows", 8)
        self.declare_parameter("terrain_unknown_ratio_max", 0.45)

        self.calibrated_rotary = ros_param_non_negative_int(self.get_parameter("calibrated_rotary").value)
        td_raw = ros_param_non_negative_int(self.get_parameter("timed_drive_ms").value)
        legacy_fwd = ros_param_non_negative_int(self.get_parameter("forward_drive_ms").value)
        self.timed_drive_ms = merge_timed_drive_ms(td_raw, legacy_fwd)
        if legacy_fwd > 0 and td_raw > 0 and legacy_fwd != td_raw:
            self.get_logger().warn("Both timed_drive_ms and forward_drive_ms set; using timed_drive_ms.")
        self._timed_drive_only = self.timed_drive_ms > 0

        enc_side = str(self.get_parameter("encoder_side").value).strip().lower()
        if enc_side not in {"left", "right"}:
            self.get_logger().warn("encoder_side must be 'left' or 'right'; defaulting to 'left'.")
            enc_side = "left"
        self.encoder_topic = "" if self._timed_drive_only else f"/sensor/encoder/{enc_side}"

        self.encoder_tolerance = max(0, int(self.get_parameter("encoder_tolerance").value))
        self.forward_encoder_increases = bool(self.get_parameter("forward_encoder_increases").value)
        self.ir_target = int(self.get_parameter("ir_target").value)
        _irm = str(self.get_parameter("ir_setup_mode").value).strip().lower()
        if _irm not in {"le", "eq"}:
            self.get_logger().warn("ir_setup_mode must be 'le' or 'eq'; defaulting to 'le'.")
            _irm = "le"
        self.ir_setup_mode = _irm
        self.bucket_start_pos = int(self.get_parameter("bucket_start_pos").value)
        raw_bucket_safety_stop = int(self.get_parameter("bucket_safety_stop").value)
        self.bucket_safety_stop = min(DIG_BUCKET_POS_MAX, raw_bucket_safety_stop)
        if raw_bucket_safety_stop > DIG_BUCKET_POS_MAX:
            self.get_logger().warn(
                f"Ignoring bucket_safety_stop:={raw_bucket_safety_stop}; dig bucket position is capped at {DIG_BUCKET_POS_MAX}%."
            )
        self.bucket_chain_speed = int(self.get_parameter("bucket_chain_speed").value)
        raw_max_cycles = int(self.get_parameter("max_cycles_le").value)
        self.max_cycles_le = max(1, min(100, raw_max_cycles))
        if raw_max_cycles != self.max_cycles_le:
            self.get_logger().warn(
                f"Clamped max_cycles_le:={raw_max_cycles} to {self.max_cycles_le}; valid range is 1..100."
            )
        self.forward_linear = float(self.get_parameter("forward_linear").value)
        self.backward_linear = float(self.get_parameter("backward_linear").value)
        self.control_dt = float(self.get_parameter("control_dt").value)
        self.ir_bucket_step_every_sec = float(self.get_parameter("ir_bucket_step_every_sec").value)
        self.ir_bucket_gate_min_drop = max(0, int(self.get_parameter("ir_bucket_gate_min_ir_drop").value))
        self.ir_bucket_gate_timeout_sec = float(self.get_parameter("ir_bucket_gate_timeout_sec").value)
        self.conveyor_seconds = 0.0
        self.end_cycle_conveyor_seconds = max(0.0, float(self.get_parameter("end_cycle_conveyor_seconds").value))
        self.phase_timeout_sec = float(self.get_parameter("phase_timeout_sec").value)
        self._use_local_terrain_grid = bool(self.get_parameter("use_local_terrain_grid").value)
        self._grid_topic = str(self.get_parameter("grid_topic").value).strip() or "/autonomy/local_terrain_grid"
        self._grid_max_age_sec = float(self.get_parameter("grid_max_age_sec").value)
        self._terrain_look_rows = max(1, int(self.get_parameter("terrain_look_rows").value))
        self._terrain_unknown_max = float(self.get_parameter("terrain_unknown_ratio_max").value)
        self._grid_np: np.ndarray | None = None
        self._grid_mono: float | None = None
        self._had_terrain_grid = False
        self._last_terrain_warn = 0.0

        if self.calibrated_rotary <= 0 and self.timed_drive_ms <= 0:
            raise RuntimeError(
                "dig_sequence needs either calibrated_rotary > 0 (encoder mode) or timed_drive_ms > 0 (timed forward+back). "
                f"Got calibrated_rotary={self.calibrated_rotary}, timed_drive_ms={td_raw}, "
                f"forward_drive_ms(legacy)={legacy_fwd} (merged timed ms={self.timed_drive_ms}). "
                "If you passed -p timed_drive_ms:=N via `lunar run dig --dig-timing-ms`, rebuild and source: "
                "`colcon build --packages-select backend` then `source install/setup.bash` (stale install often drops overrides)."
            )

        sens_qos = QoSProfile(depth=3, reliability=2, history=1, durability=2)

        self.pub_vel = self.create_publisher(Twist, "cmd/velocity", 10)
        self.pub_bucket_pos = self.create_publisher(Int16, "cmd/bucket_pos", 10)
        self.pub_bucket_vel = self.create_publisher(Int16, "cmd/bucket_vel", 10)
        self.pub_conveyor = self.create_publisher(Int16, "cmd/conveyor", 10)
        self.pub_dig_state = self.create_publisher(String, "/autonomy/dig_sequence/state", 10)
        self._wait_for_nav_arm = bool(self.get_parameter("wait_for_nav_dig_arm").value)

        self.ir_value = -1
        self.encoder_value = 0

        self.create_subscription(Int16, "/sensor/ir", self._on_ir, sens_qos)
        if not self._timed_drive_only:
            self.create_subscription(Int32, self.encoder_topic, self._on_encoder, sens_qos)
        if self._use_local_terrain_grid:
            self.create_subscription(OccupancyGrid, self._grid_topic, self._grid_cb, 10)

        self._nav_dig_arm = False
        if self._wait_for_nav_arm:
            self.create_subscription(Bool, "/autonomy/dig_arm", self._dig_arm_cb, 10)

        self.state = DigState.WAIT_NAV_ARM if self._wait_for_nav_arm else DigState.SETUP_IR
        self.phase_clock = self.get_clock().now()
        self.ir_last_step_time = self.get_clock().now()
        self.conveyor_until = None
        self._conveyor_end_applied = True

        self.bucket_pos_commanded = self.bucket_start_pos
        self.cycle_counter = 0
        self.setup_complete = False
        self._ir_setup_was_above_target = False
        self._ir_bucket_gate_waiting = False
        self._ir_anchor_before_last_bucket_step = -1
        self._ir_bucket_gate_t0 = 0.0
        # True while dig_sequence commands the belt through the mission (telemetry / dashboard hint).
        self.keep_bucket_chain_until_done = False
        # Legacy telemetry key remains false; this simplified profile has no dump/conveyor leg.
        self._post_dump_bump_pending = False
        self._last_status_log = 0.0

        self.timer = self.create_timer(self.control_dt, self._tick)
        ginfo = f"local_terrain_grid={self._grid_topic}" if self._use_local_terrain_grid else "local_terrain_grid=off"
        if self._timed_drive_only:
            fwd_desc = f"timed drive legs {self.timed_drive_ms} ms each (no encoder)"
        else:
            fwd_desc = f"encoder forward target {self.calibrated_rotary} on {self.encoder_topic}"
        gate_msg = "ir_bucket_step gate off (ir_bucket_gate_min_ir_drop:=0)"
        if self.ir_bucket_gate_min_drop > 0:
            gate_msg = (
                f"ir_bucket_step gate: min_drop={self.ir_bucket_gate_min_drop} "
                f"timeout_sec={self.ir_bucket_gate_timeout_sec}"
            )
        self.get_logger().info(
            f"dig_sequence start (state={self.state.name}): IRMode={self.ir_setup_mode}, IR→{self.ir_target}, "
            f"bucket {self.bucket_start_pos}..{self.bucket_safety_stop}, "
            f"{fwd_desc}, cycles={self.max_cycles_le}, conveyor=disabled, {ginfo}, {gate_msg}"
        )

    def _grid_cb(self, msg: OccupancyGrid) -> None:
        if msg.info.width <= 0 or msg.info.height <= 0:
            return
        try:
            arr = np.asarray(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width), order="C")
        except ValueError:
            return
        self._grid_np = arr
        self._grid_mono = time.monotonic()
        self._had_terrain_grid = True

    def _terrain_fresh(self) -> bool:
        if self._grid_mono is None:
            return False
        return (time.monotonic() - self._grid_mono) <= self._grid_max_age_sec

    def _terrain_slice_forward(self) -> np.ndarray | None:
        if self._grid_np is None:
            return None
        g = self._grid_np
        h = int(g.shape[0])
        lr = min(self._terrain_look_rows, h)
        return g[:lr, :]

    def _terrain_slice_reverse(self) -> np.ndarray | None:
        if self._grid_np is None:
            return None
        g = self._grid_np
        h = int(g.shape[0])
        lr = min(self._terrain_look_rows, h)
        return g[-lr:, :] if h >= lr else g

    def _terrain_plan_ok(self, sub: np.ndarray | None) -> tuple[bool, str]:
        """Return (allowed, reason) using the same corridor scoring as ``navigation_controller``."""
        if sub is None or sub.size == 0:
            return True, "no_slice"
        if sub.shape[0] < 2 or sub.shape[1] < 3:
            return True, "slice_too_small"
        lr = min(self._terrain_look_rows, int(sub.shape[0]))
        ln, an, reason = plan_corridor_step(
            sub, look_rows=lr, unknown_ratio_max=self._terrain_unknown_max
        )
        ok = (ln > 1e-6) or (abs(an) > 1e-6)
        return ok, reason

    def _terrain_gate_forward(self) -> tuple[bool, str]:
        if not self._use_local_terrain_grid:
            return True, "disabled"
        if not self._had_terrain_grid:
            return True, "no_grid_yet"
        if not self._terrain_fresh():
            self._maybe_warn_terrain("Dig: forward terrain grid stale — using last grid for gating (drive not blocked)")
        return self._terrain_plan_ok(self._terrain_slice_forward())

    def _terrain_gate_reverse(self) -> tuple[bool, str]:
        if not self._use_local_terrain_grid:
            return True, "disabled"
        if not self._had_terrain_grid:
            return True, "no_grid_yet"
        if not self._terrain_fresh():
            self._maybe_warn_terrain("Dig: reverse terrain grid stale — using last grid for gating (drive not blocked)")
        return self._terrain_plan_ok(self._terrain_slice_reverse())

    def _maybe_warn_terrain(self, detail: str) -> None:
        now = time.monotonic()
        if now - self._last_terrain_warn < 2.0:
            return
        self._last_terrain_warn = now
        self.get_logger().warn(detail)

    def _dig_arm_cb(self, msg: Bool) -> None:
        self._nav_dig_arm = bool(msg.data)

    def _on_ir(self, msg: Int16) -> None:
        self.ir_value = int(msg.data)

    def _on_encoder(self, msg: Int32) -> None:
        self.encoder_value = int(msg.data)

    def _stop_motion(self) -> None:
        t = Twist()
        self.pub_vel.publish(t)

    def _publish_vel(self, linear_x: float) -> None:
        t = Twist()
        t.linear.x = float(linear_x)
        self.pub_vel.publish(t)

    def _status_drive_cmd(self) -> float:
        if self.state == DigState.DRIVE_FORWARD:
            return float(self.forward_linear)
        if self.state == DigState.DRIVE_BACK:
            return float(self.backward_linear)
        return 0.0

    def _log_status_throttled(self) -> None:
        now = time.monotonic()
        if now - self._last_status_log < 1.0:
            return
        self._last_status_log = now
        fwd_ok, fwd_r = self._terrain_gate_forward()
        rev_ok, rev_r = self._terrain_gate_reverse()
        gate = ""
        if self._ir_bucket_gate_waiting:
            gate = (
                f" ir_gate=waiting(anchor={self._ir_anchor_before_last_bucket_step},"
                f"elapsed={time.monotonic() - self._ir_bucket_gate_t0:.1f}s)"
            )
        self.get_logger().info(
            "dig_status "
            f"phase={self.state.name} "
            f"drive_cmd={self._status_drive_cmd():.2f} "
            f"ir={self.ir_value}/{self.ir_target} "
            f"encoder={self.encoder_value}/{self.calibrated_rotary} "
            f"bucket={self.bucket_pos_commanded}/{self.bucket_safety_stop} "
            f"cycles={self.cycle_counter}/{self.max_cycles_le} "
            f"timed_ms={self.timed_drive_ms} "
            f"terrain_fwd={fwd_ok}:{fwd_r} terrain_rev={rev_ok}:{rev_r}"
            f"{gate}"
        )

    def _phase_elapsed(self) -> float:
        return (self.get_clock().now() - self.phase_clock).nanoseconds / 1e9

    def _reset_phase_clock(self) -> None:
        self.phase_clock = self.get_clock().now()

    def _publish_dig_state(self) -> None:
        conv_rem: float | None = None
        if self.conveyor_until is not None and self.state == DigState.CYCLE_END_CONVEYOR:
            conv_rem = max(0.0, (self.conveyor_until - self.get_clock().now()).nanoseconds / 1e9)
        fwd_ok, fwd_r = self._terrain_gate_forward()
        rev_ok, rev_r = self._terrain_gate_reverse()
        payload = {
            "stamp": time.time(),
            "phase": self.state.name,
            "wait_for_nav_dig_arm": self._wait_for_nav_arm,
            "dig_arm": bool(self._nav_dig_arm),
            "ir_value": int(self.ir_value),
            "ir_target": int(self.ir_target),
            "ir_setup_mode": self.ir_setup_mode,
            "encoder_value": int(self.encoder_value),
            "encoder_target": int(self.calibrated_rotary),
            "encoder_topic": self.encoder_topic,
            "timed_drive_ms": int(self.timed_drive_ms),
            "drive_uses_encoder": not self._timed_drive_only,
            "cycle_counter": int(self.cycle_counter),
            "max_cycles_le": int(self.max_cycles_le),
            "bucket_pos_commanded": int(self.bucket_pos_commanded),
            "keep_bucket_chain_until_done": bool(self.keep_bucket_chain_until_done),
            "bucket_chain_speed": int(self.bucket_chain_speed),
            "phase_elapsed_sec": float(self._phase_elapsed()),
            "conveyor_remaining_sec": conv_rem,
            "use_local_terrain_grid": self._use_local_terrain_grid,
            "terrain_had_grid": self._had_terrain_grid,
            "terrain_fresh": self._terrain_fresh() if self._had_terrain_grid else False,
            "terrain_forward_ok": fwd_ok,
            "terrain_reverse_ok": rev_ok,
            "terrain_gate_forward": fwd_r,
            "terrain_gate_reverse": rev_r,
            "ir_bucket_gate_min_ir_drop": int(self.ir_bucket_gate_min_drop),
            "ir_bucket_gate_timeout_sec": float(self.ir_bucket_gate_timeout_sec),
            "ir_bucket_gate_waiting": bool(self._ir_bucket_gate_waiting),
            "ir_anchor_before_last_bucket_step": int(self._ir_anchor_before_last_bucket_step),
            "ir_bucket_gate_elapsed_sec": (
                (time.monotonic() - self._ir_bucket_gate_t0) if self._ir_bucket_gate_waiting else None
            ),
            "post_dump_bucket_bump_pending": bool(self._post_dump_bump_pending),
        }
        m = String()
        m.data = json.dumps(payload)
        self.pub_dig_state.publish(m)

    def _sync_conveyor_output(self) -> None:
        """Conveyor runs only during optional end-of-cycle conveyor window."""
        if self.state != DigState.CYCLE_END_CONVEYOR:
            self.pub_conveyor.publish(Int16(data=0))
            return
        if self._conveyor_end_applied:
            self.pub_conveyor.publish(Int16(data=0))
            return
        if self.conveyor_until is None:
            return
        now = self.get_clock().now()
        if now < self.conveyor_until:
            self.pub_conveyor.publish(Int16(data=1))
        else:
            self.pub_conveyor.publish(Int16(data=0))

    def _sync_bucket_chain_output(self) -> None:
        """Keep the bucket chain spinning during active dig phases."""
        if self.state in (
            DigState.SETUP_IR,
            DigState.DRIVE_FORWARD,
            DigState.DRIVE_BACK,
        ):
            self.pub_bucket_vel.publish(Int16(data=int(self.bucket_chain_speed)))
        else:
            self.pub_bucket_vel.publish(Int16(data=0))

    def _apply_post_cycle_bump_and_maybe_repeat(self) -> None:
        self.bucket_pos_commanded = min(DIG_BUCKET_POS_MAX, self.bucket_pos_commanded + 1)
        self.pub_bucket_pos.publish(Int16(data=int(self.bucket_pos_commanded)))
        self.cycle_counter += 1
        self.get_logger().info(
            f"Post-cycle bump: bucket_pos={self.bucket_pos_commanded}, cycle_counter={self.cycle_counter}"
        )
        self._post_dump_bump_pending = False
        self._ir_bucket_gate_waiting = False

        if self.cycle_counter < self.max_cycles_le:
            if self.end_cycle_conveyor_seconds > 0.0:
                self.get_logger().info(
                    f"Starting end-of-cycle conveyor pass for {self.end_cycle_conveyor_seconds:.2f}s."
                )
                self.state = DigState.CYCLE_END_CONVEYOR
                self._conveyor_end_applied = False
                self.conveyor_until = self.get_clock().now() + rclpy.duration.Duration(
                    seconds=self.end_cycle_conveyor_seconds
                )
                self._reset_phase_clock()
                return
            self.get_logger().info("Repeating drive-forward phase.")
            self.state = DigState.DRIVE_FORWARD
            self.conveyor_until = None
            self._conveyor_end_applied = True
            self._reset_phase_clock()
        else:
            self.get_logger().info(f"{self.max_cycles_le}-cycle dig profile complete; terminating loop.")
            self.keep_bucket_chain_until_done = False
            self._post_dump_bump_pending = False
            self._stop_motion()
            self.pub_bucket_vel.publish(Int16(data=0))
            self.pub_conveyor.publish(Int16(data=0))
            self.state = DigState.DONE
            self.timer.cancel()
            if rclpy.ok():
                rclpy.shutdown()

    def _abort(self, reason: str) -> None:
        self.get_logger().error(reason)
        self.keep_bucket_chain_until_done = False
        self._post_dump_bump_pending = False
        self._stop_motion()
        self.pub_bucket_vel.publish(Int16(data=0))
        self.pub_conveyor.publish(Int16(data=0))
        self.state = DigState.DONE
        self.timer.cancel()
        if rclpy.ok():
            rclpy.shutdown()

    def _tick(self) -> None:
        try:
            if self.state == DigState.DONE:
                return

            if self._phase_elapsed() > self.phase_timeout_sec:
                self._abort("Phase timed out.")
                return

            if self.state == DigState.WAIT_NAV_ARM:
                if self._nav_dig_arm:
                    self.get_logger().info("/autonomy/dig_arm true — starting dig setup (nav handoff).")
                    self.state = DigState.SETUP_IR
                    self.setup_complete = False
                    self._reset_phase_clock()
                self._sync_conveyor_output()
                return

            if self.state == DigState.SETUP_IR:
                self._tick_setup_ir()
            elif self.state == DigState.DRIVE_FORWARD:
                self._tick_drive_forward()
            elif self.state == DigState.DRIVE_BACK:
                self._tick_drive_back()
            elif self.state == DigState.CYCLE_END_CONVEYOR:
                self._tick_cycle_end_conveyor()

            self._sync_conveyor_output()
            self._sync_bucket_chain_output()
            self._log_status_throttled()
        finally:
            self._publish_dig_state()

    def _tick_setup_ir(self) -> None:
        if not self.setup_complete:
            self._reset_phase_clock()
            self.bucket_pos_commanded = min(DIG_BUCKET_POS_MAX, self.bucket_start_pos)
            self.pub_bucket_pos.publish(Int16(data=int(self.bucket_pos_commanded)))
            self.pub_bucket_vel.publish(Int16(data=int(self.bucket_chain_speed)))
            self.setup_complete = True
            self._ir_setup_was_above_target = False
            self._ir_bucket_gate_waiting = False
            self.ir_last_step_time = self.get_clock().now()

        if self.ir_setup_mode == "eq":
            ir_met = ir_setup_stop_eq(self.ir_value, self.ir_target)
        else:
            ir_met, self._ir_setup_was_above_target = ir_setup_stop_le(
                self.ir_value,
                self.ir_target,
                self.bucket_pos_commanded,
                self.bucket_start_pos,
                self._ir_setup_was_above_target,
            )

        if ir_met:
            self._ir_bucket_gate_waiting = False
            self.keep_bucket_chain_until_done = True
            if self.ir_setup_mode == "eq":
                self.get_logger().info(
                    "IR exact match at target; setup complete "
                    f"(belt stays at bucket_chain_speed={self.bucket_chain_speed})."
                )
            else:
                self.get_logger().info(
                    f"IR reached at-or-below target (ir_value={self.ir_value}, "
                    f"ir_target={self.ir_target}); setup complete "
                    f"(belt stays at bucket_chain_speed={self.bucket_chain_speed})."
                )
            self._stop_motion()
            self.state = DigState.DRIVE_FORWARD
            self._reset_phase_clock()
            return

        if self.bucket_pos_commanded >= self.bucket_safety_stop:
            self._ir_bucket_gate_waiting = False
            self.keep_bucket_chain_until_done = True
            if self.ir_setup_mode == "eq":
                warn_tail = (
                    f"(IR never matched exact ir_target={self.ir_target}; latest ir_value={self.ir_value})"
                )
            else:
                warn_tail = (
                    f"(expected IR to descend to <= {self.ir_target} after reading above target; "
                    f"latest ir_value={self.ir_value})"
                )
            self.get_logger().warn(
                f"Bucket position safety stop at {self.bucket_safety_stop} {warn_tail}; "
                "keeping dig motors running until the sequence terminates."
            )
            self._stop_motion()
            self.state = DigState.DRIVE_FORWARD
            self._reset_phase_clock()
            return

        step_elapsed = (self.get_clock().now() - self.ir_last_step_time).nanoseconds / 1e9
        if step_elapsed < self.ir_bucket_step_every_sec:
            return

        if self.ir_bucket_gate_min_drop > 0 and self._ir_bucket_gate_waiting:
            elapsed_gate = time.monotonic() - self._ir_bucket_gate_t0
            if not ir_bucket_step_gate_released(
                self.ir_value,
                self._ir_anchor_before_last_bucket_step,
                self.ir_bucket_gate_min_drop,
                elapsed_sec=elapsed_gate,
                timeout_sec=self.ir_bucket_gate_timeout_sec,
            ):
                return
            self._ir_bucket_gate_waiting = False

        ir_before = int(self.ir_value)
        self.bucket_pos_commanded = min(DIG_BUCKET_POS_MAX, self.bucket_pos_commanded + 1)
        self.pub_bucket_pos.publish(Int16(data=int(self.bucket_pos_commanded)))
        self.ir_last_step_time = self.get_clock().now()

        if self.ir_bucket_gate_min_drop > 0:
            self._ir_bucket_gate_waiting = True
            self._ir_anchor_before_last_bucket_step = ir_before
            self._ir_bucket_gate_t0 = time.monotonic()

    def _tick_drive_forward(self) -> None:
        if self._timed_drive_only:
            reached = timed_leg_complete(self._phase_elapsed(), self.timed_drive_ms)
            if reached:
                self.get_logger().info(f"Forward phase finished after {self.timed_drive_ms} ms (timed).")
                self._stop_motion()
                self.state = DigState.DRIVE_BACK
                self._reset_phase_clock()
                return
        else:
            reached_enc = encoder_forward_target_reached(
                self.encoder_value,
                self.calibrated_rotary,
                self.encoder_tolerance,
                self.forward_encoder_increases,
            )
            if reached_enc:
                self.get_logger().info(f"Encoder {self.encoder_value} reached forward target ~{self.calibrated_rotary}.")
                self._stop_motion()
                self.state = DigState.DRIVE_BACK
                self._reset_phase_clock()
                return
        allow, detail = self._terrain_gate_forward()
        if not allow:
            self._maybe_warn_terrain(f"Dig drive held: forward blocked ({detail})")
            self._stop_motion()
            return
        self._publish_vel(self.forward_linear)

    def _tick_drive_back(self) -> None:
        if self._timed_drive_only:
            if timed_leg_complete(self._phase_elapsed(), self.timed_drive_ms):
                self.get_logger().info(f"Backward phase finished after {self.timed_drive_ms} ms (timed).")
                self._stop_motion()
                self._apply_post_cycle_bump_and_maybe_repeat()
                return
        elif encoder_returned_home(self.encoder_value, self.encoder_tolerance):
            self.get_logger().info("Encoder returned to ~0.")
            self._stop_motion()
            self._apply_post_cycle_bump_and_maybe_repeat()
            return
        allow, detail = self._terrain_gate_reverse()
        if not allow:
            self._maybe_warn_terrain(f"Dig drive held: reverse blocked ({detail})")
            self._stop_motion()
            return
        self._publish_vel(self.backward_linear)

    def _tick_cycle_end_conveyor(self) -> None:
        if self.conveyor_until is None:
            self.conveyor_until = self.get_clock().now() + rclpy.duration.Duration(
                seconds=self.end_cycle_conveyor_seconds
            )
        now = self.get_clock().now()
        if now < self.conveyor_until:
            return
        if self._conveyor_end_applied:
            return
        self._conveyor_end_applied = True
        self.get_logger().info("End-of-cycle conveyor pass complete; repeating drive-forward phase.")
        self.state = DigState.DRIVE_FORWARD
        self.conveyor_until = None
        self._reset_phase_clock()
def main(args=None):
    rclpy.init(args=args)
    node = DigSequenceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._stop_motion()
        node.pub_bucket_vel.publish(Int16(data=0))
        node.pub_conveyor.publish(Int16(data=0))
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
