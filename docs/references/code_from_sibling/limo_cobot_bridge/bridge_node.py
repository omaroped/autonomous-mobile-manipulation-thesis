#!/usr/bin/env python3
"""
bridge_node.py — Orchestrates the autonomous pick-and-place state machine.

State flow:
  INIT → NAV_TO_PICK → DETECT → PICK → NAV_TO_PLACE → PLACE → NAV_HOME → DONE
                                                                 ↓ (any state)
                                                               ABORT

Navigation timeout fix (2024-10):
  The robot spawns at x ≈ -8.95 and the pick zone is at x = 0.60.
  That is 9.55 m.  At the old approach_speed of 0.12 m/s the minimum
  travel time is ~80 s, but the timeout was hardcoded to 30 s.
  Fix:
    • cruise_speed (0.30 m/s) is used for the long traverse.
    • approach_speed (0.12 m/s) is used only for the final homing move.
    • Timeouts are read from bridge_params.yaml (no more hardcoded values).
    • A helper _compute_nav_timeout() lets callers override the value from
      params if they know the exact distance at runtime.
"""

import math
import os
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates

from mycobot_interfaces.srv import GetPlanningScene

# ---------------------------------------------------------------------------
# State enumeration
# ---------------------------------------------------------------------------
class State:
    INIT          = "INIT"
    NAV_TO_PICK   = "NAV_TO_PICK"
    DETECT        = "DETECT"
    PICK          = "PICK"
    NAV_TO_PLACE  = "NAV_TO_PLACE"
    PLACE         = "PLACE"
    NAV_HOME      = "NAV_HOME"
    DONE          = "DONE"
    ABORT         = "ABORT"


class BridgeNode(Node):

    def __init__(self):
        super().__init__("bridge_node")
        self._cb_group = ReentrantCallbackGroup()

        # ── Declare and read parameters ──────────────────────────────────
        self._declare_parameters()
        self._read_parameters()

        # ── ROS interfaces ───────────────────────────────────────────────
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self._odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_callback, 10,
            callback_group=self._cb_group)

        self._gazebo_sub = self.create_subscription(
            ModelStates, "/gazebo/model_states", self._gazebo_callback, 10,
            callback_group=self._cb_group)

        self._perception_client = self.create_client(
            GetPlanningScene, "/get_planning_scene_mycobot",
            callback_group=self._cb_group)

        # ── Runtime state ────────────────────────────────────────────────
        self._current_x   = 0.0
        self._current_y   = 0.0
        self._current_yaw = 0.0
        self._odom_received = False

        self._state = State.INIT
        self._detected_object_id = ""

        self.get_logger().info("BridgeNode initialised. Entering state machine.")
        # Give odometry time to arrive before we start moving (10 Hz control loop)
        self._main_timer = self.create_timer(
            0.1, self._step, callback_group=self._cb_group)

    # ------------------------------------------------------------------ params

    def _declare_parameters(self):
        self.declare_parameter("pick_zone_x",   0.60)
        self.declare_parameter("pick_zone_y",  -0.22)
        self.declare_parameter("pick_zone_yaw", 0.0)

        self.declare_parameter("place_zone_x",  -1.0)
        self.declare_parameter("place_zone_y",   0.5)
        self.declare_parameter("place_zone_yaw", 0.0)

        # Speeds
        self.declare_parameter("cruise_speed",   0.30)   # m/s — long traversal
        self.declare_parameter("approach_speed", 0.12)   # m/s — final homing

        # Timeouts (seconds) — read from yaml, NOT hardcoded
        self.declare_parameter("nav_to_pick_timeout",  90.0)
        self.declare_parameter("nav_to_place_timeout", 60.0)
        self.declare_parameter("nav_to_home_timeout",  75.0)

        self.declare_parameter("nav_xy_tolerance",  0.05)
        self.declare_parameter("nav_yaw_tolerance", 0.087)  # ~5 deg

        self.declare_parameter("perception_timeout", 10.0)
        self.declare_parameter("target_shape", "box")
        self.declare_parameter("target_dimensions", [0.03, 0.03, 0.03])

        self.declare_parameter("mtc_planning_timeout",   30.0)
        self.declare_parameter("mtc_execution_timeout",  60.0)

    def _read_parameters(self):
        p = self.get_parameter

        self._pick_x   = p("pick_zone_x").value
        self._pick_y   = p("pick_zone_y").value
        self._pick_yaw = p("pick_zone_yaw").value

        self._place_x   = p("place_zone_x").value
        self._place_y   = p("place_zone_y").value
        self._place_yaw = p("place_zone_yaw").value

        # Initialise spawn coordinates to 0.0 (resolved dynamically in INIT state)
        self._spawn_x   = 0.0
        self._spawn_y   = 0.0
        self._spawn_yaw = 0.0

        self._cruise_speed   = p("cruise_speed").value
        self._approach_speed = p("approach_speed").value

        self._nav_to_pick_timeout  = p("nav_to_pick_timeout").value
        self._nav_to_place_timeout = p("nav_to_place_timeout").value
        self._nav_to_home_timeout  = p("nav_to_home_timeout").value

        self._xy_tol  = p("nav_xy_tolerance").value
        self._yaw_tol = p("nav_yaw_tolerance").value

        self._perception_timeout  = p("perception_timeout").value
        self._target_shape        = p("target_shape").value
        self._target_dimensions   = p("target_dimensions").value

        self._mtc_plan_timeout = p("mtc_planning_timeout").value
        self._mtc_exec_timeout = p("mtc_execution_timeout").value

        # Log the resolved timeouts so they appear in the startup logs
        # for easy sanity-checking — no more silent 30 s surprises.
        self.get_logger().info(
            f"Navigation timeouts (s): "
            f"pick={self._nav_to_pick_timeout}, "
            f"place={self._nav_to_place_timeout}, "
            f"home={self._nav_to_home_timeout}"
        )
        self.get_logger().info(
            f"Navigation speeds (m/s): "
            f"cruise={self._cruise_speed}, "
            f"approach={self._approach_speed}"
        )

    # ------------------------------------------------------------------ odom

    def _odom_callback(self, msg: Odometry):
        # Prefer gazebo ground truth if active to bypass wheel slippage, fallback to odom
        if not hasattr(self, "_gazebo_active") or not self._gazebo_active:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            self._current_x = pos.x
            self._current_y = pos.y
            self._current_yaw = self._quat_to_yaw(ori.x, ori.y, ori.z, ori.w)
            self._odom_received = True

    def _gazebo_callback(self, msg: ModelStates):
        try:
            idx = msg.name.index("limo_cobot")
            pos = msg.pose[idx].position
            ori = msg.pose[idx].orientation
            self._current_x = pos.x
            self._current_y = pos.y
            self._current_yaw = self._quat_to_yaw(ori.x, ori.y, ori.z, ori.w)
            self._odom_received = True
            self._gazebo_active = True
        except ValueError:
            pass

    @staticmethod
    def _quat_to_yaw(x, y, z, w) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ------------------------------------------------------------------ helpers

    def _distance_to(self, tx: float, ty: float) -> float:
        dx = tx - self._current_x
        dy = ty - self._current_y
        return math.sqrt(dx * dx + dy * dy)

    def _angle_to(self, tx: float, ty: float) -> float:
        """Signed angular error from current yaw to (tx, ty)."""
        desired = math.atan2(ty - self._current_y, tx - self._current_x)
        err = desired - self._current_yaw
        # Normalise to [-π, π]
        while err >  math.pi: err -= 2 * math.pi
        while err < -math.pi: err += 2 * math.pi
        return err

    def _compute_nav_timeout(self, tx: float, ty: float,
                             override: float | None = None) -> float:
        """
        Returns a timeout that is guaranteed to be long enough for the
        robot to reach (tx, ty) even if only cruise_speed is used, plus
        a 25 s margin for ramp-up, replanning, and settling.

        If override is supplied (e.g. from the yaml parameter) and it is
        already larger than the computed minimum, the override is used.
        """
        dist    = self._distance_to(tx, ty)
        t_drive = dist / self._cruise_speed
        t_min   = t_drive + 25.0          # 25 s fixed margin

        if override is not None and override >= t_min:
            return override
        if override is not None and override < t_min:
            self.get_logger().warn(
                f"nav timeout from params ({override:.1f}s) is less than "
                f"computed minimum ({t_min:.1f}s for {dist:.2f} m at "
                f"{self._cruise_speed} m/s). Using computed value."
            )
        return t_min

    def _drive_toward(self, tx: float, ty: float, speed: float):
        """Publish a single Twist command aimed at (tx, ty)."""
        angle_err = self._angle_to(tx, ty)
        dist      = self._distance_to(tx, ty)

        cmd = Twist()
        # Rotate in-place if badly misaligned (> 20 deg), otherwise drive
        if abs(angle_err) > 0.35:
            cmd.angular.z = 0.5 * (1.0 if angle_err > 0 else -1.0)
        else:
            cmd.linear.x  = min(speed, dist)        # don't overshoot
            cmd.angular.z = 1.0 * angle_err         # proportional heading correction
        self._cmd_vel_pub.publish(cmd)

    def _stop(self):
        self._cmd_vel_pub.publish(Twist())

    def _transition(self, new_state: str):
        self.get_logger().info(
            f"State: {self._state} → {new_state}")
        self._state = new_state

    def _abort(self, reason: str):
        self._stop()
        self.get_logger().error(f"ABORT: {reason}")
        self._transition(State.ABORT)
        # NOTE: Do NOT cancel the timer! We must keep publishing zero velocity
        # (electronic brake) even after abort, otherwise the ackermann controller
        # cannot hold position against physics forces and the robot drifts.

    # ------------------------------------------------------------------ states

    def _step(self):
        """Called by the 1 Hz timer. Dispatches to the current state handler."""
        if not self._odom_received:
            self.get_logger().info("Waiting for first /odom message…")
            return

        {
            State.INIT:        self._state_init,
            State.NAV_TO_PICK: self._state_nav_to_pick,
            State.DETECT:      self._state_detect,
            State.PICK:        self._state_pick,
            State.NAV_TO_PLACE:self._state_nav_to_place,
            State.PLACE:       self._state_place,
            State.NAV_HOME:    self._state_nav_home,
            State.DONE:        self._state_done,
            State.ABORT:       self._state_abort,
        }.get(self._state, lambda: None)()

    # ── INIT ────────────────────────────────────────────────────────────────

    def _state_init(self):
        # Dynamically capture spawn odometry coordinates as baseline reference for returning home
        self._spawn_x = self._current_x
        self._spawn_y = self._current_y
        self._spawn_yaw = self._current_yaw

        self.get_logger().info(
            f"Spawn position captured dynamically: x={self._spawn_x:.3f} y={self._spawn_y:.3f} "
            f"yaw={math.degrees(self._spawn_yaw):.1f}°"
        )
        self.get_logger().info(
            f"Target coordinates (absolute Gazebo):"
            f"\n  Pick:  x={self._pick_x:.3f} y={self._pick_y:.3f}"
            f"\n  Place: x={self._place_x:.3f} y={self._place_y:.3f}"
        )

        self._nav_start_time = time.monotonic()
        # Compute the actual timeout from the resolved target coordinates
        self._current_nav_timeout = self._compute_nav_timeout(
            self._pick_x, self._pick_y, override=self._nav_to_pick_timeout)
        self.get_logger().info(
            f"NAV_TO_PICK: target=({self._pick_x:.3f},{self._pick_y:.3f}) "
            f"distance={self._distance_to(self._pick_x, self._pick_y):.2f} m "
            f"timeout={self._current_nav_timeout:.1f} s"
        )
        self._transition(State.NAV_TO_PICK)

    # ── NAV_TO_PICK ──────────────────────────────────────────────────────────

    def _state_nav_to_pick(self):
        dist    = self._distance_to(self._pick_x, self._pick_y)
        elapsed = time.monotonic() - self._nav_start_time

        if dist < self._xy_tol:
            self._stop()
            self.get_logger().info(
                f"Arrived at pick zone in {elapsed:.1f} s "
                f"(remaining dist {dist:.3f} m)"
            )
            self._transition(State.DETECT)
            return

        if elapsed > self._current_nav_timeout:
            self._abort(
                f"NAV_TO_PICK timed out after {elapsed:.1f} s "
                f"(timeout={self._current_nav_timeout:.1f} s, "
                f"remaining dist={dist:.2f} m). "
                f"Hint: spawn_x={self._spawn_x}, pick_x={self._pick_x}, "
                f"cruise_speed={self._cruise_speed} m/s"
            )
            return

        # Use cruise speed for the long traverse, approach speed for the
        # final 0.5 m so we stop precisely on the pick pose.
        speed = self._approach_speed if dist < 0.5 else self._cruise_speed
        self._drive_toward(self._pick_x, self._pick_y, speed)

        if int(elapsed) % 5 == 0 and (not hasattr(self, "_last_log_second") or self._last_log_second != int(elapsed)):
            self._last_log_second = int(elapsed)
            self.get_logger().info(
                f"NAV_TO_PICK: pose=({self._current_x:.3f}, {self._current_y:.3f}, {math.degrees(self._current_yaw):.1f}°)  "
                f"target=({self._pick_x:.3f}, {self._pick_y:.3f})  "
                f"dist={dist:.2f} m  elapsed={elapsed:.0f} s  "
                f"speed={speed} m/s  yaw_err={math.degrees(self._angle_to(self._pick_x, self._pick_y)):.1f}°"
            )

    # ── DETECT ───────────────────────────────────────────────────────────────

    def _state_detect(self):
        if hasattr(self, "_perception_called") and self._perception_called:
            self._stop()
            return
        self._perception_called = True
        self.get_logger().info("Calling perception service…")

        if not self._perception_client.wait_for_service(timeout_sec=5.0):
            self._perception_called = False
            self._abort("/get_planning_scene_mycobot service not available")
            return

        req = GetPlanningScene.Request()
        req.target_shape      = self._target_shape
        req.target_dimensions = self._target_dimensions

        future = self._perception_client.call_async(req)
        future.add_done_callback(self._perception_response_cb)

    def _perception_response_cb(self, future):
        self._perception_called = False
        try:
            resp = future.result()
        except Exception as e:
            self._abort(f"Perception service threw: {e}")
            return

        if not resp.success or not resp.target_object_id:
            self._abort(
                f"Perception failed: success={resp.success} "
                f"id='{resp.target_object_id}'"
            )
            return

        self._detected_object_id = resp.target_object_id
        self.get_logger().info(
            f"Detected object: id={self._detected_object_id}")
        self._transition(State.PICK)

    # ── PICK ─────────────────────────────────────────────────────────────────

    def _state_pick(self):
        """Launch mtc_node as a subprocess in background and keep robot stopped."""
        if not hasattr(self, "_mtc_launched") or not self._mtc_launched:
            self._mtc_launched = True
            self.get_logger().info(
                f"Launching mtc_node for object '{self._detected_object_id}'  "
                f"(plan_timeout={self._mtc_plan_timeout}s, "
                f"exec_timeout={self._mtc_exec_timeout}s)"
            )

            def _run_mtc():
                ws = os.path.expanduser(
                    os.environ.get("ROS_WS",
                        "/home/omar/Desktop/Thesis/src/ros2_ws"))
                setup  = os.path.join(ws, "install", "setup.bash")
                cmd = (
                    f"source /opt/ros/humble/setup.bash && "
                    f"source {setup} && "
                    f"ros2 launch limo_cobot_bridge run_mtc.launch.py"
                )
                self.get_logger().info(f"mtc_node cmd: {cmd}")
                result = subprocess.run(
                    cmd, shell=True, executable="/bin/bash",
                    timeout=self._mtc_plan_timeout + self._mtc_exec_timeout + 30.0
                )
                if result.returncode == 0:
                    self.get_logger().info("mtc_node completed successfully → NAV_HOME")
                    self._transition(State.NAV_HOME)
                    self._nav_start_time = time.monotonic()
                    self._current_nav_timeout = self._compute_nav_timeout(
                        self._spawn_x, self._spawn_y,
                        override=self._nav_to_home_timeout)
                    self._mtc_launched = False
                else:
                    self.get_logger().error(
                        f"mtc_node exited with code {result.returncode}")
                    self._abort(f"mtc_node failed (exit code {result.returncode})")
                    self._mtc_launched = False
                    return

            threading.Thread(target=_run_mtc, daemon=True).start()

        # Continuously publish zero velocity (electronic braking) at 10Hz to hold position
        self._stop()

    # ── NAV_TO_PLACE ─────────────────────────────────────────────────────────

    def _state_nav_to_place(self):
        dist    = self._distance_to(self._place_x, self._place_y)
        elapsed = time.monotonic() - self._nav_start_time

        if dist < self._xy_tol:
            self._stop()
            self.get_logger().info(
                f"Arrived at place zone in {elapsed:.1f} s")
            self._transition(State.PLACE)
            return

        if elapsed > self._current_nav_timeout:
            self._abort(
                f"NAV_TO_PLACE timed out after {elapsed:.1f} s "
                f"(timeout={self._current_nav_timeout:.1f} s, "
                f"remaining dist={dist:.2f} m)"
            )
            return

        speed = self._approach_speed if dist < 0.5 else self._cruise_speed
        self._drive_toward(self._place_x, self._place_y, speed)

    # ── PLACE ────────────────────────────────────────────────────────────────

    def _state_place(self):
        self.get_logger().info("Executing place…  (MTC)")
        # TODO: call the MTC place action here
        self._transition(State.NAV_HOME)
        self._nav_start_time = time.monotonic()
        self._current_nav_timeout = self._compute_nav_timeout(
            self._spawn_x, self._spawn_y, override=self._nav_to_home_timeout)

    # ── NAV_HOME ─────────────────────────────────────────────────────────────

    def _state_nav_home(self):
        dist    = self._distance_to(self._spawn_x, self._spawn_y)
        elapsed = time.monotonic() - self._nav_start_time

        if dist < self._xy_tol:
            self._stop()
            self.get_logger().info(
                f"Returned home in {elapsed:.1f} s")
            self._transition(State.DONE)
            return

        if elapsed > self._current_nav_timeout:
            self._abort(
                f"NAV_HOME timed out after {elapsed:.1f} s "
                f"(timeout={self._current_nav_timeout:.1f} s, "
                f"remaining dist={dist:.2f} m)"
            )
            return

        speed = self._cruise_speed
        self._drive_toward(self._spawn_x, self._spawn_y, speed)

    # ── DONE ─────────────────────────────────────────────────────────────────

    def _state_done(self):
        if not hasattr(self, '_done_logged') or not self._done_logged:
            self.get_logger().info("Pick-and-place cycle complete.")
            self._done_logged = True
        # Keep publishing zero velocity to hold position
        self._stop()

    def _state_abort(self):
        """Keep publishing zero velocity to prevent drift after abort."""
        self._stop()


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
