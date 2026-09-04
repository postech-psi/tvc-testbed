"""
Software-in-the-loop harness: gz-sim plant + the same flight code, no ROS.
================================================================================
Talks to Gazebo directly over gz-transport, so it runs against a bare `gz sim`
with no colcon build. That is its only reason to exist: it is the fastest way to
get the real physics engine under the real controller. The ROS 2 path
(nodes/ + launch/gazebo.launch.py) is the architecture that ships.

WHAT CHANGED, AND WHY IT MATTERS
    This file used to contain a SECOND, INDEPENDENT CONTROLLER -- its own
    altitude cascade, position loop, attitude cascade, roll loop and gimbal
    inverse, written as module-level constants and one 90-line step(). It was
    also the only controller in this project that had ever flown, so the repo
    was in the position of testing one implementation and flying another.

    Now it does what a harness does and nothing else: read odometry, build an
    EstimatedState, call TvcController, hand the result to the Gazebo HAL. The
    control law is byte-identical to the analytic harness and to the ROS 2 node.

    The gains did not change with it. control_gains.yaml's default profile
    `flight_validated` IS this file's old set, ported into inertia-normalized
    units, so the loop dynamics are the ones that flew. What the shared code
    adds on top is a better allocator: the measured feasible set instead of a
    flat 4 N split cap, asymmetric per-ring gimbal stops instead of one
    symmetric limit, and three Newton steps on the exact trigonometric map
    instead of the small-angle inverse alone.

    Not yet re-flown -- gz-transport exists only in the devcontainer. See
    docs/7-CREDIBILITY.md.

LOCKSTEP
    The loop steps on ODOMETRY ARRIVAL, not on a wall clock. Gazebo publishes
    odometry at 250 Hz of SIMULATED time, so stepping in the callback ties the
    control rate to the plant rather than to the host. A wall-clock
    `while time.time() < end: step(); sleep(dt)` loop -- what this used to do --
    is not reproducible, which means no tolerance against it means anything, and
    the effective control rate drifts with the real-time factor. PX4's SITL is
    locked to its simulator for exactly this reason.

Usage (with `gz sim` already running on the world):
    python tvc.py hover --duration 30 --altitude 2.0
"""
import argparse
import csv
import math
import time

from ..config import load, load_gains, load_vehicle_params
from ..gnc.controller import TvcController
from ..gnc.mathx import quat_to_euler
from ..gnc.types import ControlMode, EstimatedState, Setpoint
from ..hal.gazebo import rotor_speeds

ODOM_TOPIC = "/model/tvc_vehicle/odometry"
MOTOR_TOPIC = "/tvc_vehicle/command/motor_speed"
INNER_TOPIC = "/tvc_vehicle/gimbal_inner_cmd"
OUTER_TOPIC = "/tvc_vehicle/gimbal_outer_cmd"

CSV_HEADER = ["t_s", "z_m", "x_m", "y_m", "pitch_deg", "yaw_deg", "roll_deg",
              "wz_rads", "gimbal_outer_deg", "gimbal_inner_deg", "thrust_N",
              "tau_p_Nm"]


class GazeboHarness:
    """One control step per odometry message. Owns the clock and the transport."""

    def __init__(self, target_alt, rate_hz=250.0, verbose=True, log_path=None):
        # gz-transport is a devcontainer-only dependency, so it is imported here
        # rather than at module scope: `python tvc.py --help` and the test suite
        # must work on a host that has never seen Gazebo.
        from gz.transport13 import Node
        from gz.msgs10.actuators_pb2 import Actuators
        from gz.msgs10.double_pb2 import Double
        from gz.msgs10.odometry_pb2 import Odometry

        self._Actuators, self._Double = Actuators, Double

        self.params = load_vehicle_params()
        self.vehicle = load()
        rot = self.vehicle.raw["rotors"]
        self._k = rot["motor_constant"]
        self._c = rot["moment_constant"]
        self._wmax = rot["max_rot_velocity"]

        self.controller = TvcController(
            self.params, load_gains(),
            ControlMode(altitude_hold=True, position_hold=True))
        self.setpoint = Setpoint(z_des=target_alt, pos_des=(0.0, 0.0, target_alt))

        self.dt = 1.0 / rate_hz
        self.verbose = verbose
        self.log_path = log_path
        self.log = []
        self.state = None
        self.steps_done = 0
        self.running = False
        self._t_sim = 0.0
        self._last_print = 0.0

        self.node = Node()
        self.pub_motor = self.node.advertise(MOTOR_TOPIC, Actuators)
        self.pub_inner = self.node.advertise(INNER_TOPIC, Double)
        self.pub_outer = self.node.advertise(OUTER_TOPIC, Double)
        if not self.node.subscribe(Odometry, ODOM_TOPIC, self._on_odom):
            raise RuntimeError(
                "could not subscribe to %s -- is `gz sim` running?" % ODOM_TOPIC)

    # --- transport in ---------------------------------------------------------
    def _on_odom(self, msg):
        """Cache the state as an EstimatedState, then step.

        Seam A: the controller is handed an ESTIMATE even though this one is
        ground truth relabelled. gz reports the quaternion as (w,x,y,z) fields
        and body rates in twist.angular, which is already the gnc convention.
        """
        p, q, tw = msg.pose.position, msg.pose.orientation, msg.twist
        self.state = EstimatedState(
            pos_i=(p.x, p.y, p.z),
            vel_i=(tw.linear.x, tw.linear.y, tw.linear.z),
            quat=(q.w, q.x, q.y, q.z),
            omega_b=(tw.angular.x, tw.angular.y, tw.angular.z),
            stamp_s=self._t_sim,
        )
        if self.running:
            self.steps_done += 1
            self._t_sim += self.dt
            self.step()

    # --- one control step -----------------------------------------------------
    def step(self):
        """One control step: state -> TvcController -> HAL -> the three gz topics."""
        if self.state is None:
            return

        cmd = self.controller.update(self.state, self.setpoint, self.dt)
        wa, wb = rotor_speeds(cmd.thrust_n, cmd.tau_p_nm,
                              self._k, self._c, self._wmax)

        act = self._Actuators()
        act.velocity.extend([wa, wb])
        self.pub_motor.publish(act)
        self.pub_inner.publish(self._Double(data=cmd.gimbal_inner_rad))
        self.pub_outer.publish(self._Double(data=cmd.gimbal_outer_rad))

        self._record(cmd)

    def _record(self, cmd):
        s = self.state
        pitch, yaw, roll = quat_to_euler(s.quat)
        if self.log_path is not None:
            self.log.append((
                self._t_sim, s.pos_i[2], s.pos_i[0], s.pos_i[1],
                math.degrees(pitch), math.degrees(yaw), math.degrees(roll),
                s.omega_b[2],
                math.degrees(cmd.gimbal_outer_rad),
                math.degrees(cmd.gimbal_inner_rad),
                cmd.thrust_n, cmd.tau_p_nm))
        if self.verbose:
            now = time.time()
            if now - self._last_print > 1.0:
                self._last_print = now
                print("t=%5.1f  z=%5.2f  xy=(%+5.2f,%+5.2f)  "
                      "pyr=(%+6.1f,%+6.1f,%+7.1f)  gimbal=(%+5.1f,%+5.1f)  "
                      "T=%5.2f  tau_P=%+.3f%s"
                      % (self._t_sim, s.pos_i[2], s.pos_i[0], s.pos_i[1],
                         math.degrees(pitch), math.degrees(yaw),
                         math.degrees(roll),
                         math.degrees(cmd.gimbal_outer_rad),
                         math.degrees(cmd.gimbal_inner_rad),
                         cmd.thrust_n, cmd.tau_p_nm,
                         "  SAT" if (cmd.sat_gimbal or cmd.sat_roll
                                     or cmd.sat_thrust) else ""),
                      flush=True)

    def write_log(self):
        """Write the flight log CSV, with its axis-convention header line."""
        if not (self.log_path and self.log):
            return
        with open(self.log_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            # The convention token travels with the data: plot.py refuses a log
            # whose axis names it cannot trust. See docs/4-CONVENTIONS.md.
            w.writerow(["# axis_convention: %s"
                        % self.vehicle.raw["axis_convention"]])
            w.writerow(CSV_HEADER)
            w.writerows(self.log)
        print("wrote %s (%d samples)" % (self.log_path, len(self.log)))


# --- pass/fail thresholds for the hover demo ---------------------------------
# Loose on purpose. This is a smoke test that the vehicle stays where it was
# put, not a performance grade; tightening it would make ordinary retuning look
# like a regression.
ALT_ERR_MAX_M = 0.30
TILT_MAX_DEG = 15.0
DRIFT_MAX_M = 1.00


def main(argv=None):
    """Run the hover demo. Returns 0 on a stable hover, 2 otherwise."""
    ap = argparse.ArgumentParser(
        prog="tvc.py hover",
        description="Hover the Gazebo vehicle under the shared flight code.")
    ap.add_argument("--altitude", type=float, default=2.0, help="target altitude [m]")
    ap.add_argument("--duration", type=float, default=25.0, help="simulated seconds")
    ap.add_argument("--rate", type=float, default=250.0,
                    help="control rate [Hz]; must match the odometry publisher")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--log", help="write per-sample flight data to this CSV")
    args = ap.parse_args(argv)

    h = GazeboHarness(args.altitude, rate_hz=args.rate,
                      verbose=not args.quiet, log_path=args.log)

    print("waiting for odometry on %s ..." % ODOM_TOPIC, flush=True)
    for _ in range(200):
        if h.state is not None:
            break
        time.sleep(0.05)
    if h.state is None:
        print("no odometry. Start the world first:\n"
              "  bash gazebo/run_hover.sh")
        return 1

    print("hovering to %.1f m for %.0f simulated seconds\n"
          % (args.altitude, args.duration), flush=True)
    h.running = True
    target_steps = int(args.duration * args.rate)
    # Wall-clock safety net only. The run ends on simulated steps; this stops a
    # simulator running far below real time from hanging the script forever.
    deadline = time.time() + args.duration * 3.0 + 10.0
    while h.steps_done < target_steps and time.time() < deadline:
        time.sleep(0.05)
    h.running = False
    if h.steps_done < target_steps:
        print("WARNING: %d of %d control steps ran before the wall-clock limit "
              "-- the simulation is far below real time."
              % (h.steps_done, target_steps))

    s = h.state
    pitch, yaw, _ = quat_to_euler(s.quat)
    err = abs(s.pos_i[2] - args.altitude)
    tilt = math.degrees(math.hypot(pitch, yaw))
    drift = math.hypot(s.pos_i[0], s.pos_i[1])
    print("\nfinal: z=%.2f m (err %.2f)  tilt=%.1f deg  drift=%.2f m"
          % (s.pos_i[2], err, tilt, drift))
    h.write_log()

    ok = (err < ALT_ERR_MAX_M and tilt < TILT_MAX_DEG and drift < DRIFT_MAX_M)
    print("HOVER OK" if ok else "NOT STABLE -- see the numbers above")
    return 0 if ok else 2
