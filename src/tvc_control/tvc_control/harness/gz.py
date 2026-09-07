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

    This shared path has since been re-flown and frozen as the Gazebo hover
    baseline. See docs/7-CREDIBILITY.md.

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
import json
import math
import os
import time

from ..config import load, load_gains, load_vehicle_params
from ..gnc.controller import TvcController
from ..gnc.mathx import quat_rotate, quat_to_euler
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
        self._t0 = None            # first accepted stamp, so the log starts at 0
        self._last_stamp = None    # previous accepted stamp, for dt
        self._prev = None          # (pos, stamp) of the previous message
        self.rejected = 0          # messages whose twist contradicted the pose
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

        TWO THINGS THIS MESSAGE GETS WRONG, AND WHY THEY ARE HANDLED HERE
        1. The twist is in the CHILD (body) frame -- REP-105, and confirmed by
           measurement: in free fall from the world's 12.2 deg tilt the reported
           velocity is (-0.40, -0.57, -3.22) where the world velocity is purely
           vertical, and 3.295*sin(12.2 deg) = 0.696 = hypot(0.40, 0.57) to three
           digits. EstimatedState.vel_i is inertial by definition, so it is
           rotated. Feeding it through unrotated is exact only while level.
        2. The first messages carry a garbage twist. OdometryPublisher
           differences the pose against a zero-initialised `lastUpdatePose`, so
           the first sample reports the whole spawn pose divided by one step:
           651 m/s and 58 rad/s, measured. The controller's rate loop sees that
           as an enormous error and saturates the gimbal on step one.

        The gate for (2) is not a count of samples to skip and not a tuned speed
        limit. The twist has to agree with the pose derivative, which we can see
        independently; on the garbage samples the position has not moved at all
        while the twist claims hundreds of m/s, so they fail by four orders of
        magnitude. The 2x factor and the 1 m/s floor are slack for the
        publisher's own smoothing, not a threshold anything is tuned to.
        """
        p, q, tw = msg.pose.position, msg.pose.orientation, msg.twist
        pos = (p.x, p.y, p.z)
        quat = (q.w, q.x, q.y, q.z)
        v_body = (tw.linear.x, tw.linear.y, tw.linear.z)
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9

        if not self._twist_agrees_with_pose(pos, v_body, stamp):
            self.rejected += 1
            self._prev = (pos, stamp)
            return
        self._prev = (pos, stamp)

        # dt from the SIMULATOR's clock, not from the nominal rate: if the
        # odometry publisher and --rate disagree, every derivative and every
        # integrator in the cascade is scaled by the ratio, silently.
        nominal = self.dt
        dt = nominal if self._last_stamp is None else stamp - self._last_stamp
        dt = min(max(dt, 0.2 * nominal), 5.0 * nominal)
        self._last_stamp = stamp

        self.state = EstimatedState(
            pos_i=pos,
            vel_i=quat_rotate(quat, v_body),
            quat=quat,
            omega_b=(tw.angular.x, tw.angular.y, tw.angular.z),
            stamp_s=stamp,
        )
        if self.running:
            if self._t0 is None:
                self._t0 = stamp
            self._t_sim = stamp - self._t0
            self.steps_done += 1
            self.step(dt)

    def _twist_agrees_with_pose(self, pos, v_body, stamp):
        """Is this message's reported speed consistent with how far it moved?"""
        if self._prev is None:
            return False
        p0, t0 = self._prev
        span = stamp - t0
        if span <= 0.0:
            return False
        observed = math.sqrt(sum((a - b) ** 2 for a, b in zip(pos, p0))) / span
        reported = math.sqrt(sum(c * c for c in v_body))
        return reported <= max(2.0 * observed, 1.0)

    # --- one control step -----------------------------------------------------
    def step(self, dt):
        """One control step: state -> TvcController -> HAL -> the three gz topics."""
        if self.state is None:
            return

        cmd = self.controller.update(self.state, self.setpoint, dt)
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
        # Always accumulated, not only when --log was given: the golden metrics
        # are computed from this, and a summary that only exists when someone
        # remembered a flag is a summary nobody checks.
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

    def unpause(self, world):
        """Start the world's physics, now that this controller is subscribed.

        THE CONTROLLER OWNS THE START. It used to be the shell script: launch
        Gazebo, `sleep 2`, unpause. That makes the number of uncontrolled
        physics steps a function of how fast the host got this process to its
        first publish, and it is not a small effect -- two consecutive 30 s runs
        peaked at 14.6 and 58.0 degrees of thrust-axis roll from the same
        nominal initial condition. The roll channel has 11.5x less inertia than
        the lateral pair and an actuator about 3x slower, so a couple of
        milliseconds of head start is worth 4x in the transient.

        Unpausing from here closes that: the first physics step of the run
        happens with the controller already listening.
        """
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.world_control_pb2 import WorldControl

        req = WorldControl()
        req.pause = False
        service = "/world/%s/control" % world
        ok, rep = self.node.request(service, req, WorldControl, Boolean, 5000)
        if not (ok and rep.data):
            print("could not unpause via %s -- is that the world's name?"
                  % service)
            return False
        print("unpaused %s from the controller" % world, flush=True)
        return True

    def metrics(self):
        """The summary the Gazebo golden is compared on.

        METRICS, NOT SAMPLES, and that is a measurement rather than a
        preference. With the controller owning the unpause the run is close to
        reproducible but not bit-identical: two consecutive 30 s runs differ by
        at most 1.5 deg of roll, 0.6 deg of gimbal and 12 mm of altitude at any
        one sample, because the first accepted odometry message can still land
        one or two physics steps apart. Sample-wise comparison against numbers
        that move by that much would fail on nothing. These aggregates do not
        move: every one of them agrees between those same two runs to better
        than the tolerance the golden file carries.
        """
        if not self.log:
            return {}
        col = list(zip(*self.log))
        t, z, x, y, pitch, yaw, roll = col[0], col[1], col[2], col[3], col[4], col[5], col[6]
        g_out, g_in, thrust = col[8], col[9], col[10]
        tilt = [math.hypot(pitch[i], yaw[i]) for i in range(len(t))]
        drift = [math.hypot(x[i], y[i]) for i in range(len(t))]
        # Settling: the last moment the vehicle was outside a 1 deg tilt band.
        # 1 deg, not a fraction of the upset, so the number stays comparable
        # when the world's spawn attitude changes.
        outside = [i for i, v in enumerate(tilt) if v > 1.0]
        return {
            "duration_s": t[-1],
            "samples": len(t),
            "final_altitude_m": z[-1],
            "final_drift_m": drift[-1],
            "final_tilt_deg": tilt[-1],
            "final_roll_deg": roll[-1],
            "peak_tilt_deg": max(tilt),
            "peak_roll_deg": max(abs(v) for v in roll),
            "peak_drift_m": max(drift),
            "min_altitude_m": min(z),
            "max_altitude_m": max(z),
            "peak_gimbal_outer_deg": max(abs(v) for v in g_out),
            "peak_gimbal_inner_deg": max(abs(v) for v in g_in),
            "min_thrust_N": min(thrust),
            "max_thrust_N": max(thrust),
            "tilt_settling_time_s": t[outside[-1]] if outside else 0.0,
            "rejected_odometry": self.rejected,
        }

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


# --- the Gazebo golden -------------------------------------------------------
# Tolerances, per metric, chosen from the measured run-to-run spread and then
# rounded up. They are NOT performance targets: a tolerance tight enough to
# catch ordinary retuning is a tolerance people learn to regenerate. Each of
# these is comfortably larger than the difference between two consecutive runs
# and comfortably smaller than the difference the old servo gains produced,
# which was the whole vehicle tumbling.
# harness/ -> tvc_control/ -> tvc_control/ -> src/ -> the repository root.
# Five levels, and the count is the kind of thing that is wrong by one until
# someone runs it, so tests/test_consistency.py asserts the file is there.
_REPO = os.path.abspath(__file__)
for _ in range(5):
    _REPO = os.path.dirname(_REPO)
GOLDEN_PATH = os.path.join(_REPO, "reference", "golden", "hover_baseline.json")

GOLDEN_TOLERANCE = {
    "final_altitude_m": 0.02,
    "final_drift_m": 0.02,
    "final_tilt_deg": 0.20,
    "final_roll_deg": 0.20,
    "peak_tilt_deg": 0.50,
    "peak_roll_deg": 5.00,      # the softest channel; see docs/7-CREDIBILITY.md
    "peak_drift_m": 0.05,
    "min_altitude_m": 0.05,
    "max_altitude_m": 0.05,
    "peak_gimbal_outer_deg": 1.00,
    "peak_gimbal_inner_deg": 1.00,
    "min_thrust_N": 0.50,
    "max_thrust_N": 0.50,
    "tilt_settling_time_s": 0.50,
}


def write_golden(metrics, args, path=GOLDEN_PATH):
    """Freeze this run as the Gazebo baseline."""
    doc = {
        "what": "Gazebo hover metrics. Compared on aggregates, not samples -- "
                "see GazeboHarness.metrics for why.",
        "run": {"world": args.unpause or "(already running)",
                "duration_s": args.duration, "altitude_m": args.altitude,
                "rate_hz": args.rate},
        "metrics": metrics,
        "tolerance": GOLDEN_TOLERANCE,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=2, sort_keys=True)
        f.write("\n")
    print("wrote %s" % path)


def check_golden(metrics, path=GOLDEN_PATH):
    """-> True if every metric is inside tolerance. Names each one that is not."""
    with open(path, encoding="utf-8") as f:
        doc = json.load(f)
    ref, tol = doc["metrics"], doc["tolerance"]
    bad = []
    for key, limit in sorted(tol.items()):
        got, want = metrics.get(key), ref.get(key)
        if got is None or want is None:
            bad.append("%-24s MISSING (golden %s, run %s)" % (key, want, got))
        elif abs(got - want) > limit:
            bad.append("%-24s %+.4f vs golden %+.4f  (drift %+.4f > %.4f)"
                       % (key, got, want, got - want, limit))
    if bad:
        print("\nGAZEBO GOLDEN DRIFTED:")
        for line in bad:
            print("  " + line)
        print("\nIf this was meant to change the flight, re-capture with "
              "`--capture-golden` in its own commit that says which numbers "
              "moved and why. Never re-capture to make a check pass.")
        return False
    print("gazebo golden matches (%d metrics within tolerance)" % len(tol))
    return True


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
                    help="nominal control rate [Hz]. Only a fallback and a clamp now: dt comes from the odometry stamps, so a mismatch with the publisher no longer silently rescales every derivative.")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--log", help="write per-sample flight data to this CSV")
    ap.add_argument("--capture-golden", action="store_true",
                    help="freeze this run's metrics as reference/golden/"
                         "hover_baseline.json")
    ap.add_argument("--check-golden", action="store_true",
                    help="compare this run's metrics against that baseline; "
                         "exit 2 and name every drifted metric")
    ap.add_argument("--unpause", metavar="WORLD", default=None,
                    help="start this world's physics once subscribed, so "
                         "the run begins deterministically (see "
                         "GazeboHarness.unpause). Omit when the world is "
                         "already running.")
    args = ap.parse_args(argv)

    h = GazeboHarness(args.altitude, rate_hz=args.rate,
                      verbose=not args.quiet, log_path=args.log)

    print("hovering to %.1f m for %.0f simulated seconds\n"
          % (args.altitude, args.duration), flush=True)
    # running BEFORE unpause: the first message off a paused world is the first
    # message of the run, so there is no window in which the vehicle flies
    # uncommanded and no dependence on how fast this process started.
    h.running = True
    if args.unpause:
        if not h.unpause(args.unpause):
            return 1
    else:
        print("waiting for odometry on %s ..." % ODOM_TOPIC, flush=True)
        for _ in range(200):
            if h.state is not None:
                break
            time.sleep(0.05)
        if h.state is None:
            print("no odometry. Start the world first:\n"
                  "  bash gazebo/run_hover.sh")
            return 1
    # The run ends on SIMULATED time, read from the odometry stamps, so a host
    # that cannot keep up produces a shorter wall-clock run and not a different
    # flight. The wall-clock deadline is a safety net so a stalled or paused
    # simulator cannot hang the script forever.
    deadline = time.time() + args.duration * 3.0 + 10.0
    while h._t_sim < args.duration and time.time() < deadline:
        time.sleep(0.05)
    h.running = False
    if h._t_sim < args.duration:
        print("WARNING: %.1f of %.1f simulated seconds ran before the wall-clock "
              "limit -- the simulation is far below real time."
              % (h._t_sim, args.duration))
    if h.rejected:
        print("rejected %d odometry message(s) whose twist contradicted the "
              "pose (see harness/gz.py::_on_odom)" % h.rejected)

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

    m = h.metrics()
    if args.capture_golden:
        write_golden(m, args)
    if args.check_golden and not check_golden(m):
        ok = False
    return 0 if ok else 2
