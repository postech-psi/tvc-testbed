"""
Standalone hover controller for the Gazebo TVC vehicle.

Talks to Gazebo directly over gz-transport (no ROS, no colcon build), so it
runs against a bare `gz sim`. The ROS2 path (controller_node + gazebo.launch.py)
is the "real" architecture; this exists to get a flying vehicle on screen
without building the whole workspace first.

WHAT IS ACTUALLY UNSTABLE HERE
------------------------------
Not attitude. With the gimbal centred, thrust lies along body +z applied at
r = (0,0,-L), so tau = r x F = 0 no matter how the body is oriented, and
gravity acts at the CG and makes no torque at all. Attitude is therefore
NEUTRALLY stable in free flight -- a double integrator, not an inverted
pendulum. (Believing thrust-below-CG self-stabilises, or that it diverges
like a pendulum, is the "pendulum rocket fallacy" both ways round; a pendulum
has a ground pivot for gravity to act about, and a flying vehicle does not.)

What diverges is POSITION: any tilt puts a horizontal component on the thrust
vector, which accelerates the vehicle sideways, which needs an opposite tilt
to arrest. So the position loop is deliberately much slower than the attitude
loop -- letting them approach each other in bandwidth is what makes these
vehicles wobble.

CONTROL MAPPING (derived, not guessed)
--------------------------------------
Joint order is roll(x) then pitch(y), rotors on the inner ring, so thrust in
the body frame is

    F_body = T * [ sin(dp), -sin(dr)*cos(dp), cos(dr)*cos(dp) ]

with the thrust point at r = (0, 0, -L) from the CG. Then tau = r x F gives

    tau_x = -L*T*sin(dr)*cos(dp)
    tau_y = -L*T*sin(dp)
    tau_z = 0                      <- the gimbal has NO yaw authority

Inverting for the commanded deflections:

    dp = -asin( tau_y / (L*T) )
    dr = -asin( tau_x / (L*T*cos(dp)) )

Yaw IS controlled, and has to be. Its only authority is a differential
between the counter-rotating rotors:

    tau_z = momentConstant * (T_a - T_b)

which is weak (momentConstant = 0.016 m, so 2 N of split buys 0.032 N.m) and
trades directly against total thrust. It cannot be skipped, though: with the
roll axis deflected by dr, the pitch servo's axis tilts out of the body
horizontal plane and leaks tau*sin(dr) of its reaction torque into yaw. With
nothing opposing it the vehicle spins up to several rad/s, and since roll and
pitch are BODY-frame, a steady world-frame tilt then reads as a large
oscillation that is really just the frame rotating underneath it.

Usage (with `gz sim -s -r sim/worlds/tvc.sdf` already running):
    python3 sim/hover.py --duration 25 --altitude 2.0
"""
import argparse
import math
import sys
import time

from gz.transport13 import Node
from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.double_pb2 import Double
from gz.msgs10.odometry_pb2 import Odometry

# --- vehicle constants: see docs/MASS_BUDGET.md ---
MASS_KG = 1.328
INERTIA_XY = 0.0226          # kg*m^2, roll/pitch
L_ARM = 0.211                # m, gimbal pivot -> CG
G = 9.81
WEIGHT_N = MASS_KG * G

MAX_ROT_VEL = 1100.0         # rad/s, matches the SDF
THRUST_AT_MAX = 20.0         # N combined, full throttle (2026-07-20 bench)
GIMBAL_MAX = math.radians(15.0)

# --- gains ---
# Attitude plant is I*theta_ddot = -L*T*sin(delta), a double integrator, so a
# PD cascade gives s^2 + KP_RATE*s + KP_RATE*KP_ANGLE: stable for any positive
# pair. 6/15 puts the closed loop at wn = 9.5 rad/s, zeta = 0.79 -- fast
# enough to hold tilt, damped enough not to ring.
# zeta = 1.05 (critically damped): the earlier 6/15 gave zeta 0.79, which the
# position loop could ring at its 9.5 rad/s natural frequency.
KP_ANGLE = 5.0               # rad/s of rate demand per rad of angle error
KP_RATE = 22.0               # rad/s^2 of angular accel per rad/s of rate error

KP_ALT = 4.0                 # m/s per m
KD_ALT = 3.0                 # N per m/s

# Position must be MUCH slower than attitude or the two fight: at Kp=0.22 /
# Kd=0.35 the loops were only 6.5x apart and the vehicle settled into a
# steady +/-9 deg coning limit cycle at the attitude natural frequency
# (roll and pitch 90 deg out of phase). 0.10/0.20 puts them ~10x apart.
KP_POS = 0.10                # rad of tilt per m of position error
KD_POS = 0.20                # rad per m/s
MAX_TILT = math.radians(8.0)

# Yaw: weak authority, so gains are modest and the split is capped. Holding
# yaw near zero matters more for keeping the body frame aligned with the
# world frame than for pointing.
KP_YAW = 2.0                 # rad/s of yaw rate demand per rad of yaw error
KD_YAW = 0.45                # N.m per rad/s
MOMENT_CONSTANT = 0.016      # m, matches the SDF
MAX_THRUST_SPLIT = 4.0       # N between the two rotors


def quat_to_euler(w, x, y, z):
    """ZYX yaw-pitch-roll. Returns (roll, pitch, yaw) in radians."""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    # asin domain guard: near +/-90 deg pitch, float error can push |sinp|>1
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class Hover:
    def __init__(self, target_alt, verbose=True, log_path=None):
        self.target_alt = target_alt
        self.verbose = verbose
        self.state = None
        self.t0 = None
        self.last_print = 0.0
        self.log = []
        self.log_path = log_path

        self.node = Node()
        self.pub_motor = self.node.advertise(
            "/tvc_vehicle/command/motor_speed", Actuators)
        self.pub_pitch = self.node.advertise("/tvc_vehicle/gimbal_pitch", Double)
        self.pub_roll = self.node.advertise("/tvc_vehicle/gimbal_roll", Double)

        if not self.node.subscribe(Odometry, "/model/tvc_vehicle/odometry",
                                   self.on_odom):
            raise RuntimeError("could not subscribe to odometry -- is gz sim running?")

    def on_odom(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        tw = msg.twist
        roll, pitch, yaw = quat_to_euler(q.w, q.x, q.y, q.z)
        self.state = {
            "x": p.x, "y": p.y, "z": p.z,
            "vx": tw.linear.x, "vy": tw.linear.y, "vz": tw.linear.z,
            "roll": roll, "pitch": pitch, "yaw": yaw,
            "wx": tw.angular.x, "wy": tw.angular.y, "wz": tw.angular.z,
        }

    def step(self):
        s = self.state
        if s is None:
            return

        # --- altitude -> total thrust ---
        vz_des = clamp(KP_ALT * (self.target_alt - s["z"]), -1.5, 2.0)
        thrust = WEIGHT_N + KD_ALT * (vz_des - s["vz"]) * MASS_KG
        thrust = clamp(thrust, 0.5, THRUST_AT_MAX)

        # --- position -> desired tilt ---
        # Rotating body +z into the world: +pitch tips thrust toward +x, and
        # +roll tips it toward -y. So to come BACK to the origin the demands
        # carry opposite signs to each other:
        #     x > 0  needs -x force -> negative pitch
        #     y > 0  needs -y force -> positive roll
        # Getting either backwards turns this loop into positive feedback and
        # the vehicle accelerates away instead of returning.
        pitch_des = clamp(-(KP_POS * s["x"] + KD_POS * s["vx"]),
                          -MAX_TILT, MAX_TILT)
        roll_des = clamp(+(KP_POS * s["y"] + KD_POS * s["vy"]),
                         -MAX_TILT, MAX_TILT)

        # --- attitude cascade: angle -> rate -> torque ---
        wx_des = KP_ANGLE * (roll_des - s["roll"])
        wy_des = KP_ANGLE * (pitch_des - s["pitch"])
        tau_x = INERTIA_XY * KP_RATE * (wx_des - s["wx"])
        tau_y = INERTIA_XY * KP_RATE * (wy_des - s["wy"])

        # --- torque -> gimbal deflection (inverse of the mapping above) ---
        lt = max(L_ARM * thrust, 1e-3)
        dp = -math.asin(clamp(tau_y / lt, -0.99, 0.99))
        dp = clamp(dp, -GIMBAL_MAX, GIMBAL_MAX)
        dr = -math.asin(clamp(tau_x / (lt * math.cos(dp)), -0.99, 0.99))
        dr = clamp(dr, -GIMBAL_MAX, GIMBAL_MAX)

        # --- yaw -> rotor thrust differential (the only yaw authority) ---
        wz_des = KP_YAW * (0.0 - s["yaw"])
        tau_z = KD_YAW * (wz_des - s["wz"])
        # NEGATIVE: rotor A spins CCW so its drag reaction on the body is -z,
        # and B's is +z. A positive yaw torque therefore needs MORE thrust on
        # B, i.e. a negative split. Getting this backwards turns every yaw
        # disturbance into positive feedback (measured: 0 -> -44 rad/s in 2 s).
        split = clamp(-tau_z / MOMENT_CONSTANT, -MAX_THRUST_SPLIT, MAX_THRUST_SPLIT)

        # Their sum still equals the commanded thrust, so yaw authority costs
        # nothing in altitude -- only in headroom.
        t_a = clamp(0.5 * (thrust + split), 0.0, THRUST_AT_MAX)
        t_b = clamp(0.5 * (thrust - split), 0.0, THRUST_AT_MAX)

        # --- thrust -> rotor speed (per rotor: thrust = k*omega^2) ---
        half_max = THRUST_AT_MAX / 2.0
        omega_a = MAX_ROT_VEL * math.sqrt(clamp(t_a / half_max, 0.0, 1.0))
        omega_b = MAX_ROT_VEL * math.sqrt(clamp(t_b / half_max, 0.0, 1.0))
        omega = 0.5 * (omega_a + omega_b)   # for the telemetry line

        act = Actuators()
        act.velocity.extend([omega_a, omega_b])
        self.pub_motor.publish(act)
        self.pub_pitch.publish(Double(data=dp))
        self.pub_roll.publish(Double(data=dr))

        if self.log_path is not None:
            if self.t0 is None:
                self.t0 = time.time()
            self.log.append((time.time() - self.t0, s["z"], s["x"], s["y"],
                             math.degrees(s["roll"]), math.degrees(s["pitch"]),
                             math.degrees(s["yaw"]), s["wz"],
                             math.degrees(dr), math.degrees(dp), thrust, split))

        if self.verbose:
            now = time.time()
            if now - self.last_print > 1.0:
                self.last_print = now
                print("z=%5.2f  xy=(%+5.2f,%+5.2f)  rp=(%+6.1f,%+6.1f)  "
                      "YAW=%+7.1f wz=%+5.2f  gimbal=(%+5.1f,%+5.1f)  T=%5.2f"
                      % (s["z"], s["x"], s["y"],
                         math.degrees(s["roll"]), math.degrees(s["pitch"]),
                         math.degrees(s["yaw"]), s["wz"],
                         math.degrees(dr), math.degrees(dp), thrust),
                      flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--altitude", type=float, default=2.0, help="target altitude, m")
    ap.add_argument("--duration", type=float, default=25.0, help="seconds to run")
    ap.add_argument("--rate", type=float, default=250.0, help="control loop Hz")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--log", help="write per-sample flight data to this CSV")
    args = ap.parse_args()

    h = Hover(args.altitude, verbose=not args.quiet, log_path=args.log)

    print("waiting for odometry...", flush=True)
    for _ in range(200):
        if h.state is not None:
            break
        time.sleep(0.05)
    if h.state is None:
        print("no odometry received -- is `gz sim -s -r sim/worlds/tvc.sdf` running?")
        return 1

    print("hovering to %.1f m for %.0f s\n" % (args.altitude, args.duration), flush=True)
    dt = 1.0 / args.rate
    end = time.time() + args.duration
    while time.time() < end:
        h.step()
        time.sleep(dt)

    s = h.state
    err = abs(s["z"] - args.altitude)
    tilt = math.degrees(math.hypot(s["roll"], s["pitch"]))
    drift = math.hypot(s["x"], s["y"])
    print("\nfinal: z=%.2f m (err %.2f)  tilt=%.1f deg  drift=%.2f m"
          % (s["z"], err, tilt, drift))
    if h.log_path and h.log:
        import csv as _csv
        with open(h.log_path, "w", newline="") as f:
            w = _csv.writer(f)
            w.writerow(["t_s", "z_m", "x_m", "y_m", "roll_deg", "pitch_deg",
                        "yaw_deg", "wz_rads", "gimbal_roll_deg",
                        "gimbal_pitch_deg", "thrust_N", "split_N"])
            w.writerows(h.log)
        print("wrote %s (%d samples)" % (h.log_path, len(h.log)))

    ok = err < 0.3 and tilt < 15 and drift < 1.0
    print("HOVER OK" if ok else "NOT STABLE -- see numbers above")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
