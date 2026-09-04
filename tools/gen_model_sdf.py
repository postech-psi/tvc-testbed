"""
gen_model_sdf.py -- regenerate sim/models/tvc_vehicle/model.sdf from the single
source of truth (tvc_control/vehicle_params.yaml).
================================================================================
WHY THIS EXISTS
The hand-written SDF used to split the vehicle mass across six links whose
placement made the Gazebo *composite* CG land at 161 mm while the mass tool and
the controller both assumed 211 mm -- so Gazebo silently simulated a different
vehicle than hover.py was tuned for. This generator removes that failure mode:
it solves for base_link's mass / pose / inertia so the composite of ALL links
reproduces the tool's (mass, CG, inertia) EXACTLY, by construction, every time.

MODEL CHOICE (locked with the user): "single rigid base_link". base_link is the
whole vehicle; the gimbal rings and rotors carry only small nominal token masses
needed for articulation and the multicopter motor model. Those token masses are
SUBTRACTED from base_link (not added on top), so total mass stays exact and the
tilting-mass / rotor-gyroscopic effects are deliberately minimized at this stage
-- what matters here is the thrust-authority-to-inertia ratio and the gimbal
geometry, both of which are now exact.

Run:
    python tools/gen_model_sdf.py                 # writes the model.sdf in place
    python tools/gen_model_sdf.py --check         # verify composite == target, no write
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SIM = os.path.join(REPO, "sim")
sys.path.insert(0, SIM)
import vehicle_params as vp  # noqa: E402

OUT_PATH = os.path.join(SIM, "models", "tvc_vehicle", "model.sdf")

# --- nominal token links (small; subtracted from base_link) ------------------
# name, mass_kg, z_m, (ixx, iyy, izz).  x=y=0 for all of them (on the axis).
# Kept small on purpose (see module docstring). Rotors keep enough spin inertia
# (izz) for the motor model to behave; the rest is nominal.
TOKEN_LINKS = [
    ("legs_link",    0.030, -0.060, (3.0e-4, 3.0e-4, 5.0e-4)),
    ("outer_gimbal", 0.010,  0.000, (1.0e-5, 1.0e-5, 1.5e-5)),
    ("inner_gimbal", 0.010,  0.000, (1.2e-5, 1.2e-5, 1.4e-5)),
    ("rotor_a",      0.010,  0.030, (9.0e-6, 9.0e-6, 1.7e-5)),
    ("rotor_b",      0.010,  0.060, (9.0e-6, 9.0e-6, 1.7e-5)),
]


def _parallel(m, r):
    """Parallel-axis contribution m*((r.r)I - r x r) for a point/body at offset r."""
    r = np.asarray(r, float)
    return m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))


def solve_base_link(v):
    """Solve base_link (mass, position, inertia-about-itself) so the composite of
    base_link + TOKEN_LINKS equals the target (v.mass, v.cg, full I tensor)."""
    C = np.array(v.cg, float)
    I_full = np.array([
        [v.Ix,  v.Ixy, v.Ixz],
        [v.Ixy, v.Iy,  v.Iyz],
        [v.Ixz, v.Iyz, v.Iz],
    ])

    tok_mass = sum(t[1] for t in TOKEN_LINKS)
    base_mass = v.mass - tok_mass
    if base_mass <= 0:
        raise ValueError("token masses (%.3f) exceed total (%.3f)"
                         % (tok_mass, v.mass))

    # First moment: M*C = base_mass*base_pos + sum(tok_mass*tok_pos)
    tok_moment = sum(t[1] * np.array([0.0, 0.0, t[2]]) for t in TOKEN_LINKS)
    base_pos = (v.mass * C - tok_moment) / base_mass

    # Inertia about the true CG: I_full = I_base_own + base parallel term
    #   + sum(tok_own + tok parallel term).  Solve for I_base_own.
    I_base_own = I_full.copy()
    I_base_own -= _parallel(base_mass, base_pos - C)
    for _name, m, z, (ixx, iyy, izz) in TOKEN_LINKS:
        I_base_own -= np.diag([ixx, iyy, izz])
        I_base_own -= _parallel(m, np.array([0.0, 0.0, z]) - C)
    return base_mass, base_pos, I_base_own


def composite_check(v, base_mass, base_pos, I_base_own):
    """Recompose everything and return (mass, cg, I) as Gazebo would see it."""
    links = [(base_mass, base_pos, I_base_own)]
    for _name, m, z, (ixx, iyy, izz) in TOKEN_LINKS:
        links.append((m, np.array([0.0, 0.0, z]), np.diag([ixx, iyy, izz])))
    M = sum(m for m, _, _ in links)
    C = sum(m * p for m, p, _ in links) / M
    I = np.zeros((3, 3))
    for m, p, Iown in links:
        I += Iown + _parallel(m, p - C)
    return M, C, I


def render(v, base_mass, base_pos, I_base_own):
    r = v.raw
    rot, gim = r["rotors"], v
    gmax = np.deg2rad(v.gimbal_max_deg)
    # The joint velocity limit is deliberately far above the measured slew: the
    # plant model owns the rate limit (and does it per ring). Leaving the SDF's
    # own limit at the measured value would apply it twice.
    grate = np.deg2rad(v.gimbal_rate_max_deg) * 10.0
    mtc = (r.get("motor_dynamics") or {}).get("gazebo_time_constant_s", 0.001)
    bx, by, bz = base_pos
    Ib = I_base_own

    def tok(name):
        return next(t for t in TOKEN_LINKS if t[0] == name)

    def inertial(mass, pose_z, I):
        if np.ndim(I) == 1:
            ixx, iyy, izz = I
            ixy = ixz = iyz = 0.0
        else:
            ixx, iyy, izz = I[0, 0], I[1, 1], I[2, 2]
            ixy, ixz, iyz = I[0, 1], I[0, 2], I[1, 2]
        return (
            "      <pose>0 0 %.4f 0 0 0</pose>\n"
            "      <inertial>\n"
            "        <mass>%.4f</mass>\n"
            "        <inertia>\n"
            "          <ixx>%.6e</ixx><iyy>%.6e</iyy><izz>%.6e</izz>\n"
            "          <ixy>%.6e</ixy><ixz>%.6e</ixz><iyz>%.6e</iyz>\n"
            "        </inertia>\n"
            "      </inertial>" % (pose_z, mass, ixx, iyy, izz, ixy, ixz, iyz))

    lg = tok("legs_link"); og = tok("outer_gimbal"); ig = tok("inner_gimbal")
    ra = tok("rotor_a"); rb = tok("rotor_b")

    def motor_plugin(joint, link, turn, num):
        return f"""    <plugin filename="gz-sim-multicopter-motor-model-system"
            name="gz::sim::systems::MulticopterMotorModel">
      <robotNamespace>tvc_vehicle</robotNamespace>
      <jointName>{joint}</jointName>
      <linkName>{link}</linkName>
      <turningDirection>{turn}</turningDirection>
      <!-- Near-zero on purpose. The motor lag is owned by the plant model
           (tvc_control/plant/actuators.py::MotorLag), in ONE place, so both
           plants agree. It cannot live here anyway: this filters the rotor
           VELOCITY reference, and since T ~ omega^2 a first-order lag in omega
           is not a first-order lag in thrust -- it is faster near hover and
           asymmetric between spin-up and spin-down. -->
      <timeConstantUp>{mtc:.4f}</timeConstantUp>
      <timeConstantDown>{mtc:.4f}</timeConstantDown>
      <maxRotVelocity>{rot['max_rot_velocity']:.1f}</maxRotVelocity>
      <motorConstant>{rot['motor_constant']:.3e}</motorConstant>
      <momentConstant>{rot['moment_constant']:.4g}</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>{num}</motorNumber>
      <rotorDragCoefficient>{rot['rotor_drag_coefficient']:.3e}</rotorDragCoefficient>
      <rollingMomentCoefficient>{rot['rolling_moment_coefficient']:.3e}</rollingMomentCoefficient>
      <motorType>velocity</motorType>
    </plugin>"""

    def servo_plugin(joint, topic):
        return f"""    <plugin filename="gz-sim-joint-position-controller-system"
            name="gz::sim::systems::JointPositionController">
      <joint_name>{joint}</joint_name>
      <topic>{topic}</topic>
      <!-- NEUTRALISED. These gains plus the joint velocity limit used to give
           Gazebo its own gimbal lag and slew, on top of the plant model's --
           double-counting, and with a single symmetric rate where the bench
           measured 403 (inner) and 235 (outer) deg/s. The actuator dynamics now
           have one owner (plant/actuators.py::GimbalActuator), so this
           controller is asked only to track its command as fast as it can. -->
      <p_gain>60.0</p_gain>
      <i_gain>0.0</i_gain>
      <d_gain>1.0</d_gain>
      <cmd_max>5.0</cmd_max>
      <cmd_min>-5.0</cmd_min>
    </plugin>"""

    return f"""<?xml version="1.0"?>
<!--
  GENERATED FILE. Do not edit by hand.
  Regenerate with:  python tools/gen_model_sdf.py
  Source of truth:  tvc_control/vehicle_params.yaml

  Coaxial TVC VTVL vehicle for Gazebo Harmonic (gz-sim 8). base_link carries the
  WHOLE-vehicle mass properties solved so the composite of all links reproduces
  the mass tool's (mass, CG, inertia) exactly: mass {v.mass:.4f} kg, CG z={v.cg[2]*1000:.1f} mm,
  Ix={v.Ix:.6f}. The gimbal/rotor links are small nominal tokens (see
  tools/gen_model_sdf.py TOKEN_LINKS) subtracted from base_link, so this model no
  longer silently drifts from the controller's assumptions.

  Body +z is the thrust axis. rotor_a spins CCW, rotor_b CW: their reaction
  torques oppose, so a differential between them is the ONLY roll authority (the
  2-axis gimbal has none about z).
-->
<sdf version="1.9">
  <model name="tvc_vehicle">
    <pose>0 0 0 0 0 0</pose>

    <!-- ================= body (whole-vehicle mass properties) ============= -->
    <link name="base_link">
{inertial(base_mass, bz, Ib)}

      <collision name="body_collision">
        <pose>0 0 {-bz:.4f} 0 0 0</pose>
        <geometry>
          <cylinder><radius>0.045</radius><length>0.50</length></cylinder>
        </geometry>
      </collision>
      <visual name="body_visual">
        <pose>0 0 {-bz:.4f} 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://tvc_vehicle/meshes/tvc_vehicle.stl</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.25 0.27 0.32 1</ambient>
          <diffuse>0.45 0.5 0.58 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
        </material>
      </visual>

      <sensor name="imu_sensor" type="imu">
        <always_on>1</always_on>
        <update_rate>250</update_rate>
        <topic>/tvc_vehicle/imu</topic>
      </sensor>
    </link>

    <!-- Legs: nominal mass, real collision geometry for touchdown. -->
    <link name="legs_link">
      <pose>0 0 {lg[2]:.4f} 0 0 0</pose>
      <inertial>
        <mass>{lg[1]:.4f}</mass>
        <inertia>
          <ixx>{lg[3][0]:.6e}</ixx><iyy>{lg[3][1]:.6e}</iyy><izz>{lg[3][2]:.6e}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="leg_a">
        <pose>0.14 0 -0.02 0 0.6 0</pose>
        <geometry><cylinder><radius>0.008</radius><length>0.22</length></cylinder></geometry>
      </collision>
      <collision name="leg_b">
        <pose>-0.07 0.121 -0.02 0 0.6 2.094</pose>
        <geometry><cylinder><radius>0.008</radius><length>0.22</length></cylinder></geometry>
      </collision>
      <collision name="leg_c">
        <pose>-0.07 -0.121 -0.02 0 0.6 -2.094</pose>
        <geometry><cylinder><radius>0.008</radius><length>0.22</length></cylinder></geometry>
      </collision>
    </link>
    <joint name="legs_joint" type="fixed">
      <parent>base_link</parent>
      <child>legs_link</child>
    </joint>

    <!-- ================= gimbal ================= -->
    <link name="outer_gimbal">
      <pose>0 0 {og[2]:.4f} 0 0 0</pose>
      <inertial>
        <mass>{og[1]:.4f}</mass>
        <inertia>
          <ixx>{og[3][0]:.6e}</ixx><iyy>{og[3][1]:.6e}</iyy><izz>{og[3][2]:.6e}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
    </link>
    <joint name="gimbal_outer_joint" type="revolute">
      <parent>base_link</parent>
      <child>outer_gimbal</child>
      <axis>
        <xyz>1 0 0</xyz>
        <limit>
          <lower>{-gmax:.4f}</lower>
          <upper>{gmax:.4f}</upper>
          <effort>5.0</effort>
          <velocity>{grate:.4f}</velocity>
        </limit>
        <dynamics><damping>0.01</damping></dynamics>
      </axis>
    </joint>

    <link name="inner_gimbal">
      <pose>0 0 {ig[2]:.4f} 0 0 0</pose>
      <inertial>
        <mass>{ig[1]:.4f}</mass>
        <inertia>
          <ixx>{ig[3][0]:.6e}</ixx><iyy>{ig[3][1]:.6e}</iyy><izz>{ig[3][2]:.6e}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
    </link>
    <joint name="gimbal_inner_joint" type="revolute">
      <parent>outer_gimbal</parent>
      <child>inner_gimbal</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>{-gmax:.4f}</lower>
          <upper>{gmax:.4f}</upper>
          <effort>5.0</effort>
          <velocity>{grate:.4f}</velocity>
        </limit>
        <dynamics><damping>0.01</damping></dynamics>
      </axis>
    </joint>

    <!-- ================= coax rotors ================= -->
    <link name="rotor_a">
      <pose>0 0 {ra[2]:.4f} 0 0 0</pose>
      <inertial>
        <mass>{ra[1]:.4f}</mass>
        <inertia>
          <ixx>{ra[3][0]:.6e}</ixx><iyy>{ra[3][1]:.6e}</iyy><izz>{ra[3][2]:.6e}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="rotor_a_v">
        <geometry><cylinder><radius>0.102</radius><length>0.004</length></cylinder></geometry>
        <material><diffuse>0.1 0.6 0.9 0.6</diffuse></material>
      </visual>
    </link>
    <joint name="rotor_a_joint" type="revolute">
      <parent>inner_gimbal</parent>
      <child>rotor_a</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>0.004</damping></dynamics>
      </axis>
    </joint>

    <link name="rotor_b">
      <pose>0 0 {rb[2]:.4f} 0 0 0</pose>
      <inertial>
        <mass>{rb[1]:.4f}</mass>
        <inertia>
          <ixx>{rb[3][0]:.6e}</ixx><iyy>{rb[3][1]:.6e}</iyy><izz>{rb[3][2]:.6e}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="rotor_b_v">
        <geometry><cylinder><radius>0.102</radius><length>0.004</length></cylinder></geometry>
        <material><diffuse>0.9 0.3 0.1 0.6</diffuse></material>
      </visual>
    </link>
    <joint name="rotor_b_joint" type="revolute">
      <parent>inner_gimbal</parent>
      <child>rotor_b</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>0.004</damping></dynamics>
      </axis>
    </joint>

    <!-- ================= plugins (constants from vehicle_params.yaml) ===== -->
{motor_plugin('rotor_a_joint', 'rotor_a', 'ccw', 0)}

{motor_plugin('rotor_b_joint', 'rotor_b', 'cw', 1)}

{servo_plugin('gimbal_inner_joint', '/tvc_vehicle/gimbal_inner_cmd')}
{servo_plugin('gimbal_outer_joint', '/tvc_vehicle/gimbal_outer_cmd')}

    <plugin filename="gz-sim-odometry-publisher-system"
            name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <odom_publish_frequency>250</odom_publish_frequency>
      <!-- MUST be 3: the default 2D mode drops z/vz and the altitude loop then
           reads z=0 forever and commands full thrust into the ground. -->
      <dimensions>3</dimensions>
    </plugin>
  </model>
</sdf>
"""


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    check_only = "--check" in argv

    v = vp.load()
    base_mass, base_pos, I_base_own = solve_base_link(v)
    M, C, I = composite_check(v, base_mass, base_pos, I_base_own)

    print("Target   : m=%.4f  CG=(%.1f,%.1f,%.1f)mm  Ix=%.6f Iy=%.6f Iz=%.6f"
          % (v.mass, v.cg[0]*1000, v.cg[1]*1000, v.cg[2]*1000, v.Ix, v.Iy, v.Iz))
    print("Composite: m=%.4f  CG=(%.1f,%.1f,%.1f)mm  Ix=%.6f Iy=%.6f Iz=%.6f"
          % (M, C[0]*1000, C[1]*1000, C[2]*1000, I[0, 0], I[1, 1], I[2, 2]))

    cg_err = np.linalg.norm(C - np.array(v.cg)) * 1000
    I_err = abs(I[0, 0] - v.Ix)
    ok = cg_err < 1e-3 and I_err < 1e-6
    print("Match    : CG err %.2e mm, Ix err %.2e  -> %s"
          % (cg_err, I_err, "OK" if ok else "MISMATCH"))
    if not ok:
        return 1

    if check_only:
        print("(--check: not writing)")
        return 0

    with open(OUT_PATH, "w", encoding="utf-8") as f:
        f.write(render(v, base_mass, base_pos, I_base_own))
    print("Wrote %s" % OUT_PATH)
    return 0


if __name__ == "__main__":
    sys.exit(main())
