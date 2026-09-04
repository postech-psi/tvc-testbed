"""
6-DOF rigid-body dynamics. Simulation only -- this never flies.
================================================================================
Moved verbatim from physics.py.
"""

import numpy as np

from ..gnc.params import VehicleParams
from ..gnc.mathx import quat_normalize, quat_to_rotmat, thrust_axis


def quat_kinematics(q, omega):
    """dq/dt = 0.5 * Omega(omega) @ q  -- linear, singularity-free kinematics.

    Lives with the plant, not with the flight-code math: only an integrator
    needs it, and keeping it here is what lets gnc/mathx.py stay free of numpy.
    """
    wx, wy, wz = omega
    Omega = np.array([
        [0, -wx, -wy, -wz],
        [wx,  0,  wz, -wy],
        [wy, -wz,  0,  wx],
        [wz,  wy, -wx,  0],
    ])
    return 0.5 * Omega @ np.asarray(q, dtype=float)



def dynamics(t, x, T, delta, params: VehicleParams, tau_p=0.0):
    """
    x = [r(3), v(3), q(4), omega(3)]  (13 states)
    T:      scalar thrust magnitude [N]
    delta:  [delta1, delta2] gimbal deflection angles [rad]
    tau_p:  net propeller reaction torque about the THRUST AXIS [N*m], from the
            deliberate imbalance between the two counter-rotating coax props.
            Defaults to 0 so every existing caller integrates exactly as before.
    """
    v = x[3:6]
    q = quat_normalize(x[6:10])
    omega = x[10:13]

    # gnc returns plain tuples; the integrator wants arrays.
    n_hat = np.asarray(thrust_axis(delta), dtype=float)
    f_body = T * n_hat

    R = np.asarray(quat_to_rotmat(q), dtype=float)
    a_inertial = (R @ f_body) / params.m + np.array([0, 0, -params.g])

    # ROLL lever arm (gimbal pivot -> CM along body z), plus optional lateral
    # misalignment disturbance terms dx, dy (normally ~0 for a balanced vehicle).
    d_cm = np.array([params.dx, params.dy, -params.L])
    # tau_P acts along the PROP AXIS, not along body z: the props are mounted on
    # the gimbal, so their reaction torque tilts with the thrust. That is what
    # couples the roll channel into the lateral axes -- expanding gives
    #     M_x = -T*L*sin(d2)cos(d1) + tau_P*sin(d1)
    #     M_y = -T*L*sin(d1)        - tau_P*sin(d2)cos(d1)
    #     M_z =                       tau_P*cos(d1)cos(d2)
    # Dropping the tau_P terms (as this module did before) is exact only while
    # the roll channel is unused; it under-predicts lateral torque by ~6.5% of
    # full authority once tau_P is commanded.
    tau = np.cross(d_cm, f_body) + tau_p * n_hat

    # FULL inertia tensor, not the diagonal. The products of inertia are not
    # small on this airframe -- Iyz/Izz = 27.3%, Ixz/Izz = 15.4% -- so the
    # diagonal approximation dominates the thrust-axis response and would make
    # any comparison against Gazebo (which carries the full tensor in the SDF)
    # disagree for a reason that has nothing to do with the control model.
    # Off-diagonal sign convention follows the SDF's: I = [[ixx, ixy, ixz],
    # [ixy, iyy, iyz], [ixz, iyz, izz]], which is what gen_model_sdf.py emits
    # from the same YAML fields.
    I = np.array([
        [params.Ix,  params.Ixy, params.Ixz],
        [params.Ixy, params.Iy,  params.Iyz],
        [params.Ixz, params.Iyz, params.Iz],
    ])
    omega_dot = np.linalg.solve(I, tau - np.cross(omega, I @ omega))

    q_dot = quat_kinematics(q, omega)

    return np.concatenate([v, a_inertial, q_dot, omega_dot])
