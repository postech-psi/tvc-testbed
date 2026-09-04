"""
6-DOF rigid-body dynamics. Simulation only -- this never flies.
================================================================================
Moved verbatim from physics.py.
"""

import numpy as np

from ..gnc.params import VehicleParams
from ..gnc.mathx import quat_normalize, quat_to_rotmat, quat_kinematics, thrust_axis


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

    n_hat = thrust_axis(delta)
    f_body = T * n_hat

    R = quat_to_rotmat(q)
    a_inertial = (R @ f_body) / params.m + np.array([0, 0, -params.g])

    # AXIAL lever arm (gimbal pivot -> CM along body z), plus optional lateral
    # misalignment disturbance terms dx, dy (normally ~0 for a balanced vehicle).
    d_cm = np.array([params.dx, params.dy, -params.L])
    # tau_P acts along the PROP AXIS, not along body z: the props are mounted on
    # the gimbal, so their reaction torque tilts with the thrust. That is what
    # couples the axial channel into the lateral axes -- expanding gives
    #     M_x = -T*L*sin(d2)cos(d1) + tau_P*sin(d1)
    #     M_y = -T*L*sin(d1)        - tau_P*sin(d2)cos(d1)
    #     M_z =                       tau_P*cos(d1)cos(d2)
    # Dropping the tau_P terms (as this module did before) is exact only while
    # the axial channel is unused; it under-predicts lateral torque by ~6.5% of
    # full authority once tau_P is commanded.
    tau = np.cross(d_cm, f_body) + tau_p * n_hat

    I = np.diag([params.Ix, params.Iy, params.Iz])
    I_inv = np.diag([1/params.Ix, 1/params.Iy, 1/params.Iz])
    omega_dot = I_inv @ (tau - np.cross(omega, I @ omega))

    q_dot = quat_kinematics(q, omega)

    return np.concatenate([v, a_inertial, q_dot, omega_dot])
