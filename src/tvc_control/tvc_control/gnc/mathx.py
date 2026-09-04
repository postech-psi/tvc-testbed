"""
Small fixed-size math: quaternions and the gimballed thrust direction.
================================================================================
Shared by the controller and the plant. Math is neither -- putting it in `gnc`
rather than duplicating it keeps one definition of the quaternion convention,
which is the kind of thing that silently forks and then disagrees by a sign.

Convention: quaternions are (qw, qx, qy, qz), body <- inertial, unit norm.
Euler angles are READOUT ONLY -- kinematics are never propagated through them,
which is what keeps the +/-90 deg singularity out of the state.
"""

import numpy as np


def quat_normalize(q):
    """Renormalize to unit quaternion (mandatory after every integration step;
    truncation error accumulates otherwise -- see Step 1 guide, Prop. on renorm)."""
    return q / np.linalg.norm(q)


def quat_to_rotmat(q):
    """Body -> inertial rotation matrix R(q), from unit quaternion q=(qw,qx,qy,qz)."""
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),     1 - 2*(qx**2 + qy**2)],
    ])


def quat_kinematics(q, omega):
    """dq/dt = 0.5 * Omega(omega) @ q  -- linear, singularity-free kinematics."""
    wx, wy, wz = omega
    Omega = np.array([
        [0, -wx, -wy, -wz],
        [wx,  0,  wz, -wy],
        [wy, -wz,  0,  wx],
        [wz,  wy, -wx,  0],
    ])
    return 0.5 * Omega @ q


def quat_to_euler(q):
    """ZYX (yaw-pitch-roll) Euler angles, for readout/plotting ONLY -- never
    used internally for kinematics propagation (that would reintroduce gimbal lock)."""
    qw, qx, qy, qz = q
    roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
    pitch = np.arcsin(np.clip(2*(qw*qy - qz*qx), -1, 1))
    yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
    return np.array([roll, pitch, yaw])


def euler_to_quat(roll, pitch, yaw):
    """Construct a quaternion from ZYX Euler angles (used only to set up initial
    conditions / disturbances in a human-friendly way)."""
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return np.array([qw, qx, qy, qz])


def thrust_axis(delta):
    """Unit vector n_hat along the gimballed thrust axis, in the body frame.

    Exact (not small-angle). Sign convention matches Gazebo / sim/hover.py --
    n_hat = R_x(d2) R_y(d1) [0,0,1] -- so gimbal signs and gains are portable
    between the two sims. d1 deflects in the pitch plane (about body y),
    d2 in the roll plane (about body x).
    """
    d1, d2 = delta
    return np.array([
        np.sin(d1),
        -np.sin(d2) * np.cos(d1),
        np.cos(d1) * np.cos(d2),
    ])
