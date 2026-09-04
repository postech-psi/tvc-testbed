"""Phase 6: adopt the rocket axis convention. One atomic, number-preserving pass.

The mapping is a 3-CYCLE, not a swap: body x was "roll" and becomes "pitch",
body y was "pitch" and becomes "yaw", body z was "axial"/"yaw" and becomes
"roll". Applying that naively in one pass turns roll->pitch and then that same
pitch->yaw, collapsing two axes into one. So every rename goes through a
sentinel first.

Matching uses explicit whole IDENTIFIERS with non-word guards, never a substring
pass: `controller` contains `roll`, and about half the raw hits in this repo are
that false positive.
"""
import io
import os
import re

# --- (old, new) in identifier space. Order does not matter: every replacement
# --- goes old -> sentinel in pass 1, sentinel -> new in pass 2.
PAIRS = [
    # ---- external: SDF joints and gz/ROS topics ------------------------------
    # Named after the RING, not the axis: a ring is a mechanical fact invariant
    # under any convention, so a future revision does not touch model.sdf. It
    # also guarantees no old string survives with a new meaning.
    ("gimbal_roll_joint", "gimbal_outer_joint"),
    ("gimbal_pitch_joint", "gimbal_inner_joint"),
    ("/tvc_vehicle/gimbal_roll", "/tvc_vehicle/gimbal_outer_cmd"),
    ("/tvc_vehicle/gimbal_pitch", "/tvc_vehicle/gimbal_inner_cmd"),

    # ---- message fields ------------------------------------------------------
    ("gimbal_delta1_rad", "gimbal_inner_rad"),
    ("gimbal_delta2_rad", "gimbal_outer_rad"),

    # ---- ROS parameters: every axis-bearing name is NEW, none is reused ------
    ("roll_des_deg", "att_pitch_des_deg"),
    ("pitch_des_deg", "att_yaw_des_deg"),
    ("axial_des_deg", "att_roll_des_deg"),
    ("init_roll_deg", "init_att_pitch_deg"),
    ("init_pitch_deg", "init_att_yaw_deg"),
    ("init_axial_deg", "init_att_roll_deg"),

    # ---- metrics keys --------------------------------------------------------
    ("final_roll_deg", "final_pitch_deg"),
    ("final_pitch_deg", "final_yaw_deg"),
    ("final_axial_deg", "final_roll_deg"),
    ("roll_error_deg", "pitch_error_deg"),
    ("pitch_error_deg", "yaw_error_deg"),
    ("axial_error_deg", "roll_error_deg"),
    ("max_delta1_deg", "max_gimbal_inner_deg"),
    ("max_delta2_deg", "max_gimbal_outer_deg"),
    ("axial_sat_frac", "roll_sat_frac"),
    ("axial_headroom_Nm", "roll_headroom_Nm"),
    ("axial_limit_lo_Nm", "roll_limit_lo_Nm"),
    ("axial_limit_hi_Nm", "roll_limit_hi_Nm"),
    ("axial_limits_vs_thrust", "roll_limits_vs_thrust"),

    # ---- CSV columns (hover.py writes, plot_flight.py reads by name) ---------
    ("roll_deg", "pitch_deg"),
    ("pitch_deg", "yaw_deg"),
    ("yaw_deg", "roll_deg"),
    ("gimbal_roll_deg", "gimbal_outer_deg"),
    ("gimbal_pitch_deg", "gimbal_inner_deg"),

    # ---- flight code / plant / harness identifiers ---------------------------
    ("roll_des", "pitch_des"),
    ("pitch_des", "yaw_des"),
    ("axial_des", "roll_des"),
    ("roll_rate_des", "pitch_rate_des"),
    ("pitch_rate_des", "yaw_rate_des"),
    ("axial_rate_des", "roll_rate_des"),
    ("pid_roll_angle", "pid_pitch_angle"),
    ("pid_pitch_angle", "pid_yaw_angle"),
    ("pid_axial_angle", "pid_roll_angle"),
    ("pid_roll_rate", "pid_pitch_rate"),
    ("pid_pitch_rate", "pid_yaw_rate"),
    ("pid_axial_rate", "pid_roll_rate"),
    ("kp_angle_axial", "kp_angle_roll"),
    ("kp_rate_axial", "kp_rate_roll"),
    ("ki_rate_axial", "ki_rate_roll"),
    ("kd_rate_axial", "kd_rate_roll"),
    ("i_limit_axial", "i_limit_roll"),
    ("axial_headroom", "roll_headroom"),
    ("axial_limits", "roll_limits"),
    ("axial_saturated", "roll_saturated"),
    ("sat_axial", "sat_roll"),
    ("scenario_axial", "scenario_roll"),
    ("final_roll", "final_pitch"),
    ("final_pitch", "final_yaw"),
    ("roll_err", "pitch_err"),
    ("pitch_err", "yaw_err"),

    # ---- bare axis words (locals, dict keys, prose) --------------------------
    # The full 3-cycle. Omitting yaw->roll on the first attempt produced
    # `euler_to_quat(pitch, yaw, yaw)` -- a duplicate-argument SyntaxError that
    # the test suite caught immediately, which is the entire reason the suite
    # was written before this commit rather than after it.
    ("roll", "pitch"),
    ("pitch", "yaw"),
    ("yaw", "roll"),
    ("axial", "roll"),        # synonym for the thrust axis; same destination
    ("Roll", "Pitch"),
    ("Pitch", "Yaw"),
    ("Yaw", "Roll"),
    ("Axial", "Roll"),
    ("AXIAL", "ROLL"),
    ("YAW", "ROLL"),

    # ---- sim/hover.py locals -------------------------------------------------
    ("pub_roll", "pub_outer"),
    ("pub_pitch", "pub_inner"),
    ("dp", "d_inner"),
    ("dr", "d_outer"),
    ("tau_z", "tau_roll"),

    # ---- the convention token itself ----------------------------------------
    ("legacy_quadcopter", "rocket_v2"),
    ("AXIS_CONVENTION_LEGACY_QUADCOPTER", "AXIS_CONVENTION_ROCKET_V2"),
]

# Words that CONTAIN an axis word but must never be touched. Guarded by the
# identifier-boundary regex already, but listed so a future maintainer can see
# what the guard is protecting.
DENY = ("controller", "Controller", "controllers", "controllable",
        "uncontrollable", "JointPositionController", "microcontroller",
        "scrollbar", "yscrollcommand", "scrollregion", "yview_scroll",
        "rollingMomentCoefficient", "rolling_moment_coefficient",
        "coaxial", "Coaxial", "controlled")

# tests/ and docs/ are EXCLUDED and hand-edited. docs/CONVENTIONS.md and
# docs/CREDIBILITY.md are already written in the target convention -- running
# the cycle over them would rotate correct text into wrong text. The same is
# true of tests/test_axis_convention.py, which deliberately names both sides.
TARGET_DIRS = ("src", "sim", "tools")
TARGET_FILES = ("tvc_gui.py", "tvc_view3d.py", "tvc_showcase.ino")
EXTS = (".py", ".msg", ".yaml", ".ino", ".md", ".txt", ".csv")
SKIP = ("__pycache__", "build", "install", "log", ".git",
        "model.sdf",            # generated; regenerated after this runs
        "analytic_baseline.json", "validate_baseline.txt")  # goldens


def files():
    out = []
    for d in TARGET_DIRS:
        for root, dirs, names in os.walk(d):
            dirs[:] = [x for x in dirs if x not in SKIP]
            for n in names:
                if n in SKIP:
                    continue
                if n.endswith(EXTS):
                    out.append(os.path.join(root, n))
    for f in TARGET_FILES:
        if os.path.isfile(f):
            out.append(f)
    return sorted(set(out))


def apply(text):
    # Pass 1: old -> unique sentinel. Sentinels contain characters that cannot
    # appear in an identifier, so pass 2 cannot re-match a pass-1 result.
    for i, (old, _) in enumerate(PAIRS):
        text = re.sub(r'(?<![A-Za-z0-9_])%s(?![A-Za-z0-9_])' % re.escape(old),
                      '\x00%d\x00' % i, text)
    # Pass 2: sentinel -> new.
    for i, (_, new) in enumerate(PAIRS):
        text = text.replace('\x00%d\x00' % i, new)
    return text


def main():
    changed = 0
    for path in files():
        src = io.open(path, encoding='utf-8').read()
        out = apply(src)
        if out != src:
            io.open(path, 'w', encoding='utf-8', newline='\n').write(out)
            changed += 1
            print('  %s' % path)
    print('%d files changed' % changed)

    # The guard, run over the result: any surviving bare axis word must be a
    # known false positive.
    print('\nresidual bare-token scan:')
    pat = re.compile(r'(?<![A-Za-z0-9_])(axial|Axial|delta1|delta2)(?![A-Za-z0-9_])')
    hits = 0
    for path in files():
        for n, line in enumerate(io.open(path, encoding='utf-8'), 1):
            if pat.search(line) and not any(d in line for d in DENY):
                print('  %s:%d %s' % (path, n, line.strip()[:100]))
                hits += 1
    print('  %d residual hits' % hits)


if __name__ == '__main__':
    main()
