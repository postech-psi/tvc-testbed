# reference/

Material that is **not part of the simulator** but has to live somewhere, and
whose somewhere should be obvious.

```
reference/
  golden/       the frozen numerical baseline -- see its own README
  px4/          the PX4 SITL airframe config. PX4 is not installed anywhere here.
  hardware/     an Arduino sketch used for gimbal bring-up on the bench.
```

Nothing in here is imported by the simulator, and nothing in here runs in CI
except the golden check.

---

## `golden/`

The frozen numbers, so that a change claiming to move none can be checked rather
than trusted. [Its own README](golden/README.md) explains what is and is not
frozen, and when the file may legitimately change.

```bash
python tvc.py golden --check
```

## `px4/4600_tvc_coax`

A PX4 SITL airframe configuring **offboard direct actuator control**: PX4 does
state estimation, arming, failsafes and logging, while the attitude loop runs
outside it.

This is a deliberate choice, not a limitation being worked around. PX4's stock
control allocator assumes control torque comes from thrust *differences* across
fixed-direction rotors. This vehicle gets pitch and yaw from *tilting one thrust
vector*, and its two rotors are nearly co-located, so their thrust difference
gives essentially only roll. **No combination of `CA_*` geometry parameters
expresses that.**

The file's own header documents both paths, including what a custom
`ActuatorEffectivenessTVC` firmware module would involve if PX4's internal
attitude loop is ever needed. It also opens with the axis-convention warning,
because PX4 speaks FRD and calls rotation about the thrust axis "yaw" while we
call it "roll" — see [../docs/4-CONVENTIONS.md §4](../docs/4-CONVENTIONS.md).

To use it, copy into a PX4 source tree and rebuild SITL:

```bash
cp reference/px4/4600_tvc_coax $PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/
```

then add `4600_tvc_coax` to the `airframes` list in that directory's
`CMakeLists.txt`. **PX4 is not in the devcontainer image** and is not needed for
any pipeline in this repository.

## `hardware/servo_sweep.ino`

An Arduino sketch that sweeps the two gimbal servos through their travel, used
for mechanical bring-up on the bench: checking linkage ratios, finding the
mechanical stops, and confirming the servos move the rings the way the geometry
says they should.

It has **no connection to the flight code** and does not follow this
repository's axis convention — it names its two `Servo` objects after the servos
themselves. That is fine and deliberate: it is bench equipment, not vehicle
software, and it would be worse to imply it shares a frame with the simulator.
Do not import numbers from it.
