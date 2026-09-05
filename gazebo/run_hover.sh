#!/usr/bin/env bash
# Gazebo hover demo: start the world, attach the controller, unpause.
#
#   bash gazebo/run_hover.sh                          headless, 30 simulated s
#   bash gazebo/run_hover.sh --gui                    with the Gazebo window
#   bash gazebo/run_hover.sh --duration 60 --altitude 3.0
#   bash gazebo/run_hover.sh --log reference/golden/hover_baseline.csv
#
# Devcontainer only -- gz-sim and gz-transport live there. This is the
# no-colcon-build path: it drives Gazebo directly, so it needs nothing built.
# The ROS 2 equivalent is `ros2 launch tvc_control gazebo.launch.py`.
#
# START PAUSED, ATTACH, THEN UNPAUSE. That ordering is not cosmetic: the vehicle
# free-falls from its 2 m spawn in about 0.6 s, so a controller connecting even
# a second after an unpaused start finds it already on the ground -- and the
# resulting plot looks exactly like a control failure.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="${REPO}/gazebo/worlds/tvc_flight.sdf"
WORLD_NAME="tvc_flight"
# Prepend, never assign: ros_gz_sim puts /opt/ros/<distro>/share on this
# variable, and overwriting it hides every resource ROS ships.
export GZ_SIM_RESOURCE_PATH="${REPO}/gazebo/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

GUI=0
DURATION=30
ALTITUDE=2.0
RECORD=""
LOG="/tmp/flight.csv"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui)       GUI=1; shift ;;
    --duration)  DURATION="$2"; shift 2 ;;
    --altitude)  ALTITUDE="$2"; shift 2 ;;
    --record)    RECORD="$2"; shift 2 ;;
    --log)       LOG="$2"; shift 2 ;;
    -h|--help)   sed -n '2,12p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

# Killing the simulator is fiddlier than it looks:
#   - `gz` is a Ruby wrapper, so the actual process has comm "ruby" and
#     `pkill -x gz` matches nothing at all.
#   - `pkill -f "gz sim"` DOES match, but it also matches any shell whose own
#     command line contains that string -- including the one running this
#     script, which then kills itself mid-run.
# So: match comm == ruby AND "gz sim" in the arguments. Leftovers are not
# harmless: several servers publishing on the same topics feed the controller
# interleaved state from different worlds, which reads as a physics instability.
cleanup() {
  ps -eo pid,comm,args --no-headers \
    | awk '$2=="ruby" && /gz sim/ {print $1}' \
    | xargs -r kill -9 2>/dev/null || true
}
trap cleanup EXIT
cleanup; sleep 1

if [[ "$GUI" == "1" ]]; then
  echo "starting Gazebo WITH GUI (paused)..."
  gz sim "${WORLD}" &          # no -r: starts paused
else
  echo "starting Gazebo headless (paused)..."
  # --headless-rendering: required for the chase camera to produce frames with
  # no display attached. Without it the sensor exists but never renders.
  #
  # NO -r. This branch used to carry it, which starts physics immediately and
  # flatly contradicts the paused-start design this script's header explains --
  # the vehicle was already falling before the controller had subscribed. The
  # GUI branch never had it, so the headless run and the one you watched were
  # not the same experiment.
  gz sim -s --headless-rendering "${WORLD}" &
fi

# Wait by CALLING the control service, not by listing it. Two reasons: while the
# sim is paused no physics steps run, so the odometry topic does not exist yet
# and polling for that would wait forever; and gz-transport keeps a killed
# server's service NAMES in discovery for some seconds afterwards, so
# `gz service -l | grep` returns instantly on the corpse of the previous run and
# everything downstream then talks to a world that has not loaded. A call that
# returns proves a live server. The request is `pause: true`, so waiting cannot
# accidentally start the run.
echo -n "waiting for simulator"
for _ in $(seq 60); do
  if gz service -s "/world/${WORLD_NAME}/control" \
       --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
       --timeout 1000 --req "pause: true" 2>/dev/null | grep -q "data: true"; then
    echo " ready"; break
  fi
  echo -n "."; sleep 1
done

REC=""
if [[ -n "${RECORD}" ]]; then
  python3 "${REPO}/tvc.py" record --duration "${DURATION}" --out "${RECORD}" &
  REC=$!
  sleep 2   # the camera recorder must be attached before the first frame
fi

# THE CONTROLLER UNPAUSES, not this script. A `sleep 2; gz service` here made
# the number of uncontrolled physics steps depend on how fast the host got
# Python to its first publish, and two consecutive 30 s runs then peaked at
# 14.6 and 58.0 degrees of thrust-axis roll from the same initial condition.
# See tvc_control/harness/gz.py::unpause.
python3 "${REPO}/tvc.py" hover --duration "${DURATION}" \
        --altitude "${ALTITUDE}" --log "${LOG}" --unpause "${WORLD_NAME}" &
CTRL=$!

wait "${CTRL}"
[[ -n "${REC}" ]] && wait "${REC}" || true
