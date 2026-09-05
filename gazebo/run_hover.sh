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
  gz sim -s -r --headless-rendering "${WORLD}" &
fi

# Wait on the world CONTROL SERVICE, not on a topic: while the sim is paused no
# physics steps run, so the odometry topic does not exist yet and polling for it
# would wait forever. The service is up as soon as the world loads.
echo -n "waiting for simulator"
for _ in $(seq 60); do
  if gz service -l 2>/dev/null | grep -q "/world/${WORLD_NAME}/control$"; then
    echo " ready"; break
  fi
  echo -n "."; sleep 1
done

python3 "${REPO}/tvc.py" hover --duration "${DURATION}" \
        --altitude "${ALTITUDE}" --log "${LOG}" &
CTRL=$!
REC=""
if [[ -n "${RECORD}" ]]; then
  python3 "${REPO}/tvc.py" record --duration "${DURATION}" --out "${RECORD}" &
  REC=$!
fi
sleep 2   # let the controller subscribe and start publishing

echo "unpausing physics"
gz service -s "/world/${WORLD_NAME}/control" \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 3000 --req "pause: false" >/dev/null

wait "${CTRL}"
[[ -n "${REC}" ]] && wait "${REC}" || true
