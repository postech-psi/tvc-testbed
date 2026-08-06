#!/usr/bin/env bash
# Launch the Gazebo TVC hover demo.
#
# Starts the simulator PAUSED, attaches the controller, and only then lets
# physics run. That ordering is not optional: the vehicle free-falls from its
# 2 m spawn in about 0.6 s, so a controller that connects even a couple of
# seconds after an unpaused start finds it already on the ground.
#
#   ./sim/run_hover.sh                 headless, 30 s
#   ./sim/run_hover.sh --gui           with the Gazebo window
#   ./sim/run_hover.sh --duration 60 --altitude 3.0
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="${REPO}/sim/worlds/tvc_flight.sdf"
WORLD_NAME="tvc_flight"
export GZ_SIM_RESOURCE_PATH="${REPO}/sim/models"

GUI=0
DURATION=30
ALTITUDE=2.0
RECORD=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui)       GUI=1; shift ;;
    --duration)  DURATION="$2"; shift 2 ;;
    --altitude)  ALTITUDE="$2"; shift 2 ;;
    --record)    RECORD="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

# Killing the simulator is fiddlier than it looks:
#   - `gz` is a Ruby wrapper, so the actual process has comm "ruby", and
#     `pkill -x gz` matches nothing at all.
#   - `pkill -f "gz sim"` DOES match, but it also matches any shell whose own
#     command line contains that string, including the one running this
#     script, which then kills itself mid-run.
# So: match on comm == ruby AND "gz sim" in the arguments. Leftover servers
# are not harmless -- several publishing on the same topics feed the
# controller interleaved state from different worlds.
cleanup() {
  ps -eo pid,comm,args --no-headers     | awk '$2=="ruby" && /gz sim/ {print $1}'     | xargs -r kill -9 2>/dev/null || true
}
trap cleanup EXIT
cleanup; sleep 1

if [[ "$GUI" == "1" ]]; then
  echo "starting Gazebo WITH GUI (paused)..."
  gz sim "${WORLD}" &          # no -r: starts paused
else
  echo "starting Gazebo headless (paused)..."
  # --headless-rendering: required for the chase camera to produce frames
  # with no display attached. Without it the sensor exists but never renders.
  gz sim -s -r --headless-rendering "${WORLD}" &
fi

# Wait on the world CONTROL SERVICE, not on a topic: while the sim is paused
# no physics steps run, so the odometry topic does not exist yet and polling
# for it would wait forever. The service is up as soon as the world loads.
echo -n "waiting for simulator"
for _ in $(seq 60); do
  if gz service -l 2>/dev/null | grep -q "/world/${WORLD_NAME}/control$"; then
    echo " ready"; break
  fi
  echo -n "."; sleep 1
done

python3 "${REPO}/sim/hover.py" --duration "${DURATION}" --altitude "${ALTITUDE}" --log /tmp/flight.csv &
CTRL=$!
REC=""
if [[ -n "${RECORD}" ]]; then
  python3 "${REPO}/sim/record.py" --duration "${DURATION}" --out "${RECORD}" &
  REC=$!
fi
sleep 2   # let the controller subscribe and start publishing

echo "unpausing physics"
gz service -s "/world/${WORLD_NAME}/control" \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 3000 --req "pause: false" >/dev/null

wait "${CTRL}"
[[ -n "${REC}" ]] && wait "${REC}" || true
