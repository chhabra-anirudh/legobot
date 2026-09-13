#!/usr/bin/env bash
# Copy this repo's robot scripts onto the BracketBot, and pull measurements back.
#
# Other teams share this robot and ~/bbapps is not version controlled, so treat
# the robot copy as disposable and this repo as the source of truth.
#
#   ./robot/deploy.sh push          # repo -> robot (restores our scripts)
#   ./robot/deploy.sh pull          # robot -> repo (saves new taught poses)
#   BOT=bracketbot-0153.local ./robot/deploy.sh push
#
# Assumes an SSH control master is already up, e.g.
#   ssh -M -S ~/.ssh/claude-%C -o ControlPersist=1h bracketbot@bracketbot-0152.local
set -euo pipefail

BOT="${BOT:-bracketbot-0152.local}"
USER_AT="bracketbot@${BOT}"
REMOTE=/home/bracketbot/bbapps
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SSH_OPTS=(-o ControlPath="$HOME/.ssh/claude-%C")

SCRIPTS=(build_bridge.py probe_arm.py teach_pose.py)
DIAGNOSTICS=(chan_check.py log_read.py two_readers.py)

case "${1:-}" in
  push)
    for f in "${SCRIPTS[@]}"; do
      scp "${SSH_OPTS[@]}" "$HERE/$f" "$USER_AT:$REMOTE/$f"
      echo "pushed $f"
    done
    for f in "${DIAGNOSTICS[@]}"; do
      scp "${SSH_OPTS[@]}" "$HERE/diagnostics/$f" "$USER_AT:$REMOTE/$f"
      echo "pushed $f"
    done
    # Only seed taught poses if the robot has none; never clobber newer measurements.
    if ssh "${SSH_OPTS[@]}" "$USER_AT" "test -e $REMOTE/taught_poses.json"; then
      echo "kept the robot's existing taught_poses.json (run 'pull' to save it here)"
    else
      scp "${SSH_OPTS[@]}" "$HERE/calibration/taught_poses.json" "$USER_AT:$REMOTE/taught_poses.json"
      echo "seeded taught_poses.json"
    fi
    ;;
  pull)
    scp "${SSH_OPTS[@]}" "$USER_AT:$REMOTE/taught_poses.json" "$HERE/calibration/taught_poses.json"
    scp "${SSH_OPTS[@]}" "$USER_AT:/home/bracketbot/bbos/bbos/daemons/arm_left/ranges.calibration.json" \
        "$HERE/calibration/ranges.calibration.json"
    echo "pulled taught_poses.json and ranges.calibration.json into robot/calibration/"
    ;;
  *)
    sed -n '2,12p' "${BASH_SOURCE[0]}"
    exit 2
    ;;
esac
