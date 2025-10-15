#!/usr/bin/env bash
# progressive_beep.sh
# Optional initial delay, then linearly decrease interval from START_INTERVAL to END_INTERVAL over RAMP_SECONDS; then hold forever.

########################################
WAV_FILE="/home/nao/bin/windows-error.wav"
INITIAL_DELAY=300      # Seconds to wait before first play (set 0 for none)
RAMP_SECONDS=600       # Seconds to ramp
START_INTERVAL=45      # Initial gap (s)
END_INTERVAL=0         # Final gap (s) (>=0; if 0 we clamp to MIN_SLEEP)
MIN_SLEEP=0.001        # Minimum gap to avoid busy loop
########################################

set -euo pipefail

[[ -f "$WAV_FILE" ]] || { echo "Error: WAV file '$WAV_FILE' not found." >&2; exit 1; }
awk -v a="$START_INTERVAL" -v b="$END_INTERVAL" 'BEGIN{exit !(a>b)}' \
  || { echo "Error: START_INTERVAL must be > END_INTERVAL." >&2; exit 2; }
awk -v r="$RAMP_SECONDS" 'BEGIN{exit !(r>=0)}' \
  || { echo "Error: RAMP_SECONDS must be ≥ 0." >&2; exit 3; }
awk -v d="$INITIAL_DELAY" 'BEGIN{exit !(d>=0)}' \
  || { echo "Error: INITIAL_DELAY must be ≥ 0." >&2; exit 4; }

# Optional initial sleep before starting the cycle
if (( $(awk -v d="$INITIAL_DELAY" 'BEGIN{print (d>0)}') )); then
  sleep "$INITIAL_DELAY"
fi

START_TIME=$(date +%s.%N)
SPAN=$(awk -v a="$START_INTERVAL" -v b="$END_INTERVAL" 'BEGIN{print a-b}')

current_interval() {
  local now elapsed
  now=$(date +%s.%N)
  elapsed=$(awk -v n="$now" -v s="$START_TIME" 'BEGIN{print n - s}')
  if awk -v e="$elapsed" -v r="$RAMP_SECONDS" 'BEGIN{exit !(e>=r)}'; then
    printf "%.6f\n" "$END_INTERVAL"
  else
    awk -v e="$elapsed" -v r="$RAMP_SECONDS" -v start="$START_INTERVAL" -v span="$SPAN" \
        'BEGIN{printf "%.6f\n", start - span*(e/r)}'
  fi
}

while true; do
  if ! aplay -q "$WAV_FILE"; then
    echo "Playback failed (aplay exit $?); retrying in 1s..." >&2
    sleep 1
    continue
  fi
  interval=$(current_interval)
  awk -v i="$interval" -v end="$END_INTERVAL" -v floor="$MIN_SLEEP" '
    BEGIN{
      if(i<end) i=end;
      if(i<floor) i=floor;
      printf "%.6f\n", i
    }' | {
      read safe_interval
      sleep "$safe_interval"
    }
done

