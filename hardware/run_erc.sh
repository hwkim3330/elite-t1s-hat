#!/bin/sh
# Run eeschema's own ERC headlessly and save its report.
#
# This machine's kicad-cli (7.0.11) has no `sch erc` subcommand -- only
# `sch export` -- so the checker is driven through the real GUI on an Xvfb
# display.  Same engine, same defaults, nothing filtered: "Show: All" and
# "Exclusions" are both ticked before the run so the saved report contains
# every violation the tool produces.
#
# Usage:  sh run_erc.sh [schematic.kicad_sch]        -> writes erc.rpt
set -e
DIR=$(cd "$(dirname "$0")" && pwd)
SCH=${1:-$DIR/t1s_hat.kicad_sch}
DISP=:99
OUT=$DIR/erc.rpt

pgrep -x eeschema | while read pid; do kill "$pid"; done
sleep 2
rm -f "$OUT" "$DIR/~$(basename "$SCH").lck"
if ! xdpyinfo -display $DISP >/dev/null 2>&1; then
    Xvfb $DISP -screen 0 1600x1000x24 >/dev/null 2>&1 &
    sleep 3
fi
export DISPLAY=$DISP
eeschema "$SCH" >/dev/null 2>&1 &
n=0
while [ $n -lt 60 ]; do
    xdotool search --name "Schematic Editor" >/dev/null 2>&1 && break
    n=$((n+1)); sleep 1
done
sleep 8

xdotool mousemove 198 13 click 1;  sleep 2     # Inspect menu
xdotool mousemove 269 40 click 1;  sleep 5     # Electrical Rules Checker
xdotool mousemove 374 491 click 1; sleep 1     # Show: All
xdotool mousemove 742 491 click 1; sleep 1     # Show: Exclusions
xdotool mousemove 920 541 click 1; sleep 12    # Run ERC
xdotool mousemove 917 491 click 1; sleep 4     # Save...
xdotool type --delay 40 "erc";      sleep 1   # stem only: ".rpt" is already there
xdotool mousemove 1206 878 click 1; sleep 4    # Save
pgrep -x eeschema | while read pid; do kill "$pid"; done
sleep 1
tail -3 "$OUT"
