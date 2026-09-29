#!/bin/sh
# Fill the PCB's zones through the real pcbnew on an Xvfb display.
#
# pcbnew.ZONE_FILLER(...).Fill() segfaults in KiCad 7.0.11's Python bindings,
# so the generator saves the board with its zones defined but unfilled and
# calls this to do the fill with the same engine a GUI user would.
#   B      = Fill All Zones
#   Ctrl+S = save
set -e
DIR=$(cd "$(dirname "$0")" && pwd)
PCB=${1:-$DIR/t1s_hat.kicad_pcb}
DISP=:99
pgrep -x pcbnew | while read pid; do kill "$pid"; done
sleep 2
rm -f "$DIR/~$(basename "$PCB").lck"
if ! xdpyinfo -display $DISP >/dev/null 2>&1; then
    Xvfb $DISP -screen 0 1600x1000x24 >/dev/null 2>&1 &
    sleep 3
fi
export DISPLAY=$DISP
pcbnew "$PCB" >/dev/null 2>&1 &
n=0
while [ $n -lt 90 ]; do
    xdotool search --name "PCB Editor" >/dev/null 2>&1 && break
    n=$((n+1)); sleep 1
done
sleep 8
xdotool mousemove 700 400 click 1; sleep 1     # focus the canvas
xdotool key Escape;                 sleep 1
xdotool key b;                      sleep 20   # Fill All Zones
xdotool key ctrl+s;                 sleep 8    # save
pgrep -x pcbnew | while read pid; do kill "$pid"; done
sleep 2
rm -f "$DIR/~$(basename "$PCB").lck"
