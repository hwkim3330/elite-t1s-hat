#!/usr/bin/env python3
"""Check a JLC CPL against the footprints JLC will actually place.

    python3 hardware/check_jlc_cpl.py [jlcpcb/end]

JLC assembles each line of the BOM with the footprint that belongs to its LCSC
number (EasyEDA's library), positioned and rotated by the CPL. If that
footprint's origin or orientation differs from ours, the part lands off its
pads. This fetches every part's EasyEDA footprint (cached in
~/.cache/elite-t1s-hat/easyeda), places it exactly as the CPL says, and
measures how far each of its pads lands from the board pad with the same
number. Two-pad non-polarised parts may land swapped; anything else must
match pad for pad.
"""
import csv
import json
import math
import os
import sys
import subprocess
import time

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.expanduser("~/.cache/elite-t1s-hat/easyeda")
API = "https://easyeda.com/api/products/%s/components?version=6.4.19.5"
TOL = 0.15                       # mm: pad-centre agreement that counts as "on the pad"
# LEDs are compared by function, not number: EasyEDA's pad 1 is the cathode on
# some LED footprints and the anode on others, and it is where the cathode
# lands that matters. KiCad's LED footprints: pad 1 = K, pad 2 = A.
POLAR = {"D1", "D2"}
LED_PAD = {"K": "1", "A": "2"}
# EasyEDA numbers the ACT1210's pads in mirror order to TDK's drawing (its 1 sits
# where ours is 2). Both have the windings straight across, 1-4 and 2-3, so
# placed on our pads each winding still joins TRX to CMC on its own line:
# the difference is the label, not the connection.
PIN_MAP = {"L1": {"1": "2", "2": "1", "3": "4", "4": "3"}}


def easyeda(lcsc):
    os.makedirs(CACHE, exist_ok=True)
    path = os.path.join(CACHE, lcsc + ".json")
    if not os.path.exists(path):
        # curl, not urllib: EasyEDA's API answers urllib with 403 often enough
        # it also rate-limits bursts with 403, so back off and retry
        for k in range(6):
            if subprocess.run(["curl", "-sSf", "-m", "60", "-o", path, API % lcsc]).returncode == 0:
                break
            time.sleep(20 * (k + 1))
        else:
            sys.exit("EasyEDA would not serve %s; try again later" % lcsc)
    res = json.load(open(path))["result"]
    # symbol pin number -> pin name (K / A for the LEDs)
    names = {}
    for sh in res["dataStr"]["shape"]:
        if sh.startswith("P~"):
            seg = sh.split("^^")
            f = seg[3].split("~") if len(seg) > 3 else []
            names[seg[0].split("~")[3]] = f[4] if len(f) > 4 else ""
    ds = res["packageDetail"]["dataStr"]
    hx, hy = float(ds["head"]["x"]), float(ds["head"]["y"])
    pads = []
    for sh in ds["shape"]:
        if sh.startswith("PAD~"):
            f = sh.split("~")        # EasyEDA units are 10 mil, y down
            pads.append((f[8], (float(f[2]) - hx) * 0.254, -(float(f[3]) - hy) * 0.254))
    return res["packageDetail"]["title"], pads, names


def main(variant):
    board = pcbnew.LoadBoard(os.path.join(HERE, "t1s_hat.kicad_pcb"))
    lcsc = {}
    for r in csv.DictReader(open(os.path.join(variant, "bom_jlc.csv"))):
        for ref in r["Designator"].split(","):
            lcsc[ref.strip()] = r["LCSC Part #"]
    bad = 0
    for r in csv.DictReader(open(os.path.join(variant, "cpl_jlc.csv"))):
        ref = r["Designator"]
        x0, y0 = float(r["Mid X"].rstrip("m")), float(r["Mid Y"].rstrip("m"))
        th = math.radians(float(r["Rotation"]))
        title, epads, names = easyeda(lcsc[ref])
        fp = board.FindFootprintByReference(ref)
        bpads = {}
        for p in fp.Pads():
            q = p.GetPosition()
            bpads.setdefault(p.GetNumber(), []).append((q.x / 1e6 - 100.0, 100.0 - q.y / 1e6))
        placed = [(n, x0 + x * math.cos(th) - y * math.sin(th),
                   y0 + x * math.sin(th) + y * math.cos(th)) for n, x, y in epads]

        def err(mapping):
            worst = 0.0
            for n, x, y in placed:
                cands = bpads.get(mapping.get(n, n))
                if not cands:
                    continue         # e.g. an EasyEDA-only mechanical pad
                worst = max(worst, min(math.hypot(x - bx, y - by) for bx, by in cands))
            return worst
        if ref in POLAR:
            m = {n: LED_PAD[names[n]] for n in names if names[n] in LED_PAD}
            if len(m) != 2:
                sys.exit("%s: EasyEDA symbol has no K/A pin names" % ref)
            e, note = err(m), " (cathode on pad %s)" % [k for k, v in m.items() if v == "1"][0]
        else:
            e = err(PIN_MAP.get(ref, {}))
            note = " (EasyEDA pin numbers mirrored, windings same)" if ref in PIN_MAP else ""
        if e > TOL and len(epads) == 2 and ref not in POLAR:
            e2 = err({"1": "2", "2": "1"})
            if e2 <= TOL:
                e, note = e2, " (pads swapped, non-polarised)"
        ok = e <= TOL
        bad += not ok
        print("%-4s %-10s %-44s rot %6.1f  worst pad %.3f mm  %s%s" % (
            ref, lcsc[ref], title[:44], float(r["Rotation"]), e, "OK" if ok else "FAIL", note))
    print("\n%s: %d part(s) off their pads" % (variant, bad))
    return bad


if __name__ == "__main__":
    v = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "jlcpcb", "end")
    sys.exit(1 if main(v) else 0)
