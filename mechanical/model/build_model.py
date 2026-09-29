#!/usr/bin/env python3
"""3D models of the T1S HAT, for viewing (GitHub renders STL in the browser) and for
enclosure work.

    python3 mechanical/model/build_model.py

writes, next to this file:
    t1s_hat.step   the HAT with its parts (KiCad export + the parts KiCad has no model for)
    t1s_hat.stl    the same, tessellated
    stack.stl      the Elite (a plain stand-in, see below) + riser + standoffs + the HAT
                   at its stack height

Requires kicad-cli (KiCad 7), cadquery, trimesh, numpy. The KiCad 3D part library is not
needed installed: the STEP models this board uses are fetched from KiCad's own
kicad-packages3D repository into ~/.cache/elite-t1s-hat/3dmodels.

LilyGo publish their 3D model of the Elite without a licence, so it is not
redistributed here and the committed stack.stl uses a stand-in built from this repo's
own measurements: the PCB outline, the RJ45 block and the header body. If LilyGo's
model has been fetched (reference/measure.py --fetch) it is used for the renders only.

Coordinates: GEOMETRY.md's frame (Elite top view, origin at its PCB's bottom-left
corner, mm). The board file's drill origin sits on that origin, so the KiCad STEP export
lands in it directly; LilyGo's Elite STL is moved in by measure.py's (BX, BY).
"""
import os
import re
import subprocess
import sys
import tempfile
import urllib.parse
import urllib.request

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
PCB = os.path.join(REPO, "hardware", "t1s_hat.kicad_pcb")
ELITE_STL = os.path.join(REPO, "mechanical", "reference", "T-ETH-ELite-view.stl")
CACHE = os.path.expanduser("~/.cache/elite-t1s-hat/3dmodels")
PKG3D = "https://gitlab.com/kicad/libraries/kicad-packages3D/-/raw/{ref}/{path}"

BX, BY = 33.0955, 24.596          # Elite STL -> GEOMETRY.md frame (reference/measure.py)
ELITE_TOP = 0.80                  # Elite PCB top face in its STL (1.6 mm slab centred on z=0)
BOARD_T = 1.6
# Elite PCB top -> HAT underside. Calculated, not measured (ORDERING.md): riser body 8.3
# + ~6 of its pins inside J1 + J1's 8.5 body.
STACK = 23.0
HDR_CX, HDR_CY = (8.196 + 56.456) / 2, (3.360 + 5.900) / 2   # 2x20 grid centre
HOLES = [(3.33, 4.63), (61.33, 4.63), (2.98, 46.23), (63.23, 46.20)]


def fetch_models(pcb_text):
    """Every model the board references, as STEP, from kicad-packages3D (7.0.0, then master)."""
    paths = sorted(set(re.findall(r'\(model "\$\{KICAD[67]_3DMODEL_DIR\}/([^"]+)\.wrl"', pcb_text)))
    # KiCad renamed this one; the 3.45 mm-pad QFN-32 5x5 is the same body
    alias = {"Package_DFN_QFN.3dshapes/TQFN-32-1EP_5x5mm_P0.5mm_EP3.4x3.4mm":
             "Package_DFN_QFN.3dshapes/QFN-32-1EP_5x5mm_P0.5mm_EP3.45x3.45mm"}
    for p in paths:
        dst = os.path.join(CACHE, p + ".step")
        if os.path.exists(dst) and os.path.getsize(dst) > 1000:
            continue
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        src = alias.get(p, p) + ".step"
        for ref in ("7.0.0", "master"):
            try:
                url = PKG3D.format(ref=ref, path=urllib.parse.quote(src))
                with urllib.request.urlopen(url, timeout=60) as r, open(dst, "wb") as f:
                    f.write(r.read())
                break
            except Exception:
                continue
        else:
            print("  no model for", p)
    return paths


def export_hat_step(out):
    """KiCad STEP of the HAT, with J1's model left out -- the footprint's model is a male
    header on top, the real part is a female socket underneath, added in build_hat()."""
    s = open(PCB).read()
    fetch_models(s)
    s = re.sub(r'\(model "\$\{KICAD6_3DMODEL_DIR\}/Connector_PinHeader_2\.54mm\.3dshapes/'
               r'PinHeader_2x20_P2\.54mm_Vertical\.wrl".*?\n\s*\)\n', "", s, count=1, flags=re.S)
    # kicad-cli 7 does not expand the model-dir variables here: point at the cache directly
    s = re.sub(r'\(model "\$\{KICAD[67]_3DMODEL_DIR\}/([^"]+)\.wrl"',
               lambda m: '(model "%s/%s.step"' % (CACHE, m.group(1)), s)
    with tempfile.TemporaryDirectory() as td:
        tmp = os.path.join(td, "hat.kicad_pcb")
        open(tmp, "w").write(s)
        r = subprocess.run(["kicad-cli", "pcb", "export", "step", "--drill-origin", "--force",
                            "-o", out, tmp], capture_output=True, text=True)
        missing = r.stdout.count("Could not add 3D model")
        if r.returncode != 0 or not os.path.exists(out):
            sys.exit("kicad-cli step export failed:\n" + r.stdout + r.stderr)
    print("  KiCad STEP: %s (%d part models missing)" % (os.path.basename(out), missing))


def part_xy(ref):
    """G-frame position and rotation of a footprint, read with KiCad's own pcbnew."""
    import pcbnew
    fp = pcbnew.LoadBoard(PCB).FindFootprintByReference(ref)
    p = fp.GetPosition()
    return p.x / 1e6 - 100.0, 100.0 - p.y / 1e6, fp.GetOrientationDegrees()


def build_hat(step_out, stl_out):
    import cadquery as cq
    export_hat_step(step_out)
    hat = cq.importers.importStep(step_out)
    # L1: TDK ACT1210 body, 3.2 x 2.5 x 2.5 (rotated 90 deg on the board)
    x, y, rot = part_xy("L1")
    l1 = cq.Workplane("XY").box(2.5, 3.2, 2.5, centered=(True, True, False)).translate((x, y, BOARD_T))
    # J1: 2x20 female socket under the board, 8.5 mm body, with its 40 openings
    sock = (cq.Workplane("XY").box(50.8, 5.08, 8.5, centered=(True, True, False))
            .translate((HDR_CX, HDR_CY, -8.5)))
    pts = [(8.196 + 2.54 * k, yy) for k in range(20) for yy in (3.360, 5.900)]
    sock = sock.cut(cq.Workplane("XY").pushPoints(pts).rect(1.0, 1.0).extrude(8.5).translate((0, 0, -8.5)))
    model = cq.Assembly().add(hat, name="hat").add(l1, name="L1").add(sock, name="J1")
    compound = model.toCompound()
    cq.exporters.export(cq.Workplane().add(compound), step_out)
    cq.exporters.export(cq.Workplane().add(compound), stl_out, tolerance=0.03, angularTolerance=0.25)
    # render.py colours the socket like the other connectors, so it also gets a file of its own
    pd = os.path.join(HERE, "_parts")
    os.makedirs(pd, exist_ok=True)
    board_only = cq.Assembly().add(hat, name="hat").add(l1, name="L1").toCompound()
    cq.exporters.export(cq.Workplane().add(board_only), os.path.join(pd, "hat_board.stl"), tolerance=0.03, angularTolerance=0.25)
    cq.exporters.export(sock, os.path.join(pd, "j1_socket.stl"), tolerance=0.03, angularTolerance=0.25)
    print("  HAT STL: %s" % os.path.basename(stl_out))


def elite_standin():
    """The Elite from GEOMETRY.md's numbers only: PCB 66.22 x 49.19 x 1.6 (centred on
    z = 0, like LilyGo's model), the RJ45 (x -5.18..16.42, y 10.50..28.90, top 15.97 above
    the PCB) and the 2x20 header body."""
    pcb = trimesh.creation.box((66.22, 49.19, 1.6))
    pcb.apply_translation((66.22 / 2, 49.19 / 2, 0.0))
    rj45 = trimesh.creation.box((21.6, 18.4, 15.97))
    rj45.apply_translation(((-5.18 + 16.42) / 2, (10.50 + 28.90) / 2, ELITE_TOP + 15.97 / 2))
    hdr = trimesh.creation.box((50.8, 5.08, 2.5))
    hdr.apply_translation((HDR_CX, HDR_CY, ELITE_TOP + 1.25))
    return trimesh.util.concatenate([pcb, rj45, hdr])


def elite_lilygo():
    """LilyGo's own model, if fetched locally; None otherwise."""
    ref = os.path.join(REPO, "mechanical", "reference")
    if os.path.exists(ELITE_STL):
        m = trimesh.load(ELITE_STL)
    elif os.path.exists(os.path.join(ref, "T-ETH-ELite.7z")):
        sys.path.insert(0, ref)
        import measure
        m = trimesh.load(measure.stl_from_7z("T-ETH-ELite.7z"))
    else:
        return None
    m.apply_translation((BX, BY, 0.0))                # into GEOMETRY.md's frame
    return m


def build_stack(hat_stl, out):
    elite = elite_standin()
    hat = trimesh.load(hat_stl)
    hat.apply_translation((0, 0, ELITE_TOP + STACK))  # HAT underside at the stack height
    pd = os.path.join(HERE, "_parts")
    hat_board = trimesh.load(os.path.join(pd, "hat_board.stl"))
    hat_board.apply_translation((0, 0, ELITE_TOP + STACK))
    sock = trimesh.load(os.path.join(pd, "j1_socket.stl"))
    sock.apply_translation((0, 0, ELITE_TOP + STACK))
    hat_under = ELITE_TOP + STACK
    groups = {"elite": [elite], "hat": [hat_board], "connectors": [sock], "standoffs": []}
    # riser: female body sitting on the Elite's own header, long pins up into J1
    riser_z0 = ELITE_TOP + 2.5
    body = trimesh.creation.box((50.8, 5.08, 8.3))
    body.apply_translation((HDR_CX, HDR_CY, riser_z0 + 8.3 / 2))
    groups["connectors"].append(body)
    pin_top = hat_under - 8.5 + 6.0                   # ~6 mm of pin inside the socket
    for k in range(20):
        for yy in (3.360, 5.900):
            pin = trimesh.creation.box((0.64, 0.64, pin_top - (riser_z0 + 8.3)))
            pin.apply_translation((8.196 + 2.54 * k, yy, (riser_z0 + 8.3 + pin_top) / 2))
            groups["connectors"].append(pin)
    # M2.5 standoffs, Elite top face to HAT underside
    for x, y in HOLES:
        st = trimesh.creation.cylinder(radius=2.5, height=STACK, sections=6)
        st.apply_translation((x, y, ELITE_TOP + STACK / 2))
        groups["standoffs"].append(st)
    trimesh.util.concatenate([m for g in groups.values() for m in g]).export(out)
    # the same parts one file per material, for render.py only (not committed) -- with
    # LilyGo's real Elite if it is on this machine
    real = elite_lilygo()
    if real is not None:
        groups["elite"] = [real]
    print("  render Elite: %s" % ("LilyGo's model" if real is not None else "stand-in"))
    for name, ms in groups.items():
        trimesh.util.concatenate(ms).export(os.path.join(pd, name + ".stl"))
    print("  stack STL: %s (HAT underside %.1f mm above the Elite's PCB)" % (os.path.basename(out), STACK))


def main():
    step = os.path.join(HERE, "t1s_hat.step")
    stl = os.path.join(HERE, "t1s_hat.stl")
    build_hat(step, stl)
    build_stack(stl, os.path.join(HERE, "stack.stl"))


if __name__ == "__main__":
    main()
