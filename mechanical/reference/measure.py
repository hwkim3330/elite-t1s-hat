#!/usr/bin/env python3
"""Re-derive every number in ../t_eth_elite_hat_adapter/GEOMETRY.md from LilyGo's files.

    python3 measure.py              # uses the vendored archives in this folder
    python3 measure.py --fetch      # download them from LilyGo's repo first

Nothing in GEOMETRY.md is hand-measured or read off a drawing by eye. This is
the script that produced it, kept so the claim is auditable rather than
asserted: run it and the four mounting holes, the 40-pin grid and the component
heights come back out of LilyGo's own DXF and 3D model.

Requires: ezdxf, trimesh, py7zr (pip install --user).
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = "https://raw.githubusercontent.com/Xinyuan-LilyGO/LilyGO-T-ETH-Series/master"
FILES = {
    "T-ETH-ELite.dxf": f"{REPO}/shell/T-ETH-ELite.dxf",
    "T-ETH-ELite.7z": f"{REPO}/shell/3D/T-ETH-ELite.7z",
    "T-ETH-ELite-LoRa-Shield.7z": f"{REPO}/shell/3D/T-ETH-ELite-LoRa-Shield.7z",
}
# board-centred CAD -> board-local, origin at the PCB's bottom-left corner
BX, BY = 33.0955, 24.596


def fetch():
    import urllib.request
    for name, url in FILES.items():
        dst = os.path.join(HERE, name)
        if os.path.exists(dst):
            print(f"  have {name}")
            continue
        print(f"  get  {name}")
        urllib.request.urlretrieve(url, dst)


def need(name):
    p = os.path.join(HERE, name)
    if not os.path.exists(p):
        sys.exit(f"missing {name} - run with --fetch")
    return p


def stl_from_7z(name):
    """Extract the .stl out of a vendored .7z into a temp dir, return its path."""
    import tempfile
    import py7zr
    out = os.path.join(tempfile.gettempdir(), "lilygo_cad")
    os.makedirs(out, exist_ok=True)
    with py7zr.SevenZipFile(need(name)) as z:
        members = z.getnames()
        z.extractall(out)
    return os.path.join(out, members[0])


def components(mesh):
    return mesh.split(only_watertight=False)


def holes_from_dxf():
    """Mounting holes out of the 2D mechanical DXF (flattened to LINE entities)."""
    import ezdxf
    from collections import defaultdict
    doc = ezdxf.readfile(need("T-ETH-ELite.dxf"))
    segs = [((e.dxf.start.x, e.dxf.start.y), (e.dxf.end.x, e.dxf.end.y))
            for e in doc.modelspace() if e.dxftype() == "LINE"]
    # circles are exported as short segment chains; recover them as connected
    # components on a 0.01mm grid
    parent = {}

    def find(a):
        parent.setdefault(a, a)
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    def key(p):
        return (round(p[0] * 100), round(p[1] * 100))

    for s, e in segs:
        union(key(s), key(e))
    comp = defaultdict(list)
    for s, e in segs:
        comp[find(key(s))].append((s, e))

    boards, holes = [], []
    for ss in comp.values():
        xs = [p[0] for sg in ss for p in sg]
        ys = [p[1] for sg in ss for p in sg]
        w, h = max(xs) - min(xs), max(ys) - min(ys)
        if w > 60 and h > 45:
            boards.append((w, h, min(xs), min(ys)))
        elif 2.3 < w < 3.3 and abs(w - h) < 0.15:
            holes.append(((max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2, w))
    # the outline is drawn twice (line width); the board origin is their midpoint
    x0 = np.mean([b[2] for b in boards])
    y0 = np.mean([b[3] for b in boards])
    return sorted([(x - x0, y - y0, d) for x, y, d in holes], key=lambda t: (t[1], t[0]))


def from_3d(stl, label):
    import trimesh
    m = trimesh.load(stl)
    bodies = components(m)
    slab = max((b for b in bodies if 1.4 < (b.bounds[1] - b.bounds[0])[2] < 1.8),
               key=lambda b: np.prod((b.bounds[1] - b.bounds[0])[:2]))
    e = slab.bounds[1] - slab.bounds[0]
    print(f"\n[{label}] PCB slab {e[0]:.3f} x {e[1]:.3f} x {e[2]:.3f}")

    # holes: interior rings of the slab's midplane section
    sec = slab.section(plane_origin=[0, 0, 0], plane_normal=[0, 0, 1])
    p2, T = sec.to_2D()
    rings = []
    for poly in p2.polygons_full:
        for r in poly.interiors:
            xs = [p[0] for p in r.coords]
            ys = [p[1] for p in r.coords]
            rings.append((max(xs) - min(xs), max(ys) - min(ys),
                          (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2))
    pts = np.array([[r[2], r[3], 0, 1] for r in rings])
    absxy = (T @ pts.T).T[:, :2]
    print("  mounting holes (Ø2.3-2.9):")
    for r, a in sorted(zip(rings, absxy), key=lambda t: (t[1][1], t[1][0])):
        if 2.3 < r[0] < 2.9 and abs(r[0] - r[1]) < 0.2:
            print(f"    ({a[0]+BX:7.3f}, {a[1]+BY:7.3f})  Ø{r[0]:.3f}")

    # the 40-pin header: the body exactly 50.80 wide (= 20 x 2.54)
    hdr = next((b for b in bodies
                if abs((b.bounds[1] - b.bounds[0])[0] - 50.80) < 0.05
                and 7 < (b.bounds[1] - b.bounds[0])[1] < 8), None)
    if hdr is not None:
        bb = hdr.bounds
        s = hdr.section(plane_origin=[0, 0, (bb[1][2] - 0.6)], plane_normal=[0, 0, 1])
        q2, T2 = s.to_2D()
        c = np.array([[p.centroid.x, p.centroid.y, 0, 1] for p in q2.polygons_full])
        c3 = (T2 @ c.T).T[:, :2]
        xs = np.sort(np.unique(np.round(c3[:, 0] + BX, 3)))
        ys = np.sort(np.unique(np.round(c3[:, 1] + BY, 3)))
        print(f"  header: {len(q2.polygons_full)} pins, {len(xs)} cols x {len(ys)} rows")
        print(f"    x {xs.min():.3f} .. {xs.max():.3f}, pitch "
              f"{np.unique(np.round(np.diff(xs),3))}")
        print(f"    y {ys}")
        print(f"    plastic body x {bb[0][0]+BX:.3f}..{bb[1][0]+BX:.3f}  "
              f"y {bb[0][1]+BY:.3f}..{bb[1][1]+BY:.3f}")
        print(f"    pin tip z {bb[1][2]:.3f} (PCB top 0.80 -> {bb[1][2]-0.80:.2f} above)")

    # outline, simplified - shows the notches
    from shapely.geometry import Polygon
    poly = max(p2.polygons_full, key=lambda p: p.area)
    ring = np.array(poly.exterior.coords)
    pts = np.c_[ring, np.zeros(len(ring)), np.ones(len(ring))]
    a = (T @ pts.T).T[:, :2] + [BX, BY]
    print("  outline (simplified 0.05):")
    print("   ", np.round(np.array(Polygon(a).simplify(0.05).exterior.coords), 2).tolist())

    # what a stacked board has to clear
    print("  parts with top > 3.5mm:")
    tall = []
    for b in bodies:
        bb = b.bounds
        ex = bb[1] - bb[0]
        if bb[1][2] > 3.5 and ex[0] * ex[1] > 3:
            tall.append((bb[1][2], bb[0][0] + BX, bb[0][1] + BY,
                         bb[1][0] + BX, bb[1][1] + BY))
    for t in sorted(tall, reverse=True)[:6]:
        print(f"    top z {t[0]:6.2f}  x {t[1]:6.2f}..{t[3]:6.2f}  y {t[2]:6.2f}..{t[4]:6.2f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--fetch", action="store_true")
    a = ap.parse_args()
    if a.fetch:
        fetch()
    print("[DXF] mounting holes, board-local:")
    for x, y, d in holes_from_dxf():
        print(f"    ({x:7.3f}, {y:7.3f})  Ø{d:.3f}")
    from_3d(stl_from_7z("T-ETH-ELite.7z"), "3D base board")
    from_3d(stl_from_7z("T-ETH-ELite-LoRa-Shield.7z"), "3D LilyGo LoRa shield")
