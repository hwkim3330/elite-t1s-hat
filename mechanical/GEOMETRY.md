# Verified geometry — the single source of truth for this board

> **Correction, 2026-09-28: header pin numbering.** This file originally said
> nothing about which end is pin 1, and the boards inferred it as the low-x
> end on the edge row. That was the exact 180° opposite. LilyGo's own pin-map
> image of the Elite (top view) shows 3V3 (pin 1) at the USB-C end on the inner
> column. That orientation of the photo is pinned down twice by numbers in this
> file: the 58.00 mm hole pair is the header side, and the per-end hole offsets
> (1.9 mm at one end, 0.35 mm at the other) match the photo with USB-C at high
> x; the WROOM antenna, at the USB-C end in the photo, is where the keepout
> sits (x 58…66). The grid positions themselves were always right.
>
> Carried over on 2026-09-28 from an earlier adapter project (a Pi-HAT carrier
> for the Elite), where it was first written. The
> T1S HAT uses its outline, mounting holes and 2×20 grid as-is. Its notch
> section does not apply here: this board clears the RJ45 by stack height
> instead (see `../hardware/README.md`). `reference/measure.py` re-derives
> every number from LilyGo's own files, in `reference/`.

Every number below was measured from a primary source and cross-checked against
at least one independent one. Nothing here is estimated, rounded from a photo,
or carried over from the Raspberry Pi HAT spec by assumption. **If the KiCad
files and this file ever disagree, this file is right and the board is wrong.**

Frame used throughout: **T-ETH-Elite top view, origin = its PCB bottom-left
corner, +x right, +y up.** All units mm.

## Sources

| tag | what | where |
|---|---|---|
| `DXF` | LilyGo 2D mechanical DXF | `shell/T-ETH-ELite.dxf`, github.com/Xinyuan-LilyGO/LilyGO-T-ETH-Series |
| `CAD` | LilyGo 3D model of the base board | `shell/3D/T-ETH-ELite.7z` → `T-ETH-ELite.stl`, same repo |
| `SHLD` | LilyGo 3D model of their own LoRa shield | `shell/3D/T-ETH-ELite-LoRa-Shield.7z`, same repo |
| `RIG` | `/home/kim/stl-model/acrylic-frame/make_plates.py` — the same hole set, **physically validated**: a real board was offered up to a laser-cut plate and sat centred on its holes (see that repo's `CUTTING.md`) |

## T-ETH-Elite base board

| item | value | source |
|---|---|---|
| PCB outline | 66.191 × 49.192, R3 corners | `CAD` slab; `DXF` agrees |
| antenna keepout notch (right edge) | x 59.84…66.19, y 23.93…42.22 | `CAD` outline |
| PCB thickness | 1.575 (≈1.6) | `CAD` |

### Mounting holes — asymmetric, NOT a rectangle

| # | `DXF` | `CAD` | `SHLD` | **use this** (`RIG`, physically validated) |
|---|---|---|---|---|
| bottom-left | (3.327, 4.628) | (3.339, 4.630) | (3.333, 4.636) | **(3.33, 4.63)** |
| bottom-right | (61.367, 4.638) | (61.339, 4.630) | (61.433, 4.636) | **(61.33, 4.63)** |
| top-left | (2.987, 46.288) | (2.991, 46.228) | (3.004, 46.228) | **(2.98, 46.23)** |
| top-right | (63.227, 46.198) | (63.241, 46.198) | (63.254, 46.198) | **(63.23, 46.20)** |

Bottom pair spacing **58.00**, top pair **60.25** — the asymmetry is real design,
not a CAD slip; three independent files agree to <0.1. PCB hole Ø2.500 on the
board itself; M2.5 hardware.

A useful consequence: **the pattern is self-keying.** 58.00 ≠ 60.25 means this
adapter physically cannot be bolted on rotated 180°.

### 40-pin GPIO header

| item | value | source |
|---|---|---|
| pin grid | 20 cols × 2 rows, 2.54 pitch | `CAD`, sectioned above the plastic |
| column x | 8.196 + 2.54·k, k = 0…19 → 8.196 … 56.456 | `CAD` |
| row y | **3.360** and **5.900** | `CAD` |
| **pin numbering** | **pin 1 = (56.456, 5.900)**: the USB-C (high-x) end, on the **inner** row. Odd pins run along y 5.900 toward low x, even pins (5V, 5V, GND …) along the board-edge row y 3.360. Pin 39 = (8.196, 5.900), pin 40 = (8.196, 3.360) | `IMG` + cross-check |
| plastic body | x 6.926…57.726 (=50.800), y 0.860…8.399 | `CAD` |
| pin tip height | z = 10.10, i.e. **9.30 above the PCB** | `CAD` |

Internal consistency check that ties the two datasets together: the pin field is
centred at x 32.326 against the bottom hole pair's centre 32.33, and the rows sit
at 4.63 ± 1.27 — exactly straddling the bottom holes' y. That is the Raspberry Pi
header-to-hole relationship, reproduced by LilyGo.

### Component heights above the PCB — what a stacked board must clear

Heights below are LilyGo CAD z, in which the PCB's **top face is at 0.80**
(the 1.575 slab is centred on z=0). Subtract 0.80 for height above the PCB
surface. The comparison that matters is a difference, so it holds in either
frame.

| part | top (CAD z) | = above PCB | footprint |
|---|---|---|---|
| **RJ45 (with magnetics)** | **15.97** | **15.17** | x −5.18…16.42 (hangs 5.18 off the left edge), y 10.50…28.90 |
| 40-pin header pins | 10.10 | 9.30 | x 6.93…57.73, y 0.86…8.40 |
| two parts at the far edge (BOOT/RST) | 4.00 | 3.20 | x 41.48…45.98 and 52.10…56.60, y 45.85…49.35 |
| ESP32-S3-WROOM-1 | 3.90 | 3.10 | x 40.47…65.97, y 24.07…42.08 |

**The RJ45 is 5.87 mm taller than the header pins.** Anything that plugs onto
that header and spans the left half of the board fouls it unless it is notched
**or** lifted clear of 15.97 altogether.

Stated precisely, because an earlier draft of this file overstated it: a tall
enough stacking header alone would lift a HAT above the RJ45 with no adapter at
all. What that does *not* give you is anywhere to put the HAT's far mounting
pair, which lands at y = 53.63, past the Elite's own 49.19 edge — so the HAT
would hang off two screws at the connector end. On a board carrying a
lever-actuated terminal block that is the difference between a fixture and a
wobble. The notch, the far holes and the BOOT/RST access are what this board
adds; only the last two are things a header cannot do.

## LilyGo's own LoRa shield — the reference design for stacking on this board

| item | value |
|---|---|
| outline | same 66.22 × 49.19, R3 |
| **RJ45 notch** | x 0…16.87, y 11.25…28.45 (cut into the left edge) |
| mounting holes | the same 4 asymmetric holes, within 0.1 of the base board |
| connector | one 2×20 **stacking header**: socket barrel ~13.0 below the shield PCB, pins reaching 9.30 above it — one part, one set of 40 holes |

So LilyGo stack their shields at ~13 mm above the base PCB and clear the RJ45
with a notch. This adapter does the same thing.

## The TSN Lab 10Base-T1S HAT (the board this adapter exists to carry)

Silkscreen on the product photo reads **`10BASE-T1S HAT / LAN8651 / For RPi /
TSNlab+Ethercrafts`** — note **LAN8651**, not the LAN8650 the devicemart listing
states.

Measured off the product photo, scaled by the known 48.26 mm pin-row span:

| item | value | confidence |
|---|---|---|
| mounting holes | a **rectangle**, ratio 0.852 vs the Pi spec's 49/58 = 0.845 | good — it is a standard Pi 58 × 49 pattern |
| board | ≈63 × 55, i.e. a normal-size HAT | photo-derived, ±2 |
| edge inset of holes | ≈3.5 | photo-derived |

Photo-derived numbers are **not** used for anything this board's correctness
depends on. They are used only to justify one decision: that placing the two far
holes on the Pi spec's 49 mm offset is the right call. The listing's "57×75×23"
does not match the photo and is assumed to be packaging.

## Therefore — this adapter

Outline 0…66.22 × 0…**57.20**, R3 corners. Same width as the base board;
extended +8.0 in y past it so it can carry the HAT's far mounting holes, which
land beyond the base board's own edge.

**RJ45 notch: x 0…17.40, y 10.00…29.40** — deliberately ~0.5 larger all round
than the RJ45's measured envelope, rather than a copy of LilyGo's slightly
tighter 16.87/11.25/28.45. The notch region is empty board on this design, so
the extra clearance costs nothing.

### Holes — 6, all Ø2.75 (M2.5 free fit)

| ref | position | serves |
|---|---|---|
| H1 | (3.33, 4.63) | T-ETH-Elite bottom-left **and** HAT near-left |
| H2 | (61.33, 4.63) | T-ETH-Elite bottom-right **and** HAT near-right |
| H3 | (2.98, 46.23) | T-ETH-Elite top-left |
| H4 | (63.23, 46.20) | T-ETH-Elite top-right |
| H5 | (3.33, 53.63) | HAT far-left |
| H6 | (61.33, 53.63) | HAT far-right |

H1/H2 are shared on purpose. Deriving the Pi near-hole positions in this frame
from the header (centre 32.326 ± 29.0, y = the header centre 4.63) gives
(3.326, 4.63) and (61.326, 4.63) — the T-ETH-Elite's own bottom holes to within
**0.005 mm**. They are the same holes; one pair is drilled, not two.

H5/H6 are H1/H2 offset by the Pi spec's +49.0 in y.

### Connector

One 2×20 THT stacking header, pins on the grid above: x = 8.196 + 2.54·k
(k = 0…19), y = 3.360 and 5.900. Plated through-holes with annular rings both
sides — the solder joint is what mechanically anchors the stack.

No traces. The signal path is the stacking header's own pin, straight through
from the socket below to the exposed pin above; the board contributes nothing
electrical and pin-to-pin identity is guaranteed by the part, not by copper.
Every pad is an isolated net by design.

### Stack-up that results

```
   TSN Lab 10Base-T1S HAT (LAN8651)
        ↑ plugs onto the pins protruding above the adapter
   ── this adapter ──                      ≈13 above the Elite PCB
        ↑ stacking-header socket swallows the Elite's 9.30 pins
        ↑ RJ45 (15.97) passes up through the notch
   LilyGO T-ETH-Elite (ESP32-S3)
```

Standoffs: Elite → adapter ≈13 (set by the chosen stacking header's barrel);
adapter → HAT set by that header's above-board pin length. Both are properties
of the header part, so pick the standoffs after picking the header, not before.
