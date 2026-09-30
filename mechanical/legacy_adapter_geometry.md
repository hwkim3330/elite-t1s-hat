# Legacy: the Pi-HAT adapter's geometry

**Not this board.** Kept from the earlier adapter project (a carrier that put a
Raspberry-Pi-format HAT on the Elite) for the record. Its 57.20 mm outline, RJ45
notch and six holes are *not* the T1S HAT's; for that, see
[`GEOMETRY.md`](GEOMETRY.md), whose Elite measurements both designs share.

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
