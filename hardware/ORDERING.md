# Ordering the T1S HAT at JLCPCB

What to upload, what to tick, and the three things to look at before paying.
Stock figures are from JLCPCB's part search on **2026-09-28**.

## 1. PCB

Upload **`gerbers/t1s_hat_gerbers.zip`** (10 gerber layers, 4 of them copper,
plus separate PTH/NPTH Excellon and a job file).

| option | value | why |
|---|---|---|
| Layers | **4** | Rev B. Signal / GND / 3V3 / signal, see `ELECTRICAL.md` "Board rules" |
| Stack-up | **JLC04161H-7628** | the one the board file is built for: 0.2104 mm prepreg to each plane |
| Impedance control | **yes**, 50 Ω single-ended on L1 ref L2, 0.35 mm | the T1S pair (CMC_P/N, BUS_P/N). Optional; without it the pair still sits at ≈50 Ω by design, JLC just doesn't test it |
| Dimensions | 66.22 × 49.19 mm (JLC reads it from Edge.Cuts) | |
| Thickness | **1.6 mm** | |
| Material | FR-4 | |
| Outer / inner copper | 1 oz / 0.5 oz | JLC's default for that stack-up |
| Min track / space | 0.15 / 0.15 mm | the generator's rules; inside JLC standard |
| Min hole | 0.20 mm | U1's exposed-pad thermal vias |
| Surface finish | **ENIG** recommended, lead-free HASL works | 0.5 mm-pitch QFN and 0402 lands sit flatter on ENIG |
| Via covering | tented | |
| Remove order number | "specify a location" (no free spot; or pay for removal) | |

## 2. Assembly (PCBA), top side only

Upload **`jlcpcb/bom_jlc.csv`** and **`jlcpcb/cpl_jlc.csv`**. Both are written
by `make_t1s_hat.py` in JLCPCB's own column names. Only fitted parts are in
them: DNP parts and J1 are left out on purpose (see section 3).

| ref | part | LCSC | JLC class | stock |
|---|---|---|---|---|
| U1 | Microchip LAN8651B1-E/LMX, VQFN-32 5×5 | C22386973 | extended | **~200: check first** |
| Y1 | YXC X322525MOB4SI 25 MHz CL12 3225-4P | C9006 | basic | 74 k |
| L1 | TDK ACT1210L-201-2P-TL00 CMC | C131444 | extended | 4 k |
| CN1 | Phoenix 1803293 MC 1,5/4-G-3,81 (THT) | C480536 | extended | 2.4 k |
| R7 | 12k4 1 % 0603 | C22865 | extended (no basic exists) | 228 k |
| D1 | green LED 0603 | C12624 | extended | 353 k |
| D2 | yellow LED 0603 | C89811 | preferred ext. | 41 k |
| C1–C3 | 100 nF 100 V X7R 0805 | C28233 | basic | |
| C4 | 4.7 µF 25 V **X7R** 0805 (CCOMP) — **PROTOTYPE_X7R**: Microchip asks for metal film, see ELECTRICAL.md | C354262 | extended | 581 k |
| C6 | 10 µF 25 V X5R 0805 | C15850 | basic | |
| C5 C7 C9 C11 C13 | 100 nF 0603 | C14663 | basic | |
| C8 C10 C12 C14 | 10 nF 0603 | C57112 | basic | |
| C15 C16 | 18 pF C0G 0603 | C1647 | basic | |
| R3 | 100 k 0805 | C149504 | basic | |
| R4–R6 | 10 k 0603 | C25804 | basic | |
| R8 R9 | 1 k 0603 | C21190 | basic | |
| FB1–FB4 | 0 Ω 0603 (ferrite-bead option) | C21189 | basic | |

7 extended parts, so expect 7 extended-part setup fees. CN1 is through-hole:
either let JLC fit it (THT assembly fee) or delete its line and hand-solder it.

**Before confirming, look at JLC's placement preview for these, because a
rotation convention that differs between KiCad and JLC's reel data is the
usual way a board comes back wrong:**

1. **U1 pin 1.** The silkscreen dot/chamfer on the QFN has to match the
   part's pin-1 mark. If it is off by 90° or 180°, fix it in the preview (JLC
   lets you rotate there).
2. **D1, D2 polarity.** Cathode is pad 1, which goes to the LAN8651's DIOA
   pin; the anode goes to R8/R9 and 3V3.
3. **Y1.** Pads 1 and 3 have to be the crystal and 2/4 the lid. The 3225-4P
   orientation is symmetric, so a 90° error would put the crystal across the
   lid pads.

## 3. Not assembled by JLC: order separately and hand-solder

| what | part | note |
|---|---|---|
| J1: 2×20 female socket, 2.54 mm | BOOMELE 2.54-2*20P, LCSC **C5124634** (8.5 mm body, 3.1 mm tail) | **The body goes on the BOTTOM of this board**, facing the riser; solder from the top. This is why it is not in the CPL: a top-side placement would be upside down in the stack. |
| Riser: 2×20 stacking header (PC104 style) | BOOMELE, LCSC **C35165** (8.3 mm body, 12.3 mm pins) | Plugs onto the Elite's own pins (female side down); its long pins go up into J1. |
| Bus plug for CN1 | Phoenix 1803594, LCSC **C480512** (low stock, 128), or clone Kefa KF2EDGK-3.81-4P **C440860** | the clone mates with the Phoenix header |
| M2.5 standoffs × 4 | length = measured stack height | see below |

**Stack height, calculated, not measured:** riser body 8.3 mm + the ~6 mm of
its pins that J1 swallows + J1's 8.5 mm body ≈ **23 mm** from the Elite's PCB
to this board's underside. That is more than the 18–20 mm the README first
assumed, and nothing on the board depends on it. If you want it lower, trim
the riser pins. Buy standoffs after the stack is assembled and measured.

## 4. Termination: decide before ordering, or leave DNP and hand-fit

R1/R2 ship unfitted. Where the node sits on the bus decides them:

| node position | R1, R2 | LCSC |
|---|---|---|
| **end of the bus** (a 2-node bench is two end nodes) | 49R9 1 % 1206, ≥0.5 W | C4014562 (0.75 W) |
| interior drop node | 1K5 1 % 1206 | C26030 |

The data sheet's "1 W" 49R9 exists in 1206 (Susumu HRG3216P, Vishay PHP01206)
but had **zero stock** anywhere at JLC on 2026-09-28. The 0.75 W part is what
can be bought. Dissipation at 10BASE-T1S levels is milliwatts, so 1 W is
margin, not need.

To have JLC fit them for an end-node batch, add a line to `bom_jlc.csv`
(`49R9,"R1,R2",R_1206_3216Metric,C4014562`). Their positions are already in
`cpl.csv`; copy the R1/R2 lines into `cpl_jlc.csv` in the same format.

## 5. For an 8-node bus

Eight nodes on one bus means **two end nodes (49R9) and six drop nodes
(1K5, per `ELECTRICAL.md`)**. Order 10 boards and fit R1/R2 by hand. That is simpler than two
BOM variants and leaves spares. At ~200 in stock, U1 is the only part that
limits the order quantity.
