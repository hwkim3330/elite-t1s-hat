# T1S HAT — electrical design spec

Our own 10BASE-T1S board for the LilyGO T-ETH-Elite, built around a
**Microchip LAN8651** MAC-PHY. Replaces buying TSN Lab's HAT; same chip family,
same host, our schematic.

Companion to [`../mechanical/GEOMETRY.md`](../mechanical/GEOMETRY.md), which fixes the
mechanical side and is reused unchanged. This file is the electrical source of
truth: **if the KiCad files disagree with this file, this file is right.**

Every value below is quoted from Microchip DS60001734F (LAN8650/1 data sheet)
or DS60001718D (AN1718, LAN86xx Bus Interface Network reference design). No
value here is inferred or carried over from a similar part.

## Why LAN8651 and not LAN8650

Same pinout except three pins, and those three decide the whole power design:

| pin | LAN8650 | LAN8651 |
|---|---|---|
| 6, 15 | VDDC (+1.8 V core in) | DNC |
| 21 | VDDC (+1.8 V core in) | **CCOMP** |

> "Core LDO Supply Compensation — CCOMP — Internal +1.8 V LDO core
> compensation. This pin requires a 4.7 μF low ESR capacitor to the PCB ground
> plane. This pin is only on the LAN8651." — DS60001734F Table 3-8

**The LAN8651 regulates its own 1.8 V core internally.** The LAN8650 does not:
its reference schematic needs an external 1.8 V LDO plus an SS14 sequencing
diode between the rails. Choosing LAN8651 deletes a regulator, a diode, a rail
and its whole decoupling group, and leaves the board **single-supply 3.3 V**.
TSN Lab's HAT uses LAN8651 too (their device-tree overlay declares
`compatible = "microchip,lan8651"`, and the board silkscreen reads LAN8651 even
though the shop listing says LAN8650).

## Pinout — 32-VQFN 5×5 mm, 0.5 mm pitch, exposed pad = VSS

| # | name | # | name | # | name | # | name |
|---|---|---|---|---|---|---|---|
| 1 | INH | 9 | IRQ_N | 17 | VDDP | 25 | VDDAU |
| 2 | VSS | 10 | SDO | 18 | DIOA0 | 26 | RBIAS |
| 3 | VSS | 11 | CS_N | 19 | DIOA1 | 27 | XTI |
| 4 | TEST | 12 | SCLK | 20 | DIOA2 | 28 | XTO |
| 5 | VSS | 13 | SDI | 21 | **CCOMP** | 29 | VDDA |
| 6 | DNC | 14 | DIOB1 | 22 | DIOA3 | 30 | TRXP |
| 7 | VDDP | 15 | DNC | 23 | DIOA4 | 31 | TRXN |
| 8 | RESET_N | 16 | DIOB0 | 24 | WAKE_OUT | 32 | WAKE_IN |

**Exposed pad must be connected to ground.**

## Mandatory pin handling — quoted, not guessed

| pin | what the data sheet says | so |
|---|---|---|
| TEST (4) | "This pin must be connected to VDDP." | **to 3V3**, not ground. Easy to get backwards. |
| DIOB1 (14) | "reserved for Microchip test use. It is recommended that this pin is connected directly to ground." | to GND |
| DNC (6, 15) | "must be left floating externally" | no connect, no copper stub |
| WAKE_IN (32) | "When not used, this pin should be connected to VSS." | to GND (VDDAU domain) |
| WAKE_OUT (24) | "When not used, this pin should be left unconnected." | NC |
| DIOA0-4, DIOB0 | "When not used, these pins may be connected directly to ground." | see LEDs below; the rest to GND |
| RESET_N (8) | "When not used, this pin must be connected directly to VDDP." | driven from the host **plus** a 10 k pull-up to 3V3, so a floating GPIO during ESP32 boot cannot hold the PHY in reset |
| RBIAS (26) | "requires connection of a 12.4 kΩ resistor to ground… within ±1% across the entire expected operating temperature range" | **12.4 kΩ 1 %**, no substitutions |

## Power — single 3.3 V rail

3V3 comes from the 40-pin header (pins 1 and 17). No other rail on the board.

| net | pins |
|---|---|
| VDDP | 7, 17 |
| VDDA | 29 |
| VDDAU | 25 (must stay powered in sleep — here it is simply always on) |

Decoupling, per the data sheet's own layout rule — *"Place one 0.01 µF and one
0.1 µF at each power pin. The 0.01 µF must be closest to the pin."*:

- **each** of VDDP 7, VDDP 17, VDDA 29, VDDAU 25 → 0.1 µF + 0.01 µF
- one 10 µF bulk near the device, on the supply side
- **CCOMP (21) → 4.7 µF low-ESR** to ground. Required, not optional. The data
  sheet says "low ESR capacitor"; Microchip's Hardware Design Checklist
  (DS60001746D) says "low ESR **metal film**". A 4.7 µF metal-film part is not a
  practical 0805, so this board uses an **X7R** MLCC (Yageo CC0805KKX7R8BB475):
  low ESR, stable over temperature, and at 1.8 V with no meaningful DC-bias loss.
  If Microchip confirms the film requirement, C4 is the one part to change. 0.1 µF and
  0.01 µF alongside are "useful but not required" — include the 0.1 µF.
- Ideally two vias per decoupling cap to the plane, and caps must not share vias.

**If a real ferrite bead is fitted,** the checklist requires a local ~10 µF bulk
capacitor between the bead and the pin's decoupling. The board has no footprint
for that, so FB1–FB4 are 0 Ω links in practice; with 0 Ω links a single 10 µF on
the 3.3 V supply (C6) is exactly what the checklist asks for.

Optional, footprint-only: ferrite beads (~300 Ω @ 100 MHz, e.g. Würth
742792640, DC rating ≥2× the pin current) to island VDDA (FB1) / VDDAU (FB2) /
VDDP pin 7 (FB3) / VDDP pin 17 (FB4). The
data sheet recommends carrying the option through prototype. Populate 0 Ω.

## Clock

25.0 MHz crystal, fundamental, parallel resonant, across XTI (27) / XTO (28),
load caps **C1 = C2 = 18 pF** (data sheet allows 10–22 pF).

> "external series resistors should not be used… The device contains an internal
> ~1 MΩ resistance in parallel with the crystal amplifier… therefore an external
> resistance between XTO and XTI should not be used."

So: no series resistor, no feedback resistor. Oscillator margin should be >10
and at least 5 — a crystal with low ESR and the right CL matters here.

### The fitted part (added 2026-09-28, after LCSC sourcing)

**Y1 = YXC X322525MOB4SI, LCSC C9006**: 25.000 MHz, **CL 12 pF**, ESR 50 Ω
max, ±10 ppm at 25 °C, ±20 ppm over −40…+85 °C, SMD3225 **4-pad** (1/3
crystal, 2/4 lid → GND). JLCPCB basic part, ~74 k in stock.

Two corrections to what the board had before:

- **Package.** The first layout used a 3.2 × 1.5 mm 2-pad land. No 25 MHz
  crystal is made in that package (3215 is the 32.768 kHz watch-crystal size),
  so it matched no orderable part. The land is now KiCad's
  `Crystal_SMD_3225-4Pin_3.2x2.5mm`.
- **CL 12 pF, with the 18 pF load caps kept.** The load a crystal sees is
  C1·C2/(C1+C2) + C_stray = 18/2 + ~3 pF ≈ **12 pF**. A CL 18 pF crystal on
  18 pF caps would therefore run fast; it would need ~30 pF caps instead. The
  data sheet's C1 = C2 = 18 pF figure is kept, and the crystal is chosen to
  match it.

**Oscillator margin, as far as it can be computed without the amplifier's
gm.** The critical transconductance for this crystal is

  gm_crit = 4 · ESR · (2πf)² · (C0 + CL)²
          = 4 · 50 Ω · (2π · 25 MHz)² · (3 pF + 12 pF)² ≈ **1.1 mA/V**

where C0 ≤ 3 pF is taken from YXC's YSX321SL family sheet. A margin of >10
therefore needs the LAN8651's oscillator gm to be ≥ 11 mA/V. **Microchip does
not publish that gm**, so the margin cannot be closed on paper. For comparison,
a CL 18 part would need about twice as much (2.2 mA/V). Measure it on the
first board: add series resistance until oscillation stops. The resistance at
which it stops, divided by ESR, is the margin.

**Frequency error, estimated.** ±10 ppm tolerance + ±20 ppm over temperature,
plus pulling from the stray-capacitance guess. At ~11 ppm per pF of load error
for a typical 5 fF motional capacitance, a ±2 pF error in the 3 pF guess
costs ±22 ppm. The worst case is ≈ 52 ppm, inside Ethernet's ±100 ppm.

## Host interface — matches TSN Lab's device-tree overlay on purpose

Their overlay (`lan8650-overlay-rpi4.dts`) declares `spi0`, `reg = <0>` (CE0),
`interrupts = <23 …>`. Wiring to the same header pins means their SDK and the
mainline `microchip,lan8651` driver both work unmodified if this board is ever
put on a Pi, and costs nothing on the ESP32 side.

| LAN8651 | header pin | RPi name | ESP32-S3 on the Elite |
|---|---|---|---|
| SDI (13) | 19 | GPIO10 / SPI0 MOSI | **IO11** SPI_MOSI (shared with the TF card) |
| SDO (10) | 21 | GPIO9 / SPI0 MISO | **IO9** SPI_MISO (shared with the TF card) |
| SCLK (12) | 23 | GPIO11 / SPI0 SCLK | **IO10** SPI_SCLK (shared with the TF card) |
| CS_N (11) | 24 | GPIO8 / SPI0 CE0 | **IO0 = BOOT strap** — see below; 10 k pull-up to 3V3 |
| IRQ_N (9) | 16 | GPIO23 | **IO39**; 10 k pull-up to 3V3 |
| RESET_N (8) | 15 | GPIO22 | **IO42**; 10 k pull-up to 3V3 |

SPI clock up to 25 MHz. Protocol is OPEN Alliance TC6.

The ESP32 column is read off LilyGo's own schematic (`schematic/T-ETH-ELite.pdf`
in `Xinyuan-LilyGO/LilyGO-T-ETH-Series`, sheet 2, connector J1), and it is
mirrored in [`firmware/t1s_node/pins.h`](firmware/t1s_node/pins.h). Two
consequences of it:

- **CS_N lands on IO0, the ESP32-S3's BOOT strap.** That is where a Pi puts
  CE0 and it works: the strap wants IO0 high at reset, both the Elite (its R1)
  and this board (R5) pull it up, and CS_N is an input on the LAN8651, so
  nothing here can drag it low. But pressing the Elite's BOOT button on a
  running node asserts CS_N mid-transaction. The driver catches that as a
  parity error; do not do it on a live bus. Flashing is unaffected.
- **MOSI/MISO/SCLK are the Elite's general SPI, shared with its TF card**
  (card CS = IO12). Firmware holds IO12 high. The Elite's W5500 is on a
  different bus (IO47/21/48/45), so the two Ethernet interfaces can run
  together.

We deliberately do **not** copy TSN Lab's FXL6408 I²C GPIO expander (their
GPIO24 interrupt) or their rotary node-ID switch: the PLCA node ID is a
register write, so it belongs in firmware, and dropping the expander removes a
part, an I²C bus and an interrupt line.

## Bus Interface Network — TRXP (30) / TRXN (31) to the cable

Order from the chip outward, per AN1718:

```
TRXP/TRXN ─ CMC(L1) ─ C1/C2 100nF ─ [R1/R2 termination] ─ [ESD] ─ CN1
```

| ref | value | note |
|---|---|---|
| L1 | common-mode choke, 130 µH @ 100 kHz — TDK ACT1210D-131-2P-TL00 (or ACT1210E-241 / Murata DLW32MH241MX2) | **always populated** |
| C1, C2 | 0.1 µF, 100 V, 0805 | DC block / galvanic isolation. **Always populated, on every node.** |
| R1, R2 | **49.9 Ω 1 % 1 W 1206** for an END-OF-BUS node<br>**1.5 kΩ 1 % 1206** for an interior DROP node | footprints fitted, **DNP by default** — stuff per where this node sits on the bus |
| R3 | 100 kΩ 5 % 0805 + C3 | common-mode termination (drop node), per AN1718's drop-node topology |
| MOV1/2 | TDK AVRH10C221KT1R5YA8 or Panasonic EZA-EG3W11AV | ESD, optional, footprint by the connector |
| CN1 | 4-pin 3.81 mm pluggable terminal block | P_in/N_in + P_out/N_out, the two P's and the two N's shorted on board so the node taps a daisy chain |

**No pin header on the bus.** AN1718 wants ESD and termination in-line with
stubs minimised; a jumper header on T1S is a stub. Termination is selected by
which resistors get stuffed, marked on silkscreen, not by a removable jumper.

## Status LEDs

Two, on DIOA0 (18) and DIOA1 (19) — configurable outputs in the VDDP domain, so
1 kΩ series to 3V3. Firmware maps them to PLCA status / activity. Remaining
DIOA2/3/4 and DIOB0 go to ground.

## Board rules

- **4-layer, 1.6 mm, JLCPCB JLC04161H-7628 (Rev B, 2026-09-28).**
  F.Cu signals and parts / In1.Cu solid GND / In2.Cu +3V3 plane / B.Cu the
  three SPI escapes that cannot stay on top, plus a GND pour. AN1718's 50 Ω
  single-ended TRXP/TRXN is **met by construction**: over 0.2104 mm of 7628
  prepreg (εr 4.4), a 0.35 mm microstrip is 50.1 Ω (IPC-2141), and every bus
  trace from the choke to CN1 is 0.35 mm, asserted by the generator. Rev A
  was 2-layer and argued its way out of this; that argument is in git history.
  It was not worth the cost: an 84 mm VDDP detour, a 0 Ω jumper, a signal
  running between resistor pads, and one via per decoupling cap.
- **In1.Cu carries no tracks at all** (asserted). It is the reference for
  every F.Cu signal, and the T1S pair runs over it unbroken except at the
  choke void AN1718 asks for.
- **Every 3V3 connection is a via into In2.Cu** beside its pad. There is no
  3V3 trace anywhere on the board.
- **Each VDDP pin is its own ferrite-option island** (pin 7 + TEST via FB3,
  pin 17 via FB4), with its 0.01 µF and 0.1 µF directly at the pin.
- **TRXP/TRXN short, length-matched, symmetric, no vias.** Let this drive
  placement: the LAN8651 goes near the bus connector, ahead of tidiness.
- **All-layer void under the common-mode choke**, per AN1718, to stop coupling
  from its core.
- ESD parts and coupling caps close to the connector; minimise stubs on the bus.
- Exposed pad stitched to the ground plane with a via array.
- **Antenna keepout:** the Elite's ESP32-S3-WROOM antenna sits under this board
  at x 58…66.22, y 22…44 (its own PCB is cut away at x 59.84…66.19,
  y 23.93…42.22). A ground plane above a PCB antenna detunes it, so **no copper
  on any layer** in that window: all four layers, planes included, are cut
  there by one rule area, and the generator checks every filled polygon.

## Mechanical — inherited, with one deliberate departure

From [`../mechanical/GEOMETRY.md`](../mechanical/GEOMETRY.md), unchanged:

- outline 66.22 × 49.19, R3 corners
- four asymmetric mounting holes (3.33, 4.63) (61.33, 4.63) (2.98, 46.23)
  (63.23, 46.20) — 58.00 apart at the bottom, 60.25 at the top, self-keying
- 2×20 header grid, x = 8.196 + 2.54·k (k = 0…19), y = 3.360 and 5.900

**Departure: no RJ45 notch. The outline is a plain rectangle.** GEOMETRY.md's
notch exists for a board sitting ~13 mm up, which would foul the Elite's
15.97 mm RJ45. This board is instead lifted clear of the jack on a taller
riser, so nothing has to be cut away — and that matters more than convenience
here, because the notch was the largest interruption in the bottom-side ground
pour, right where the T1S pair and its choke want a clean reference.

### The riser is two-stage, and this is easy to get wrong

The Elite's header pins are only **9.30 mm** tall. A single tall stacking
header does *not* work: its contacts sit near the top of the barrel, so with a
~17 mm barrel the Elite's pins reach barely halfway and never touch. What
works is two stages — a 2×20 stacking riser onto the Elite's pins, presenting
longer pins, then this board's socket onto those. Put that in the BOM notes,
not just here.

Resulting height above the Elite PCB: **≈23 mm (calculated)**, set by the riser. Nothing
in the design may depend on an exact figure.

The Elite carries an on-board PoE front end — RJ45 `HY931147C` tapping
POWER+/− into an **SDaPo DP9900M-5V** PD module (5 V, 9 W @ 70 °C), with a
polyfuse, an SMAJ58A TVS and 4.7 nF/2 kV isolation caps. That module is **not
in LilyGo's 3D model** (it does not appear among the parts over 3.5 mm) and
SDaPo's datasheet carries no dimensioned drawing, so CAD cannot settle its
height. Reported from the physical board: **it stands about level with the
RJ45**, so the RJ45's measured 15.97 mm remains the height to clear and the
riser covers both. Source is direct observation of the real hardware, not a
document — if a future board revision changes the PD module, re-check it.

### Bottom side is usable

At ~23 mm the underside of this board has 14+ mm of clearance everywhere
except over the RJ45 footprint (x −5.18…16.42, y 10.50…28.90), where it is
much less. Prefer the bottom for anything bulky. A later radar variant will
want that space.

## Not decided here

- Radar variant (A121 / XM125) — a derivative of this board, specified
  separately once this one links.
- PLCA node count, node IDs, coordinator election: firmware, not hardware.
