#!/usr/bin/env python3
"""
Generate the T1S HAT: symbol library, schematic, PCB, and all fab outputs.

Run:  python3 make_t1s_hat.py      (KiCad 7 pcbnew Python bindings required)

Sources of truth, in this order:
  ../mechanical/GEOMETRY.md  mechanical   (outline, holes, 2x20 grid) -- inherited
  ELECTRICAL.md        electrical   (every part, value and pin rule)

Nothing in either file is re-derived here.  Deviations from them that were
directed by the project owner after those files were written are marked
"DIRECTED:" in the comments and are listed in README.md.

COORDINATE FRAME -- read before touching a number
-------------------------------------------------
GEOMETRY.md's frame ("G frame"): T-ETH-Elite top view, origin at the Elite's
PCB bottom-left corner, +x right, +y UP.  KiCad's PCB frame has +y DOWN.
The single conversion is kx()/ky()/V(); the G origin sits at KiCad (100,100).
Same convention as ../pcb/make_board.py, from which the mechanical code here
is reused.
"""

import csv
import json
import math
import os
import re
import subprocess
import sys
import uuid as _uuid
import zipfile

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
NAME = "t1s_hat"
PCB_PATH = os.path.join(HERE, NAME + ".kicad_pcb")
SCH_PATH = os.path.join(HERE, NAME + ".kicad_sch")
PRO_PATH = os.path.join(HERE, NAME + ".kicad_pro")
SYMDIR = os.path.join(HERE, "sym")
SYMLIB = os.path.join(SYMDIR, "t1s_hat.kicad_sym")
FPDIR = os.path.join(HERE, "fp", "t1s_hat.pretty")

KSYM = "/usr/share/kicad/symbols/"
KFP = "/usr/share/kicad/footprints/"


def fp_dir(lib):
    """Directory holding footprint library `lib`.  Project-local parts live in
    fp/t1s_hat.pretty; everything else comes from the KiCad install."""
    return FPDIR if lib == "t1s_hat" else KFP + lib + ".pretty"

# ---------------------------------------------------------------- frame ----
ORIGIN_X = 100.0
ORIGIN_Y = 100.0


def kx(gx):
    return ORIGIN_X + gx


def ky(gy):
    return ORIGIN_Y - gy


def V(gx, gy):
    return pcbnew.VECTOR2I(pcbnew.FromMM(kx(gx)), pcbnew.FromMM(ky(gy)))


def MM(v):
    return pcbnew.FromMM(v)


def SZ(w, h):
    return pcbnew.VECTOR2I(pcbnew.FromMM(w), pcbnew.FromMM(h))


def gof(pt):
    """KiCad VECTOR2I -> G-frame (x, y) in mm."""
    return (pcbnew.ToMM(pt.x) - ORIGIN_X, ORIGIN_Y - pcbnew.ToMM(pt.y))


# ------------------------------------------------- GEOMETRY.md mechanical --
# DIRECTED: the RJ45 notch is dropped; this board is lifted clear of the jack
# by a taller riser instead, so the outline is the plain rectangle.  Every
# other mechanical number below is GEOMETRY.md's, unchanged.
BOARD_W = 66.22
BOARD_H = 49.19
CORNER_R = 3.0
BOARD_T = 1.6

HOLE_D = 2.75
HOLES = [
    ("H1", 3.33, 4.63, "T-ETH-Elite bottom-left"),
    ("H2", 61.33, 4.63, "T-ETH-Elite bottom-right"),
    ("H3", 2.98, 46.23, "T-ETH-Elite top-left"),
    ("H4", 63.23, 46.20, "T-ETH-Elite top-right"),
]

# Pin NUMBERING on the grid, corrected 2026-09-28: pin 1 is at the USB-C end
# (high x) and the odd pins are on the INNER row (y 5.900), the even pins
# (5V, 5V, GND, ...) on the board-edge row -- Raspberry Pi convention.  Source:
# LilyGo's own pin-map image of the Elite (top view: 3V3 at the USB-C end, inner
# column), cross-checked against GEOMETRY.md's asymmetric hole pattern, which
# puts USB-C at high x.  Rev A/B had pin 1 at the opposite corner, i.e. the
# whole header rotated 180 deg -- HAT GND on the Elite's 5 V.
HDR_X_PIN1 = 56.456
HDR_PITCH = 2.54
HDR_ROW_ODD = 5.900
HDR_ROW_EVEN = 3.360


def pin_xy(pin):
    """G-frame (x, y) of 40-pin header pin `pin` (1..40)."""
    k = (pin - 1) // 2
    return HDR_X_PIN1 - HDR_PITCH * k, (HDR_ROW_ODD if pin % 2 else HDR_ROW_EVEN)


# ELECTRICAL.md: the Elite's WROOM antenna sits under this board here.
KEEPOUT = (58.0, 22.0, 66.22, 44.0)       # x0, y0, x1, y1

EDGE_W = 0.10
SILK_W = 0.15


# ===========================================================================
#  THE CIRCUIT -- one table, used for the symbol library, the schematic and
#  the PCB, so the three cannot disagree.
# ===========================================================================

# LAN8651, 32-VQFN 5x5, exposed pad = VSS.  Pin table verbatim from
# ELECTRICAL.md.  Columns: number, name, electrical type, symbol side.
#   side: L/R/T/B on the schematic symbol body.
LAN8651_PINS = [
    # --- host SPI + reset, left of the symbol -----------------------------
    (11, "CS_N",     "input",          "L", 22.86),
    (12, "SCLK",     "input",          "L", 20.32),
    (13, "SDI",      "input",          "L", 17.78),
    (10, "SDO",      "output",         "L", 15.24),
    (9,  "IRQ_N",    "open_collector", "L", 12.70),
    (8,  "RESET_N",  "input",          "L",  7.62),
    # --- config / analogue, left ------------------------------------------
    (4,  "TEST",     "input",          "L",  2.54),
    (27, "XTI",      "passive",        "L", -2.54),
    (28, "XTO",      "output",         "L", -5.08),
    (26, "RBIAS",    "passive",        "L", -10.16),
    (21, "CCOMP",    "passive",        "L", -12.70),
    # --- bus, right --------------------------------------------------------
    (30, "TRXP",     "bidirectional",  "R", 22.86),
    (31, "TRXN",     "bidirectional",  "R", 20.32),
    # --- configurable IO, right -------------------------------------------
    (18, "DIOA0",    "bidirectional",  "R", 15.24),
    (19, "DIOA1",    "bidirectional",  "R", 12.70),
    (20, "DIOA2",    "bidirectional",  "R", 10.16),
    (22, "DIOA3",    "bidirectional",  "R",  7.62),
    (23, "DIOA4",    "bidirectional",  "R",  5.08),
    (16, "DIOB0",    "bidirectional",  "R",  2.54),
    (14, "DIOB1",    "bidirectional",  "R",  0.00),
    # --- power management / unused, right ---------------------------------
    (32, "WAKE_IN",  "input",          "R", -5.08),
    (24, "WAKE_OUT", "output",         "R", -7.62),
    (1,  "INH",      "open_collector", "R", -10.16),
    (6,  "DNC",      "passive",        "R", -15.24),
    (15, "DNC",      "passive",        "R", -17.78),
    # --- supplies, top -----------------------------------------------------
    (7,  "VDDP",     "power_in",       "T", -7.62),
    (17, "VDDP",     "power_in",       "T", -2.54),
    (29, "VDDA",     "power_in",       "T",  2.54),
    (25, "VDDAU",    "power_in",       "T",  7.62),
    # --- grounds, bottom ---------------------------------------------------
    (2,  "VSS",      "power_in",       "B", -7.62),
    (3,  "VSS",      "power_in",       "B", -2.54),
    (5,  "VSS",      "power_in",       "B",  2.54),
    (33, "VSS_EP",   "power_in",       "B",  7.62),
]

# The 40-pin host header.  Pin electrical types say who drives what: this is
# the ESP32-S3 side, so the six signals ELECTRICAL.md assigns are declared
# output/input accordingly and the rest are passive.  Names are the Raspberry
# Pi names, which is what ELECTRICAL.md's host-interface table uses.
RPI_NAMES = {
    1: "3V3", 2: "5V", 3: "GPIO2/SDA1", 4: "5V", 5: "GPIO3/SCL1", 6: "GND",
    7: "GPIO4", 8: "GPIO14/TXD", 9: "GND", 10: "GPIO15/RXD", 11: "GPIO17",
    12: "GPIO18", 13: "GPIO27", 14: "GND", 15: "GPIO22", 16: "GPIO23",
    17: "3V3", 18: "GPIO24", 19: "GPIO10/MOSI", 20: "GND", 21: "GPIO9/MISO",
    22: "GPIO25", 23: "GPIO11/SCLK", 24: "GPIO8/CE0", 25: "GND",
    26: "GPIO7/CE1", 27: "ID_SD", 28: "ID_SC", 29: "GPIO5", 30: "GND",
    31: "GPIO6", 32: "GPIO12", 33: "GPIO13", 34: "GND", 35: "GPIO19",
    36: "GPIO16", 37: "GPIO26", 38: "GPIO20", 39: "GND", 40: "GPIO21",
}
HDR_DRIVEN = {15: "output", 16: "input", 19: "output", 21: "input",
              23: "output", 24: "output"}
HDR_GND = [6, 9, 14, 20, 25, 30, 34, 39]
HDR_3V3 = [1, 17]

# Net attached to each header pin; None => no-connect.
HDR_NET = {}
for _p in range(1, 41):
    HDR_NET[_p] = None
for _p in HDR_GND:
    HDR_NET[_p] = "GND"
for _p in HDR_3V3:
    HDR_NET[_p] = "+3V3"
HDR_NET.update({15: "RESET_N", 16: "IRQ_N", 19: "SPI_MOSI",
                21: "SPI_MISO", 23: "SPI_SCLK", 24: "SPI_CS_N"})

# LAN8651 pin -> net; None => no-connect (no copper stub either).
U1_NET = {
    1: None,            # INH   DS60001734F Tbl 3-5: "left unconnected" if unused
    2: "GND", 3: "GND", 5: "GND",
    4: "VDDP",          # TEST  "must be connected to VDDP"
    6: None, 15: None,  # DNC   "must be left floating externally"
    # Rev B: each VDDP pin is its own ferrite-option island fed from the
    # +3V3 plane -- pin 7 (+TEST) through FB3, pin 17 through FB4 -- with its
    # 0.1 + 0.01 uF right at the pin.  Rev A had to join them with a 0 ohm
    # jumper and an 84 mm lap of the board; four layers make that pointless.
    7: "VDDP", 17: "VDDP_17",
    8: "RESET_N", 9: "IRQ_N", 10: "SPI_MISO", 11: "SPI_CS_N",
    12: "SPI_SCLK", 13: "SPI_MOSI",
    14: "GND", 16: "GND",          # DIOB1 "connected directly to ground"
    18: "LED0_K", 19: "LED1_K",
    20: "GND", 22: "GND", 23: "GND",   # DIOA2/3/4 unused -> ground
    21: "CCOMP",
    24: None,           # WAKE_OUT "should be left unconnected"
    25: "VDDAU", 26: "RBIAS", 27: "XTI", 28: "XTO", 29: "VDDA",
    30: "TRXP", 31: "TRXN",
    32: "GND",          # WAKE_IN  "should be connected to VSS"
    33: "GND",          # exposed pad
}

FP_C0603 = "Capacitor_SMD:C_0603_1608Metric"
FP_C0805 = "Capacitor_SMD:C_0805_2012Metric"
FP_R0603 = "Resistor_SMD:R_0603_1608Metric"
FP_R0805 = "Resistor_SMD:R_0805_2012Metric"
FP_R1206 = "Resistor_SMD:R_1206_3216Metric"
FP_L0603 = "Inductor_SMD:L_0603_1608Metric"
FP_LED = "LED_SMD:LED_0603_1608Metric"
FP_MOV = "t1s_hat:Varistor_TDK_AVRH10_1005"

# ---------------------------------------------------------------------------
# PARTS.  Each entry:
#   ref, lib_id, value, footprint, {pin: net}, (pcb_x, pcb_y, pcb_rot),
#   (sch_x, sch_y), dnp, description, lcsc
# pcb_rot is in KiCad degrees and is checked by reading the pads back.
# ---------------------------------------------------------------------------
P = []


def part(ref, lib_id, value, fp, nets, pcb, sch, dnp=False, descr="", lcsc=""):
    P.append(dict(ref=ref, lib_id=lib_id, value=value, fp=fp, nets=nets,
                  pcb=pcb, sch=sch, dnp=dnp, descr=descr, lcsc=lcsc))


part("U1", "t1s_hat:LAN8651",
     "LAN8651",
     "Package_DFN_QFN:TQFN-32-1EP_5x5mm_P0.5mm_EP3.4x3.4mm_ThermalVias",
     U1_NET, (30.0, 20.0, 0), (175.0, 130.0),
     descr="Microchip LAN8651 10BASE-T1S MAC-PHY, OPEN Alliance TC6 SPI, "
           "32-VQFN 5x5 (internal 1.8 V core LDO -> single 3.3 V rail)")

part("J1", "t1s_hat:HAT_HEADER_2x20",
     "2x20 socket (riser stack)",
     "Connector_PinHeader_2.54mm:PinHeader_2x20_P2.54mm_Vertical",
     HDR_NET, (0, 0, 0), (60.0, 120.0),
     descr="2x20 2.54 mm THT socket; mates with the riser that plugs onto "
           "the T-ETH-Elite's own 9.30 mm header pins")

# --- bus interface network, AN1718 order: CMC - caps - term - ESD - CN1 -----
# L1: TDK ACT1210L-201-2P-TL00.  AN1718 names ACT1210D-131 / ACT1210E-241 /
# Murata DLW32MH241MX2 as EXAMPLES ("exact part numbers ... must be determined
# by the customer's unique application"), all 130...240 uH @ 100 kHz in a
# 3.2 x 2.5 mm package.  The -201 is the same ACT1210 package at 200 uH, inside
# that span, AEC-Q200, and -- unlike the three examples -- stocked at LCSC
# (C131444), which is where this board gets built.  Windings are 1-4 and 2-3.
part("L1", "Device:L_Coupled_1423", "200uH @ 100kHz CMC",
     "t1s_hat:L_CommonMode_TDK_ACT1210_3225",
     {1: "TRXP", 2: "TRXN", 3: "CMC_N", 4: "CMC_P"},
     (28.5, 26.6, 90), (250.0, 100.0), lcsc="C131444",
     descr="Common-mode choke, 2-line, 200 uH @ 100 kHz, EIA 1210 "
           "(3.2x2.5 mm), AEC-Q200 - TDK ACT1210L-201-2P-TL00 (AN1718 L1). "
           "ALWAYS FITTED.")
part("C1", "Device:C", "100nF/100V", FP_C0805,
     {1: "CMC_P", 2: "BUS_P"}, (29.5, 30.8, 90), (250.0, 118.0),
     descr="AC coupling / galvanic isolation, bus P. ALWAYS FITTED (AN1718)")
part("C2", "Device:C", "100nF/100V", FP_C0805,
     {1: "CMC_N", 2: "BUS_N"}, (27.5, 30.8, 90), (250.0, 136.0),
     descr="AC coupling / galvanic isolation, bus N. ALWAYS FITTED (AN1718)")
part("R1", "Device:R", "49R9 1% (DNP)", FP_R1206,
     {1: "BUS_P", 2: "BUS_CT"}, (32, 34.2, 180), (288.0, 112.0), dnp=True,
     descr="Bus termination, P leg. 49R9 1% for an END node (AN1718: 1 W; the orderable part is 0.75 W, see ORDERING.md), 1K5 1% for "
           "a DROP node. Shipped unstuffed - see README.")
part("R2", "Device:R", "49R9 1% (DNP)", FP_R1206,
     {1: "BUS_N", 2: "BUS_CT"}, (25, 34.2, 0), (288.0, 142.0), dnp=True,
     descr="Bus termination, N leg. 49R9 1% for an END node (AN1718: 1 W; the orderable part is 0.75 W, see ORDERING.md), 1K5 1% for "
           "a DROP node. Shipped unstuffed - see README.")
part("R3", "Device:R", "100k 5%", FP_R0805,
     {1: "BUS_CT", 2: "GND"}, (30.9, 37, 0), (300.0, 127.0),
     descr="Common-mode termination, centre tap to ground (AN1718 R3)")
part("C3", "Device:C", "100nF/100V", FP_C0805,
     {1: "BUS_CT", 2: "GND"}, (26.1, 37, 180), (312.0, 127.0),
     descr="Common-mode termination, centre tap to ground (AN1718 C3)")
# MOV1/MOV2: the part ELECTRICAL.md already names, TDK AVRH10C221KT1R5YA8 --
# a 1005 [0402] chip varistor, 220 V V1mA, 1.5 pF, 25 kV IEC61000-4-2, not the
# 1206 land this board used to infer.  LCSC C2157827.
part("MOV1", "Device:Varistor", "220V(1mA) 1.5pF ESD (DNP)", FP_MOV,
     {1: "BUS_P", 2: "GND"}, (37.5, 37, 0), (330.0, 112.0), dnp=True,
     lcsc="C2157827",
     descr="ESD element on bus P, at the connector (AN1718 MOV1) - TDK "
           "AVRH10C221KT1R5YA8 chip varistor, 1005 [0402], 1.5 pF. Optional.")
part("MOV2", "Device:Varistor", "220V(1mA) 1.5pF ESD (DNP)", FP_MOV,
     {1: "BUS_N", 2: "GND"}, (21, 37, 180), (330.0, 142.0), dnp=True,
     lcsc="C2157827",
     descr="ESD element on bus N, at the connector (AN1718 MOV2) - TDK "
           "AVRH10C221KT1R5YA8 chip varistor, 1005 [0402], 1.5 pF. Optional.")
part("CN1", "Connector_Generic:Conn_01x04", "T1S BUS  P N N P",
     "Connector_Phoenix_MC:PhoenixContact_MC_1,5_4-G-3.81_1x04_"
     "P3.81mm_Horizontal",
     {1: "BUS_P", 2: "BUS_N", 3: "BUS_N", 4: "BUS_P"},
     # 0.35 mm in from y = 41: the body outline otherwise lands 0.02 mm from
     # Edge.Cuts and the silk is clipped at the board edge.
     (30.405, 40.65, 180), (360.0, 127.0),
     descr="MDI connector, 4-pin 3.81 mm pluggable terminal block. "
           "Pins 1..4 = P N N P; the two P and the two N are shorted on "
           "board so the node taps a daisy chain.")

# --- clock -----------------------------------------------------------------
# 3225 4-pad, not the 3215 2-pad this board first had: no 25 MHz crystal is
# made in 3.2 x 1.5 mm (that package is a 32.768 kHz watch crystal), so the old
# land matched no orderable part.  Pads 1/3 are the crystal, 2/4 the lid, tied
# to each other under the body and to the plane through one via.
# CL 12 pF, not 18: with the data sheet's 18 pF load caps the crystal sees
# 18/2 + ~3 pF stray = 12 pF, so a CL 12 part is the one that runs on
# frequency with those caps.  YXC X322525MOB4SI is also the only 25 MHz part
# that is a JLCPCB basic part.  See ELECTRICAL.md, "Clock".
part("Y1", "Device:Crystal_GND24", "25.000MHz CL12",
     "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm",
     {1: "XTO", 2: "GND", 3: "XTI", 4: "GND"}, (36.9, 26.2, 0), (120.0, 170.0),
     lcsc="C9006",
     descr="25.000 MHz fundamental crystal, CL 12 pF, ESR 50 ohm max, "
           "SMD3225-4P - YXC X322525MOB4SI. No series or feedback resistor "
           "(data sheet).")
part("C15", "Device:C", "18pF", FP_C0603,
     {1: "XTO", 2: "GND"}, (35.65, 29.4, 90), (104.0, 178.0),
     descr="Crystal load capacitor C1 (data sheet: C1 = C2 = 18 pF)",
     lcsc="C1647")
part("C16", "Device:C", "18pF", FP_C0603,
     {1: "XTI", 2: "GND"}, (38.15, 29.4, 90), (136.0, 178.0),
     descr="Crystal load capacitor C2 (data sheet: C1 = C2 = 18 pF)",
     lcsc="C1647")

# --- analogue reference ----------------------------------------------------
part("R7", "Device:R", "12k4 1%", FP_R0603,
     {1: "RBIAS", 2: "GND"}, (41.0, 26.8, 90), (120.0, 200.0),
     descr="RBIAS. 12.4 kohm 1% over the whole operating temperature range "
           "- no substitutions (data sheet).")

# --- supplies --------------------------------------------------------------
part("FB1", "Device:FerriteBead", "0R (FB opt.)", FP_L0603,
     {1: "+3V3", 2: "VDDA"}, (33.0, 30.4, 270), (60.0, 60.0),
     descr="VDDA supply island option: 0 ohm fitted. A real bead needs a local 10 uF after it (DS60001746D); may be replaced by a "
           "~300 ohm @ 100 MHz bead (e.g. Wuerth 742792640).")
part("FB2", "Device:FerriteBead", "0R (FB opt.)", FP_L0603,
     {1: "+3V3", 2: "VDDAU"}, (44, 29.5, 270), (60.0, 90.0),
     descr="VDDAU supply island option: 0 ohm fitted. A real bead needs a local 10 uF after it (DS60001746D).")
part("FB3", "Device:FerriteBead", "0R (FB opt.)", FP_L0603,
     {1: "+3V3", 2: "VDDP"}, (21.5, 18.725, 0), (60.0, 30.0),
     descr="VDDP (pin 7) supply island option: 0 ohm fitted. A real bead needs a local 10 uF after it (DS60001746D).")
part("FB4", "Device:FerriteBead", "0R (FB opt.)", FP_L0603,
     {1: "+3V3", 2: "VDDP_17"}, (34.6, 14.1, 180), (110.0, 80.0),
     descr="VDDP (pin 17) supply island option: 0 ohm fitted. A real bead needs a local 10 uF after it (DS60001746D).")
part("C6", "Device:C", "10uF", FP_C0805,
     {1: "+3V3", 2: "GND"}, (16, 18, 0), (40.0, 45.0),
     descr="Bulk decoupling on the 3.3 V supply side")

# per-pin decoupling: 0.1 uF + 0.01 uF at EACH power pin, 0.01 nearest
part("C7", "Device:C", "100nF", FP_C0603,
     {1: "VDDP", 2: "GND"}, (23.8, 19.5, 90), (90.0, 45.0),
     descr="VDDP (pin 7) decoupling, 0.1 uF")
part("C8", "Device:C", "10nF", FP_C0603,
     {1: "VDDP", 2: "GND"}, (25.4, 19.5, 90), (105.0, 45.0),
     descr="VDDP (pin 7) decoupling, 0.01 uF - closest to the pin")
part("C9", "Device:C", "100nF", FP_C0603,
     {1: "VDDP_17", 2: "GND"}, (34.6, 15.7, 0), (120.0, 45.0),
     descr="VDDP (pin 17) decoupling, 0.1 uF")
part("C10", "Device:C", "10nF", FP_C0603,
     {1: "VDDP_17", 2: "GND"}, (34.6, 17.3, 0), (135.0, 45.0),
     descr="VDDP (pin 17) decoupling, 0.01 uF - closest to the pin")
# C11/C12 sit 0.8 mm further out than the QFN would like so that their ground
# stitching vias clear the all-layer void under the choke, which grew when L1
# went to its real 3.2 x 2.5 mm body.
part("C11", "Device:C", "100nF", FP_C0603,
     {1: "VDDA", 2: "GND"}, (32.4, 28, 180), (150.0, 45.0),
     descr="VDDA (pin 29) decoupling, 0.1 uF")
part("C12", "Device:C", "10nF", FP_C0603,
     {1: "VDDA", 2: "GND"}, (32.4, 26.4, 180), (165.0, 45.0),
     descr="VDDA (pin 29) decoupling, 0.01 uF - closest to the pin")
part("C13", "Device:C", "100nF", FP_C0603,
     {1: "VDDAU", 2: "GND"}, (47.5, 27.6, 0), (180.0, 45.0),
     descr="VDDAU (pin 25) decoupling, 0.1 uF")
part("C14", "Device:C", "10nF", FP_C0603,
     {1: "VDDAU", 2: "GND"}, (47.5, 26, 0), (195.0, 45.0),
     descr="VDDAU (pin 25) decoupling, 0.01 uF - closest to the pin")
# CCOMP: the data sheet's body (DS60001734F, power section) and Microchip's hardware
# design checklist (DS60001746D) both ask for "4.7 uF low ESR (metal film)"; only the
# pin table says plain "low ESR capacitor". A 4.7 uF metal-film part is not a practical
# 0805, so C4 is an X7R MLCC: a deliberate deviation for the prototypes, NOT Microchip-
# compliant, to be checked on the first boards (ELECTRICAL.md, "Bring-up").
part("C4", "Device:C", "4.7uF X7R", FP_C0805,
     {1: "CCOMP", 2: "GND"}, (39.5, 20.55, 0), (215.0, 45.0),
     descr="CCOMP: internal +1.8 V core LDO compensation. REQUIRED, low ESR, "
           "to the ground plane (LAN8651 only). Microchip asks for metal film; "
           "X7R MLCC here is a prototype deviation, see ELECTRICAL.md.")
part("C5", "Device:C", "100nF", FP_C0603,
     {1: "CCOMP", 2: "GND"}, (35.4, 21, 0), (232.0, 45.0),
     descr="CCOMP support capacitor (data sheet: useful, not required)")

# --- host-side pull-ups ----------------------------------------------------
part("R4", "Device:R", "10k", FP_R0603,
     {1: "+3V3", 2: "RESET_N"}, (22.9, 16.2, 0), (95.0, 95.0),
     descr="RESET_N pull-up: a floating host GPIO during ESP32 boot must not "
           "hold the PHY in reset")
# The three pull-ups sit on their signal where it reaches the header, so none
# of them costs a via of its own: R4 and R6 in-line on F.Cu, R5 straight onto
# header pin 24 (a through-hole pad, so F.Cu meets the B.Cu route there).
part("R5", "Device:R", "10k", FP_R0603,
     {1: "+3V3", 2: "SPI_CS_N"}, (26.42, 8.8, 0), (95.0, 105.0),
     descr="SPI CS_N pull-up")
part("R6", "Device:R", "10k", FP_R0603,
     {1: "+3V3", 2: "IRQ_N"}, (25.8, 14.2, 0), (95.0, 115.0),
     descr="IRQ_N pull-up (open-drain capable interrupt output)")

# --- status LEDs -----------------------------------------------------------
part("R8", "Device:R", "1k", FP_R0603,
     {1: "+3V3", 2: "LED0_A"}, (47.5, 19, 180), (330.0, 60.0),
     descr="LED0 series resistor to 3V3 (DIOA0 sinks)")
part("D1", "Device:LED", "GRN", FP_LED,
     {2: "LED0_A", 1: "LED0_K"}, (44, 19, 0), (345.0, 60.0),
     descr="Status LED 0 on DIOA0 - firmware maps it (e.g. PLCA status)")
part("R9", "Device:R", "1k", FP_R0603,
     {1: "+3V3", 2: "LED1_A"}, (47.5, 21.5, 180), (330.0, 75.0),
     descr="LED1 series resistor to 3V3 (DIOA1 sinks)")
part("D2", "Device:LED", "YEL", FP_LED,
     {2: "LED1_A", 1: "LED1_K"}, (44, 21.5, 0), (345.0, 75.0),
     descr="Status LED 1 on DIOA1 - firmware maps it (e.g. activity)")

# LCSC numbers for JLCPCB assembly, each checked against JLCPCB's part search on
# 2026-09-28 (stock and basic/extended in ORDERING.md).  Parts given one in
# their part() call keep it.
LCSC = {
    "U1": "C22386973",                    # LAN8651B1-E/LMX, extended, ~200 in stock
    "C1": "C28233", "C2": "C28233", "C3": "C28233",   # 100n 100V 0805 X7R, basic
    "C4": "C354262",                      # 4.7u 25V 0805 X7R, Yageo CC0805KKX7R8BB475
    "C6": "C15850",                       # 10u 25V 0805 X5R, basic
    "C5": "C14663", "C7": "C14663", "C9": "C14663",   # 100n 0603, basic
    "C11": "C14663", "C13": "C14663",
    "C8": "C57112", "C10": "C57112", "C12": "C57112",  # 10n 0603, basic
    "C14": "C57112",
    "R3": "C149504",                      # 100k 0805 1%, basic
    "R4": "C25804", "R5": "C25804", "R6": "C25804",   # 10k 0603, basic
    "R7": "C22865",                       # 12k4 1% 0603, extended (no basic exists)
    "R8": "C21190", "R9": "C21190",       # 1k 0603, basic
    "FB1": "C21189", "FB2": "C21189", "FB3": "C21189",  # 0R 0603, basic
    "FB4": "C21189",
    "D1": "C12624",                       # green 0603, extended
    "D2": "C2287",                        # KENTO KT-0603Y yellow 0603 (C89811 fell to 20 in stock, 2026-09-30)
    "CN1": "C480536",                     # Phoenix 1803293 MC 1,5/4-G-3,81
    "R1": "C4014562", "R2": "C4014562",   # DNP option: 49R9 1% 1206 0.75 W
}
for _p in P:
    if not _p["lcsc"] and _p["ref"] in LCSC:
        _p["lcsc"] = LCSC[_p["ref"]]

BY_REF = {p["ref"]: p for p in P}

# Nets that get a PWR_FLAG on the schematic: their only drivers on this board
# are passive (the header, or a 0 ohm bead), so ERC needs to be told they are
# supplies.
PWR_FLAG_NETS = ["+3V3", "GND", "VDDP", "VDDP_17", "VDDA", "VDDAU"]


# ===========================================================================
#  SYMBOL LIBRARY -- LAN8651 (no KiCad symbol exists) + the host header
# ===========================================================================
FONT = '(effects (font (size 1.27 1.27)))'


def _pin(etype, x, y, ang, length, name, number, hide=False):
    h = " hide" if hide else ""
    return ('      (pin %s line (at %.4f %.4f %d) (length %.2f)%s\n'
            '        (name "%s" (effects (font (size 1.27 1.27))))\n'
            '        (number "%s" (effects (font (size 1.27 1.27))))\n'
            '      )\n' % (etype, x, y, ang, length, h, name, number))


def _rect(x1, y1, x2, y2):
    return ('      (rectangle (start %.3f %.3f) (end %.3f %.3f)\n'
            '        (stroke (width 0.254) (type default))\n'
            '        (fill (type background))\n'
            '      )\n' % (x1, y1, x2, y2))


def _prop(key, val, x, y, hide=False, just=None):
    e = '(effects (font (size 1.27 1.27))'
    if just:
        e += ' (justify %s)' % just
    if hide:
        e += ' hide'
    e += ')'
    return '      (property "%s" "%s" (at %.3f %.3f 0)\n        %s\n      )\n' \
        % (key, val, x, y, e)


def build_lan8651_symbol():
    XL, XR, YT, YB = -19.05, 19.05, 27.94, -27.94
    LEN = 3.81
    s = '    (symbol "LAN8651" (in_bom yes) (on_board yes)\n'
    s += _prop("Reference", "U", XL, YT + 2.54, just="left")
    s += _prop("Value", "LAN8651", XL, YT + 5.08, just="left")
    s += _prop("Footprint",
               "Package_DFN_QFN:TQFN-32-1EP_5x5mm_P0.5mm_EP3.4x3.4mm"
               "_ThermalVias", 0, 0, hide=True)
    s += _prop("Datasheet", "https://www.microchip.com/DS60001734", 0, 0,
               hide=True)
    s += _prop("Description",
               "10BASE-T1S MAC-PHY with OPEN Alliance TC6 SPI, 32-VQFN",
               0, 0, hide=True)
    s += '      (symbol "LAN8651_0_1"\n' + _rect(XL, YT, XR, YB) + '      )\n'
    s += '      (symbol "LAN8651_1_1"\n'
    for num, name, etype, side, coord in LAN8651_PINS:
        if side == "L":
            s += _pin(etype, XL - LEN, coord, 0, LEN, name, str(num))
        elif side == "R":
            s += _pin(etype, XR + LEN, coord, 180, LEN, name, str(num))
        elif side == "T":
            s += _pin(etype, coord, YT + LEN, 270, LEN, name, str(num))
        else:
            s += _pin(etype, coord, YB - LEN, 90, LEN, name, str(num))
    s += '      )\n    )\n'
    return s


def build_header_symbol():
    """2x20 host header.  Pin electrical types declare the ESP32-S3 side as
    the driver of the six signals ELECTRICAL.md assigns, so ERC sees the SPI
    inputs of U1 as driven.  Everything else is passive."""
    XL, XR = -10.16, 10.16
    LEN = 3.81
    top = 25.4
    s = '    (symbol "HAT_HEADER_2x20" (in_bom yes) (on_board yes)\n'
    s += _prop("Reference", "J", XL, top + 5.08, just="left")
    s += _prop("Value", "HAT_HEADER_2x20", XL, top + 2.54, just="left")
    s += _prop("Footprint",
               "Connector_PinHeader_2.54mm:PinHeader_2x20_P2.54mm_Vertical",
               0, 0, hide=True)
    s += _prop("Datasheet", "", 0, 0, hide=True)
    s += _prop("Description",
               "40-pin Raspberry-Pi-compatible host header on the "
               "LilyGO T-ETH-Elite", 0, 0, hide=True)
    s += '      (symbol "HAT_HEADER_2x20_0_1"\n'
    s += _rect(XL, top, XR, top - 21 * 2.54) + '      )\n'
    s += '      (symbol "HAT_HEADER_2x20_1_1"\n'
    for p in range(1, 41):
        k = (p - 1) // 2
        y = top - 2.54 * (k + 1)
        et = HDR_DRIVEN.get(p, "passive")
        if p in HDR_3V3 or p in HDR_GND or p in (2, 4):
            et = "passive"
        if p % 2:
            s += _pin(et, XL - LEN, y, 0, LEN, RPI_NAMES[p], str(p))
        else:
            s += _pin(et, XR + LEN, y, 180, LEN, RPI_NAMES[p], str(p))
    s += '      )\n    )\n'
    return s


def write_symbol_lib():
    os.makedirs(SYMDIR, exist_ok=True)
    s = '(kicad_symbol_lib (version 20220914) (generator t1s_hat)\n'
    s += build_lan8651_symbol()
    s += build_header_symbol()
    s += ')\n'
    with open(SYMLIB, "w") as f:
        f.write(s)
    print("wrote %s" % SYMLIB)


# ===========================================================================
#  FOOTPRINT LIBRARY -- two lands KiCad does not ship, each drawn from the
#  chosen part's own datasheet.  Nothing here is scaled off a picture or
#  guessed from a neighbouring size code.
# ===========================================================================
def _mod(name, descr, tags, pads, silk=(), crtyd=(0, 0), attr="smd"):
    """Emit a .kicad_mod.  pads: (number, x, y, w, h).  crtyd: (half_x, half_y).
    silk: list of (x1, y1, x2, y2) lines on F.SilkS."""
    s = '(footprint "%s" (version 20221018) (generator t1s_hat)\n' % name
    s += '  (layer "F.Cu")\n'
    s += '  (descr "%s")\n  (tags "%s")\n' % (descr, tags)
    s += '  (attr %s)\n' % attr
    s += ('  (fp_text reference "REF**" (at 0 %.3f) (layer "F.SilkS")\n'
          '    (effects (font (size 0.8 0.8) (thickness 0.12)))\n  )\n'
          % (-crtyd[1] - 0.7))
    s += ('  (fp_text value "%s" (at 0 %.3f) (layer "F.Fab") hide\n'
          '    (effects (font (size 0.8 0.8) (thickness 0.12)))\n  )\n'
          % (name, crtyd[1] + 0.7))
    for x1, y1, x2, y2 in silk:
        s += ('  (fp_line (start %.4f %.4f) (end %.4f %.4f)\n'
              '    (stroke (width 0.12) (type solid)) (layer "F.SilkS"))\n'
              % (x1, y1, x2, y2))
    cx, cy = crtyd
    for x1, y1, x2, y2 in ((-cx, -cy, cx, -cy), (cx, -cy, cx, cy),
                           (cx, cy, -cx, cy), (-cx, cy, -cx, -cy)):
        s += ('  (fp_line (start %.4f %.4f) (end %.4f %.4f)\n'
              '    (stroke (width 0.05) (type solid)) (layer "F.CrtYd"))\n'
              % (x1, y1, x2, y2))
    for num, x, y, w, h in pads:
        s += ('  (pad "%s" smd roundrect (at %.4f %.4f) (size %.4f %.4f)\n'
              '    (layers "F.Cu" "F.Paste" "F.Mask") (roundrect_rratio 0.2))\n'
              % (num, x, y, w, h))
    s += ')\n'
    return s


# --- L1: TDK ACT1210L-201-2P-TL00, EIA 1210 (3.2 x 2.5 mm) 4-terminal CMC ---
# DIRECTED: AN1718's example chokes (ACT1210D-131, ACT1210E-241, Murata
# DLW32MH241MX2) are all 3.2 x 2.5 mm parts; the stock KiCad footprint used
# before was an 1206 (3.2 x 1.6 mm) land, 0.9 mm too narrow, and KiCad ships no
# 4-terminal CMC land in this size.  The part actually fitted is the
# ACT1210L-201-2P-TL00 -- same ACT1210 package, 200 uH @ 100 kHz (inside
# AN1718's 130...240 uH span), AEC-Q200, and stocked at LCSC as C131444.
#
# Land pattern quoted from TDK's ACT1210 data sheet, "Layout recommendation"
# (drawing IND1512-9, TDK Electronics ACT1210, June 2025, page 3):
#     overall across the pads   4.1 mm      -> pad length = (4.1-2.0)/2 = 1.05
#     gap between the columns   2.0 mm      -> pad centres at +/- 1.525
#     overall across the rows   1.6 mm      -> pad width  = (1.6-0.4)/2 = 0.60
#     gap between the rows      0.4 mm      -> pad centres at +/- 0.50
# Body, same data sheet: 3.2 +/-0.2 long, 2.5 +/-0.2 wide, 3.9 max over the
# terminals, 2.5 max high.  Courtyard = 3.9 x 2.7 max body + 0.25 mm a side.
# Pin configuration and circuit diagram: 1 and 2 are one end, 3 and 4 the
# other, the windings are 1-4 and 2-3 -- which is Device:L_Coupled_1423.
ACT1210_PAD_L = 1.05
ACT1210_PAD_W = 0.60
ACT1210_PITCH_X = 1.525
ACT1210_PITCH_Y = 0.50

# --- MOV1/MOV2: TDK AVRH10C221KT1R5YA8, the part ELECTRICAL.md already names -
# It is a 1005 [0402] chip varistor -- 1.0 x 0.5 x 0.5 mm -- not the 1206 the
# board previously assumed (three size codes out).  LCSC C2157827.
# Land pattern quoted from TDK's "Chip varistors / Automotive grade / AVR
# series" catalogue (vpd_automotive_varistors_avr_en, March 2026), AVRH 1005
# RECOMMENDED LAND PATTERN: pad length 0.35...0.45, gap 0.3...0.5, pad width
# 0.4...0.6.  Middle of each range: 0.40 long, 0.40 gap, 0.50 wide, so the pad
# centres sit at +/- 0.40.  (KiCad's R_0402_1005Metric is 0.54 x 0.64 on a
# 1.02 mm pitch -- outside every one of those three ranges, which is why this
# is drawn here instead.)  Body 1.0 +/-0.05 x 0.5 +/-0.05; courtyard = the
# larger of body and land, + 0.25 mm a side.
AVRH10_PAD_L = 0.40
AVRH10_PAD_W = 0.50
AVRH10_PITCH = 0.40


def write_footprint_lib():
    os.makedirs(FPDIR, exist_ok=True)
    mods = {}

    px, py = ACT1210_PITCH_X, ACT1210_PITCH_Y
    w, h = ACT1210_PAD_L, ACT1210_PAD_W
    mods["L_CommonMode_TDK_ACT1210_3225"] = _mod(
        "L_CommonMode_TDK_ACT1210_3225",
        "TDK ACT1210 series 2-line common-mode choke, EIA 1210 "
        "(3.2x2.5 mm), 4 terminals. Land pattern from the TDK ACT1210 data "
        "sheet layout recommendation (4.1 x 1.6 overall, 2.0 and 0.4 gaps).",
        "common mode choke CMC ACT1210 3225 1210 TDK 10BASE-T1S",
        [("1", -px, py, w, h), ("2", -px, -py, w, h),
         ("3", px, -py, w, h), ("4", px, py, w, h)],
        silk=[(-1.95, -1.45, 1.95, -1.45), (-1.95, 1.45, 1.95, 1.45)],
        crtyd=(2.20, 1.60))

    p, w, h = AVRH10_PITCH, AVRH10_PAD_L, AVRH10_PAD_W
    mods["Varistor_TDK_AVRH10_1005"] = _mod(
        "Varistor_TDK_AVRH10_1005",
        "TDK AVRH10 series chip varistor, 1005 [0402] (1.0x0.5 mm). Land "
        "pattern from TDK's AVR automotive catalogue recommended land "
        "(0.40 pad, 0.40 gap, 0.50 wide).",
        "varistor ESD MOV AVRH10 1005 0402 TDK 10BASE-T1S",
        [("1", -p, 0, w, h), ("2", p, 0, w, h)],
        crtyd=(0.85, 0.55))

    for name, text in sorted(mods.items()):
        with open(os.path.join(FPDIR, name + ".kicad_mod"), "w") as f:
            f.write(text)
    print("wrote %s (%d footprints)" % (FPDIR, len(mods)))


# ===========================================================================
#  s-expression helpers: pull a symbol definition out of a .kicad_sym
# ===========================================================================
_SYM_CACHE = {}


def _read_lib(path):
    if path not in _SYM_CACHE:
        _SYM_CACHE[path] = open(path).read()
    return _SYM_CACHE[path]


def sym_block(lib_id):
    """Return (raw_block_text, pins) for 'Lib:Name'.  pins is a list of
    dicts: number, name, etype, x, y, ang (library coords, y up)."""
    lib, name = lib_id.split(":", 1)
    path = SYMLIB if lib == "t1s_hat" else KSYM + lib + ".kicad_sym"
    txt = _read_lib(path)
    tok = '(symbol "%s" ' % name
    i = txt.find(tok)
    if i < 0:
        raise RuntimeError("symbol %s not in %s" % (name, path))
    depth, j = 0, i
    while True:
        c = txt[j]
        if c == '(':
            depth += 1
        elif c == ')':
            depth -= 1
            if depth == 0:
                j += 1
                break
        elif c == '"':
            j += 1
            while txt[j] != '"':
                j += 2 if txt[j] == '\\' else 1
        j += 1
    block = txt[i:j]

    pins = []
    for m in re.finditer(
            r'\(pin\s+(\S+)\s+(\S+)\s+\(at\s+(-?[\d.]+)\s+(-?[\d.]+)\s+'
            r'(-?[\d.]+)\)\s+\(length\s+([\d.]+)\)(\s+hide)?\s*'
            r'\(name\s+"((?:[^"\\]|\\.)*)"', block):
        rest = block[m.end():]
        nm = re.search(r'\(number\s+"((?:[^"\\]|\\.)*)"', rest)
        pins.append(dict(number=nm.group(1), name=m.group(8),
                         etype=m.group(1),
                         x=float(m.group(3)), y=float(m.group(4)),
                         ang=int(float(m.group(5))),
                         length=float(m.group(6))))
    # de-duplicate: alternate body styles repeat pin numbers
    seen, out = set(), []
    for p in pins:
        if p["number"] in seen:
            continue
        seen.add(p["number"])
        out.append(p)
    return block, out


def lib_symbol_for_sch(lib_id):
    """Re-emit a library symbol with its lib-qualified name, for the
    schematic's (lib_symbols) block."""
    block, _ = sym_block(lib_id)
    name = lib_id.split(":", 1)[1]
    return block.replace('(symbol "%s" ' % name, '(symbol "%s" ' % lib_id, 1)


# ===========================================================================
#  SCHEMATIC
# ===========================================================================
# Sheet placement, revised as a block so the circuit reads in groups:
#   y  30..60   supply rail, ferrite islands and every decoupling pair
#   y  95..165  host header (left), pull-ups, LAN8651 (centre)
#   y 100..140  bus interface network (right), in AN1718 order
#   y 185..215  clock, RBIAS, status LEDs
SCH_POS = {
    "C6": (40, 45),  "FB3": (62, 45),  "C7": (86, 45),  "C8": (101, 45),
    "C9": (116, 45), "C10": (131, 45),
    "FB1": (158, 45), "C11": (182, 45), "C12": (197, 45),
    "FB2": (222, 45), "C13": (246, 45), "C14": (261, 45),
    "C4": (290, 45), "C5": (305, 45),
    "FB4": (110, 80),
    "J1": (60, 130),
    "R4": (110, 100), "R5": (110, 125), "R6": (110, 150),
    "U1": (190, 130),
    "L1": (275, 105), "C1": (300, 95), "C2": (300, 125),
    "R1": (325, 100), "R2": (325, 138), "R3": (345, 119), "C3": (362, 119),
    "MOV1": (385, 100), "MOV2": (385, 138), "CN1": (420, 119),
    "Y1": (150, 200), "C15": (133, 210), "C16": (167, 210),
    "R7": (200, 200),
    "R8": (285, 195), "D1": (305, 195), "R9": (285, 215), "D2": (305, 215),
}
def snap(v):
    """KiCad's schematic connection grid is 50 mil = 1.27 mm.  Every symbol
    origin is snapped to it, otherwise ERC fills up with endpoint_off_grid."""
    return round(v / 1.27) * 1.27


for _p in P:
    if _p["ref"] in SCH_POS:
        _p["sch"] = tuple(snap(v) for v in SCH_POS[_p["ref"]])

SHEET_UUID = "5f3b7c10-1a2b-4c3d-8e9f-0a1b2c3d4e50"


def _u(seed):
    return str(_uuid.uuid5(_uuid.UUID(SHEET_UUID), seed))


def sch_pin_pos(part, pin):
    """Sheet (x, y) of a pin, and its outward unit direction (dx, dy).
    All symbols are placed unrotated, so the only transform is the y flip."""
    _, pins = sym_block(part["lib_id"])
    pd = [q for q in pins if q["number"] == str(pin)][0]
    X, Y = part["sch"]
    px, py = X + pd["x"], Y - pd["y"]
    d = {0: (-1, 0), 180: (1, 0), 90: (0, 1), 270: (0, -1)}[pd["ang"] % 360]
    return px, py, d


SCH = []


def w(a, b, seed):
    SCH.append('  (wire (pts (xy %.2f %.2f) (xy %.2f %.2f))\n'
               '    (stroke (width 0) (type default)) (uuid %s)\n  )\n'
               % (a[0], a[1], b[0], b[1], _u("w" + seed)))


def lbl(x, y, txt, d, seed):
    ang, just = (0, "left") if d[0] >= 0 else (0, "right")
    if d[1] > 0:
        ang, just = 270, "right"
    elif d[1] < 0:
        ang, just = 90, "left"
    SCH.append('  (label "%s" (at %.2f %.2f %d)\n'
               '    (effects (font (size 1.27 1.27)) (justify %s bottom))\n'
               '    (uuid %s)\n  )\n' % (txt, x, y, ang, just, _u("l" + seed)))


def nc(x, y, seed):
    SCH.append('  (no_connect (at %.2f %.2f) (uuid %s))\n'
               % (x, y, _u("n" + seed)))


def sch_symbol(lib_id, x, y, ref, value, fp, seed, dnp=False, pins=None,
               show_fields=True, extra_props=()):
    _, plist = sym_block(lib_id)
    if pins is None:
        pins = [q["number"] for q in plist]
    s = '  (symbol (lib_id "%s") (at %.2f %.2f 0) (unit 1)\n' % (lib_id, x, y)
    s += '    (in_bom %s) (on_board yes) (dnp %s)\n' % (
        "yes" if not lib_id.startswith("power:") else "no",
        "yes" if dnp else "no")
    s += '    (uuid %s)\n' % _u("s" + seed)
    hidden = lib_id.startswith("power:")
    s += ('    (property "Reference" "%s" (at %.2f %.2f 0)\n'
          '      (effects (font (size 1.27 1.27)) (justify left)%s)\n    )\n'
          % (ref, x + 2.2, y - 2.2, " hide" if hidden else ""))
    s += ('    (property "Value" "%s" (at %.2f %.2f 0)\n'
          '      (effects (font (size 1.27 1.27)) (justify left)%s)\n    )\n'
          % (value, x + 2.2, y + 0.6,
             "" if show_fields else " hide"))
    if fp is not None:
        s += ('    (property "Footprint" "%s" (at %.2f %.2f 0)\n'
              '      (effects (font (size 1.27 1.27)) hide)\n    )\n'
              % (fp, x, y))
    s += ('    (property "Datasheet" "" (at %.2f %.2f 0)\n'
          '      (effects (font (size 1.27 1.27)) hide)\n    )\n' % (x, y))
    for k, v in extra_props:
        s += ('    (property "%s" "%s" (at %.2f %.2f 0)\n'
              '      (effects (font (size 1.27 1.27)) hide)\n    )\n'
              % (k, v, x, y))
    for pn in pins:
        s += '    (pin "%s" (uuid %s))\n' % (pn, _u("p" + seed + pn))
    s += ('    (instances (project "%s" (path "/%s"\n'
          '      (reference "%s") (unit 1)\n    )))\n  )\n'
          % (NAME, SHEET_UUID, ref))
    SCH.append(s)


def sch_text(x, y, txt, size=2.0, seed=""):
    SCH.append('  (text "%s" (at %.2f %.2f 0)\n'
               '    (effects (font (size %.2f %.2f) bold) (justify left bottom))\n'
               '    (uuid %s)\n  )\n' % (txt, x, y, size, size,
                                         _u("t" + seed + txt)))


def sch_box(x0, y0, x1, y1, seed):
    SCH.append('  (rectangle (start %.2f %.2f) (end %.2f %.2f)\n'
               '    (stroke (width 0.2) (type dash)) (fill (type none))\n'
               '    (uuid %s)\n  )\n' % (x0, y0, x1, y1, _u("r" + seed)))


PWR_N = [0]


def attach(px, py, d, net, seed):
    """From a pin endpoint, draw the stub and terminate it: a power symbol for
    GND/+3V3, a plain label otherwise.

    Rails that need the stub to turn a corner (the header's, which leave
    sideways) run out to their own lane first -- 5.08 mm for GND, 7.62 mm for
    +3V3 -- so a turning stub can never land on the neighbouring pin's stub,
    which is 2.54 mm long.  That bug silently shorted RESET_N to +3V3 on the
    first pass, so the lane separation is load-bearing, not cosmetic."""
    if net == "GND" and d == (0, -1):
        # A GND pin pointing UP (Y1's lid pin): the generic path below would
        # run the stub out and then straight back down over itself, which
        # netlists fine but ERC reports as an unconnected pin.  Turn sideways.
        w((px, py), (px, py - 2.54), seed)
        w((px, py - 2.54), (px + 5.08, py - 2.54), seed + "s")
        PWR_N[0] += 1
        sch_symbol("power:GND", px + 5.08, py - 2.54, "#PWR%03d" % PWR_N[0],
                   "GND", None, seed + "G", show_fields=False)
        return
    reach = 2.54
    if net == "GND" and d != (0, 1):
        reach = 5.08
    elif net == "+3V3" and d != (0, -1):
        reach = 7.62
    ax, ay = px + d[0] * reach, py + d[1] * reach
    w((px, py), (ax, ay), seed)
    if net == "GND":
        if d != (0, 1):
            w((ax, ay), (ax, ay + 2.54), seed + "g")
            ay += 2.54
        PWR_N[0] += 1
        sch_symbol("power:GND", ax, ay, "#PWR%03d" % PWR_N[0], "GND", None,
                   seed + "G", show_fields=False)
    elif net == "+3V3":
        if d != (0, -1):
            w((ax, ay), (ax, ay - 2.54), seed + "v")
            ay -= 2.54
        PWR_N[0] += 1
        sch_symbol("power:+3V3", ax, ay, "#PWR%03d" % PWR_N[0], "+3V3", None,
                   seed + "V", show_fields=False)
    else:
        lbl(ax, ay, net, d, seed)


def write_schematic():
    del SCH[:]
    PWR_N[0] = 0
    used = sorted({p["lib_id"] for p in P} |
                  {"power:GND", "power:+3V3", "power:PWR_FLAG"})

    for p in P:
        X, Y = p["sch"]
        sch_symbol(p["lib_id"], X, Y, p["ref"], p["value"], p["fp"],
                   p["ref"], dnp=p["dnp"],
                   extra_props=(("Description", p["descr"]),
                                ("LCSC", p["lcsc"])) if p["lcsc"] else
                               (("Description", p["descr"]),))
        for pin, net in sorted(p["nets"].items()):
            px, py, d = sch_pin_pos(p, pin)
            seed = "%s.%s" % (p["ref"], pin)
            if net is None:
                nc(px, py, seed)
            else:
                attach(px, py, d, net, seed)

    # PWR_FLAGs.  These nets are fed from passive pins only (the header, or a
    # 0 ohm bead), so ERC has to be told they are supplies.
    fx, fy = snap(40.0), snap(250.0)
    for net in PWR_FLAG_NETS:
        PWR_N[0] += 1
        sch_symbol("power:PWR_FLAG", fx, fy, "#FLG%03d" % PWR_N[0],
                   "PWR_FLAG", None, "FLG" + net, show_fields=False)
        # Both a PWR_FLAG pin and a rail symbol pin leave their origin
        # vertically, so the link between them is drawn horizontally: that
        # keeps each symbol's own graphic clear of the wire, which a vertical
        # link does not, and a doubled-back vertical wire left the +3V3 flag
        # unconnected on the first pass.
        w((fx, fy), (fx + 5.08, fy), "flg" + net)
        if net in ("GND", "+3V3"):
            PWR_N[0] += 1
            sch_symbol("power:" + net, fx + 5.08, fy,
                       "#PWR%03d" % PWR_N[0], net, None, "flgp" + net,
                       show_fields=False)
        else:
            lbl(fx + 5.08, fy, net, (1, 0), "flgl" + net)
        fx += snap(30.0)

    sch_text(30, 25, "T1S HAT for LilyGO T-ETH-Elite  -  Microchip LAN8651 "
                     "10BASE-T1S MAC-PHY", 3.5, "title")
    sch_text(30, 33, "Every value is from ELECTRICAL.md (Microchip "
                     "DS60001734F / AN1718 DS60001718D). Single 3.3 V rail: "
                     "the LAN8651 regulates its own 1.8 V core.", 1.8, "sub")
    sch_text(30, 62, "SUPPLY + DECOUPLING   0.1uF and 0.01uF at EVERY power "
                     "pin, 0.01uF nearest.  FB1..4 are the data sheet's "
                     "optional supply-island beads, fitted as 0R.", 1.8, "g1")
    sch_text(60, 76, "FB4 - VDDP pin 17 has its own island (pin 7 is FB3). "
                     "Both are VDDP; the split is a layout choice.", 1.8, "g7")
    sch_text(255, 85, "BUS INTERFACE NETWORK (AN1718 MINIMAL BIN)   "
                      "CMC - AC coupling - termination - ESD - MDI connector",
             1.8, "g2")
    sch_text(255, 160, "R1/R2 SHIP UNSTUFFED.  Fit 49R9 1% 1W for an "
                       "END-OF-BUS node, or 1K5 1% for an interior DROP node.",
             1.8, "g3")
    sch_text(120, 228, "CLOCK  25.000 MHz, CL 18 pF.  No series and no "
                       "feedback resistor: the device has ~1 Mohm internally "
                       "across the amplifier.", 1.8, "g4")
    sch_text(255, 228, "STATUS LEDs on DIOA0/DIOA1 (VDDP domain, 1k to 3V3; "
                       "the DIO sinks).  DIOA2/3/4 and DIOB0/DIOB1 to ground.",
             1.8, "g5")
    sch_text(30, 248, "PWR_FLAG: these rails are fed through passive pins "
                      "(header pads, 0R beads), so ERC is told where the "
                      "supply enters.", 1.8, "g6")
    sch_box(28, 36, 330, 70, "bx1")
    sch_box(250, 86, 470, 165, "bx2")
    sch_box(115, 180, 235, 232, "bx3")
    sch_box(250, 180, 345, 232, "bx4")

    out = '(kicad_sch (version 20230121) (generator eeschema)\n'
    out += '  (uuid %s)\n' % SHEET_UUID
    out += '  (paper "A2")\n'
    out += ('  (title_block\n    (title "T1S HAT - LAN8651 10BASE-T1S")\n'
            '    (date "")\n    (rev "A")\n'
            '    (company "")\n'
            '    (comment 1 "Generated by make_t1s_hat.py from ELECTRICAL.md")\n'
            '  )\n')
    out += '  (lib_symbols\n'
    for lid in used:
        out += lib_symbol_for_sch(lid) + "\n"
    out += '  )\n'
    out += "".join(SCH)
    out += '  (sheet_instances\n    (path "/" (page "1"))\n  )\n)\n'
    with open(SCH_PATH, "w") as f:
        f.write(out)
    print("wrote %s (%d symbols)" % (SCH_PATH, len(P)))


# ===========================================================================
#  PROJECT + LIBRARY TABLES
# ===========================================================================
# Net classes.  DIRECTED: the board is 2-layer, so the T1S pair is a
# microstrip over the solid B.Cu pour (1.51 mm of FR4).  It is NOT 50 ohm on
# that stack and is not claimed to be; see README.  The pair still gets its
# own class so its width, clearance and the AN1718 3x-trace-width spacing rule
# are enforced by DRC rather than by hand.
T1S_NETS = ["TRXP", "TRXN", "CMC_P", "CMC_N", "BUS_P", "BUS_N"]
NETCLASSES = [
    dict(name="Default", clearance=0.15, track_width=0.25,
         via_diameter=0.60, via_drill=0.30),
    dict(name="Power", clearance=0.15, track_width=0.45,
         via_diameter=0.70, via_drill=0.35,
         nets=["+3V3", "GND", "VDDP", "VDDP_17", "VDDA", "VDDAU"]),
    dict(name="T1S", clearance=0.15, track_width=0.35,
         via_diameter=0.60, via_drill=0.30, nets=T1S_NETS),
]


def write_project():
    classes = []
    assign = []
    for nc_ in NETCLASSES:
        classes.append({
            "bus_width": 12, "clearance": nc_["clearance"],
            "diff_pair_gap": 0.25, "diff_pair_via_gap": 0.25,
            "diff_pair_width": 0.2, "line_style": 0,
            "microvia_diameter": 0.3, "microvia_drill": 0.1,
            "name": nc_["name"], "pcb_color": "rgba(0, 0, 0, 0.000)",
            "schematic_color": "rgba(0, 0, 0, 0.000)",
            "track_width": nc_["track_width"],
            "via_diameter": nc_["via_diameter"], "via_drill": nc_["via_drill"],
            "wire_width": 6,
        })
        for n in nc_.get("nets", []):
            assign.append([n if n in ("+3V3", "GND") else "/" + n,
                           nc_["name"]])
    pro = {
        "board": {"design_settings": {"defaults": {}}},
        "boards": [],
        "cvpcb": {"equivalence_files": []},
        "libraries": {"pinned_footprint_libs": [], "pinned_symbol_libs": []},
        "meta": {"filename": NAME + ".kicad_pro", "version": 1},
        "net_settings": {
            "classes": classes,
            "meta": {"version": 3},
            "net_colors": None,
            "netclass_assignments": dict(assign),
            "netclass_patterns": [],
        },
        "pcbnew": {"last_paths": {}, "page_layout_descr_file": ""},
        "schematic": {"legacy_lib_dir": "", "legacy_lib_list": []},
        "sheets": [[SHEET_UUID, ""]],
        "text_variables": {},
    }
    with open(PRO_PATH, "w") as f:
        json.dump(pro, f, indent=2)
        f.write("\n")
    print("wrote %s" % PRO_PATH)


def write_lib_tables():
    with open(os.path.join(HERE, "sym-lib-table"), "w") as f:
        f.write('(sym_lib_table\n  (version 7)\n'
                '  (lib (name "t1s_hat")(type "KiCad")'
                '(uri "${KIPRJMOD}/sym/t1s_hat.kicad_sym")'
                '(options "")(descr "Parts created for this board"))\n)\n')
    with open(os.path.join(HERE, "fp-lib-table"), "w") as f:
        f.write('(fp_lib_table\n  (version 7)\n'
                '  (lib (name "t1s_hat")(type "KiCad")'
                '(uri "${KIPRJMOD}/fp/t1s_hat.pretty")'
                '(options "")(descr "Lands drawn from the chosen parts\' own '
                'data sheets"))\n)\n')
    print("wrote sym-lib-table, fp-lib-table")


# ===========================================================================
#  PCB
# ===========================================================================
HDR_LIB = KFP + "Connector_PinHeader_2.54mm.pretty"
HDR_FP = "PinHeader_2x20_P2.54mm_Vertical"

NETS = {}          # name -> NETINFO_ITEM
PADPOS = {}        # (ref, padnum) -> (gx, gy)


def add_seg(board, layer, p0, p1, width):
    s = pcbnew.PCB_SHAPE(board)
    s.SetShape(pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(V(*p0))
    s.SetEnd(V(*p1))
    s.SetLayer(layer)
    s.SetWidth(MM(width))
    board.Add(s)
    return s


def add_arc(board, layer, p0, pmid, p1, width):
    s = pcbnew.PCB_SHAPE(board)
    s.SetShape(pcbnew.SHAPE_T_ARC)
    s.SetArcGeometry(V(*p0), V(*pmid), V(*p1))
    s.SetLayer(layer)
    s.SetWidth(MM(width))
    board.Add(s)
    return s


def add_text(board, txt, gx, gy, h=1.0, w=None, th=0.15, angle=0,
             layer=None, just="center", mirror=False):
    if w is None:
        w = h * 0.82
    if layer is None:
        layer = pcbnew.F_SilkS
    t = pcbnew.PCB_TEXT(board)
    t.SetText(txt)
    t.SetLayer(layer)
    t.SetPosition(V(gx, gy))
    t.SetTextSize(SZ(w, h))
    t.SetTextThickness(MM(th))
    t.SetTextAngle(pcbnew.EDA_ANGLE(angle, pcbnew.DEGREES_T))
    t.SetHorizJustify({"center": pcbnew.GR_TEXT_H_ALIGN_CENTER,
                       "left": pcbnew.GR_TEXT_H_ALIGN_LEFT,
                       "right": pcbnew.GR_TEXT_H_ALIGN_RIGHT}[just])
    t.SetVertJustify(pcbnew.GR_TEXT_V_ALIGN_CENTER)
    if mirror:
        t.SetMirrored(True)
    board.Add(t)
    return t


def add_dot(board, layer, gx, gy, r):
    """Filled circle.  One shape, so -- unlike a stroked triangle, whose legs
    overlap each other at the apex -- it cannot violate silk clearance against
    itself."""
    s = pcbnew.PCB_SHAPE(board)
    s.SetShape(pcbnew.SHAPE_T_CIRCLE)
    s.SetStart(V(gx, gy))
    s.SetEnd(V(gx + r, gy))
    s.SetLayer(layer)
    s.SetWidth(MM(0.05))
    s.SetFilled(True)
    board.Add(s)
    return s


def add_npth(board, ref, gx, gy, diameter, descr):
    fp = pcbnew.FOOTPRINT(board)
    fp.SetFPID(pcbnew.LIB_ID("t1s_hat", "NPTH_%.2fmm" % diameter))
    fp.SetReference(ref)
    fp.SetValue("NPTH_%.2fmm" % diameter)
    # the library's description, not the per-hole one: KiCad's footprint-vs-
    # library check compares it, and HOLES already says which hole is which
    fp.SetDescription("Unplated mounting hole, %.2f mm (M2.5 free fit)" % diameter)
    fp.Reference().SetVisible(False)
    fp.Value().SetVisible(False)
    fp.SetAttributes(pcbnew.FP_EXCLUDE_FROM_POS_FILES |
                     pcbnew.FP_EXCLUDE_FROM_BOM)
    pad = pcbnew.PAD(fp)
    pad.SetNumber("")
    pad.SetAttribute(pcbnew.PAD_ATTRIB_NPTH)
    pad.SetShape(pcbnew.PAD_SHAPE_CIRCLE)
    pad.SetDrillShape(pcbnew.PAD_DRILL_SHAPE_CIRCLE)
    pad.SetSize(SZ(diameter, diameter))
    pad.SetDrillSize(SZ(diameter, diameter))
    pad.SetLayerSet(pcbnew.PAD.UnplatedHoleMask())
    fp.Add(pad)
    board.Add(fp)
    fp.SetPosition(V(gx, gy))
    pad.SetPosition(V(gx, gy))
    pad.SetPos0(pcbnew.VECTOR2I(0, 0))
    # save this very footprint as the library's copy, so DRC has an entry to
    # compare the board's holes against and the two cannot drift apart
    lib = fp.Duplicate()
    lib.SetPosition(pcbnew.VECTOR2I(0, 0))
    lib.SetReference("REF**")
    lib.SetDescription("Unplated mounting hole, %.2f mm (M2.5 free fit)" % diameter)
    pcbnew.PCB_PLUGIN().FootprintSave(FPDIR, lib)
    return fp


def draw_outline(board):
    """Plain rectangle, 3 mm rounded corners.  DIRECTED: no RJ45 notch -- a
    taller riser lifts this board over the jack instead of cutting round it,
    which also leaves the B.Cu ground pour uninterrupted."""
    L, R, B, T = 0.0, BOARD_W, 0.0, BOARD_H
    r = CORNER_R
    d = r * (1 - 0.70710678)
    add_seg(board, pcbnew.Edge_Cuts, (L + r, B), (R - r, B), EDGE_W)
    add_arc(board, pcbnew.Edge_Cuts, (R - r, B), (R - d, B + d), (R, B + r),
            EDGE_W)
    add_seg(board, pcbnew.Edge_Cuts, (R, B + r), (R, T - r), EDGE_W)
    add_arc(board, pcbnew.Edge_Cuts, (R, T - r), (R - d, T - d), (R - r, T),
            EDGE_W)
    add_seg(board, pcbnew.Edge_Cuts, (R - r, T), (L + r, T), EDGE_W)
    add_arc(board, pcbnew.Edge_Cuts, (L + r, T), (L + d, T - d), (L, T - r),
            EDGE_W)
    add_seg(board, pcbnew.Edge_Cuts, (L, T - r), (L, B + r), EDGE_W)
    add_arc(board, pcbnew.Edge_Cuts, (L, B + r), (L + d, B + d), (L + r, B),
            EDGE_W)


def net(board, name):
    if name not in NETS:
        n = pcbnew.NETINFO_ITEM(board, name)
        board.Add(n)
        NETS[name] = n
    return NETS[name]


def place_header(board):
    """2x20 THT socket, pads on GEOMETRY.md's grid.  Orientation is chosen by
    measuring the resulting pad vectors, exactly as ../pcb/make_board.py does,
    rather than by trusting a sign convention."""
    want_p3 = (-MM(HDR_PITCH), 0)   # pin 3 one pitch toward low x
    want_p2 = (0, MM(HDR_PITCH))    # pin 2 on the edge row: lower G-y = higher board y
    tol = MM(0.001)
    chosen = None
    for deg in (0, 90, 180, 270):
        fp = pcbnew.FootprintLoad(HDR_LIB, HDR_FP)
        fp.SetPosition(pcbnew.VECTOR2I(0, 0))
        fp.SetOrientation(pcbnew.EDA_ANGLE(deg, pcbnew.DEGREES_T))
        p = {pad.GetNumber(): pad.GetPosition() for pad in fp.Pads()}
        d3 = (p["3"].x - p["1"].x, p["3"].y - p["1"].y)
        d2 = (p["2"].x - p["1"].x, p["2"].y - p["1"].y)
        if (abs(d3[0] - want_p3[0]) < tol and abs(d3[1] - want_p3[1]) < tol and
                abs(d2[0] - want_p2[0]) < tol and
                abs(d2[1] - want_p2[1]) < tol):
            chosen = deg
            break
    if chosen is None:
        raise RuntimeError("no 90 deg orientation gives the required grid")
    print("  J1 orientation %d deg gives GEOMETRY.md's pad grid" % chosen)

    fp = pcbnew.FootprintLoad(HDR_LIB, HDR_FP)
    fp.SetOrientation(pcbnew.EDA_ANGLE(chosen, pcbnew.DEGREES_T))
    fp.SetFPID(pcbnew.LIB_ID("Connector_PinHeader_2.54mm", HDR_FP))
    fp.SetReference("J1")
    fp.SetValue(BY_REF["J1"]["value"])
    fp.Value().SetVisible(False)
    fp.Reference().SetPosition(V(9.0, 8.6))
    fp.Reference().SetTextSize(SZ(0.85, 1.0))
    fp.Reference().SetTextThickness(MM(0.15))
    fp.Reference().SetTextAngle(pcbnew.EDA_ANGLE(0, pcbnew.DEGREES_T))
    fp.SetAttributes(pcbnew.FP_THROUGH_HOLE)
    board.Add(fp)
    pads = {p.GetNumber(): p for p in fp.Pads()}
    target = V(*pin_xy(1))
    cur = pads["1"].GetPosition()
    pos = fp.GetPosition()
    fp.SetPosition(pcbnew.VECTOR2I(pos.x + (target.x - cur.x),
                                   pos.y + (target.y - cur.y)))
    for num, pad in pads.items():
        n = HDR_NET[int(num)]
        if n:
            pad.SetNet(net(board, n))
    return fp


def place_part(board, p):
    lib, fpn = p["fp"].split(":", 1)
    fp = pcbnew.FootprintLoad(fp_dir(lib), fpn)
    if fp is None:
        raise RuntimeError("footprint %s not found" % p["fp"])
    fp.SetFPID(pcbnew.LIB_ID(lib, fpn))
    fp.SetReference(p["ref"])
    fp.SetValue(p["value"])
    fp.Value().SetVisible(False)
    # NOTE: do not SetDescription() here.  KiCad 7's lib_footprint_mismatch
    # test compares the description against the library copy, so setting it
    # raises a warning on every part.  The descriptions live in bom.csv.
    fp.Reference().SetTextSize(SZ(0.70, 0.80))
    fp.Reference().SetTextThickness(MM(0.12))
    if p["dnp"] and hasattr(pcbnew, "FP_DNP"):
        fp.SetAttributes(fp.GetAttributes() | pcbnew.FP_DNP)
    board.Add(fp)
    fp.SetOrientation(pcbnew.EDA_ANGLE(p["pcb"][2], pcbnew.DEGREES_T))
    fp.SetPosition(V(p["pcb"][0], p["pcb"][1]))
    for pad in fp.Pads():
        num = pad.GetNumber()
        try:
            key = int(num)
        except ValueError:
            continue
        n = p["nets"].get(key)
        if n:
            pad.SetNet(net(board, n))
    return fp


PADNET = {}


def index_pads(board):
    PADPOS.clear()
    PADNET.clear()
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            k = (fp.GetReference(), pad.GetNumber())
            PADPOS[k] = gof(pad.GetPosition())
            nn = pad.GetNetname()
            PADNET[k] = nn if nn else None


def pt(spec):
    """Route waypoint: ('REF', 'pad') or (x, y)."""
    if isinstance(spec[0], str):
        return PADPOS[(spec[0], str(spec[1]))]
    return (float(spec[0]), float(spec[1]))


def route(board, netname, width, waypoints, layer=None):
    if layer is None:
        layer = pcbnew.F_Cu
    n = net(board, netname)
    for spec in waypoints:
        if isinstance(spec[0], str):
            have = PADNET.get((spec[0], str(spec[1])))
            if have != netname:
                raise RuntimeError(
                    "route on %s touches pad %s.%s which is on %s"
                    % (netname, spec[0], spec[1], have))
    pts = [pt(s) for s in waypoints]
    for a, b in zip(pts, pts[1:]):
        if abs(a[0] - b[0]) < 1e-9 and abs(a[1] - b[1]) < 1e-9:
            continue
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(V(*a))
        t.SetEnd(V(*b))
        t.SetWidth(MM(width))
        t.SetLayer(layer)
        t.SetNet(n)
        board.Add(t)


def via(board, netname, gx, gy, dia=0.6, drill=0.3):
    v = pcbnew.PCB_VIA(board)
    v.SetPosition(V(gx, gy))
    v.SetWidth(MM(dia))
    v.SetDrill(MM(drill))
    v.SetViaType(pcbnew.VIATYPE_THROUGH)
    v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    v.SetNet(net(board, netname))
    board.Add(v)
    return v


def zone(board, netname, layers, poly, priority=0, rule_area=False,
         no_pour=True, no_tracks=False, no_vias=False, no_pads=False,
         no_fp=False, name=""):
    z = pcbnew.ZONE(board)
    ls = pcbnew.LSET()
    for l in layers:
        ls.addLayer(l)
    z.SetLayerSet(ls)
    # NOTE: build the outline with ZONE.AddPolygon, not SetOutline(poly_set).
    # SetOutline takes ownership of the pointer while SWIG still frees the
    # Python object, and the resulting double free crashes the zone filler and
    # SaveBoard.  That cost an hour; do not "simplify" it back.
    ch = pcbnew.SHAPE_LINE_CHAIN()
    for gx, gy in poly:
        ch.Append(V(gx, gy))
    ch.SetClosed(True)
    z.AddPolygon(ch)
    z.SetZoneName(name)
    if rule_area:
        z.SetIsRuleArea(True)
        z.SetDoNotAllowCopperPour(no_pour)
        z.SetDoNotAllowTracks(no_tracks)
        z.SetDoNotAllowVias(no_vias)
        z.SetDoNotAllowPads(no_pads)
        z.SetDoNotAllowFootprints(no_fp)
    else:
        z.SetNet(net(board, netname))
        z.SetAssignedPriority(priority)
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
        z.SetLocalClearance(MM(0.20))
        z.SetMinThickness(MM(0.15))
        z.SetThermalReliefGap(MM(0.30))
        z.SetThermalReliefSpokeWidth(MM(0.50))
    board.Add(z)
    return z


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


# ---------------------------------------------------------------------------
# ROUTING.  Every net on this board is routed on F.Cu; B.Cu carries nothing
# but the ground pour, which is the reference plane the T1S pair needs.
# Widths: 0.35 for the T1S pair (its net class), 0.45 for supplies, 0.25 for
# signals, 0.30 for short fan-out stubs at the QFN.
# ---------------------------------------------------------------------------
WT, WP, WS, WF = 0.35, 0.45, 0.25, 0.30
WQ = 0.15          # neck width inside the QFN's 0.5 mm pitch

ROUTES = []
VIAS = []


def build_routes():
    """Waypoint lists in G-frame mm.  Pad references keep the copper glued to
    the footprints, so moving a part moves its route with it, and route()
    refuses any waypoint whose pad is on a different net."""
    del ROUTES[:]
    del VIAS[:]
    R = ROUTES.append
    G = VIAS.append

    # ================= T1S pair, U1 -> CMC -> caps -> term -> CN1 =========
    # Symmetric about x = 29.0 from the QFN all the way to the connector, so
    # the two legs are the same length by construction.  No vias on either.
    # L1 pad order is the ACT1210's own: 1 and 2 at the chip end, 3 and 4 at
    # the connector end, windings 1-4 (P) and 2-3 (N).
    R(("TRXP", WQ, [("U1", "30"), (29.25, 24.75), (29.00, 25.00)]))
    R(("TRXP", WT, [(29.00, 25.00), ("L1", "1")]))
    R(("TRXN", WQ, [("U1", "31"), (28.75, 24.00), (28.00, 24.75)]))
    R(("TRXN", WT, [(28.00, 24.75), ("L1", "2")]))
    R(("CMC_P", WT, [("L1", "4"), (29.00, 29.10), (29.50, 29.60),
                     ("C1", "1")]))
    R(("CMC_N", WT, [("L1", "3"), (28.00, 29.10), (27.50, 29.60),
                     ("C2", "1")]))
    R(("BUS_P", WT, [("C1", "2"), (29.50, 32.80), (33.46, 32.80),
                     (33.46, 38.40), (30.405, 38.40), ("CN1", "1")]))
    R(("BUS_N", WT, [("C2", "2"), (27.50, 32.80), (23.54, 32.80),
                     (23.54, 38.40), (26.595, 38.40), ("CN1", "2")]))
    R(("BUS_P", WT, [(33.46, 37.00), ("MOV1", "1")]))
    R(("BUS_N", WT, [(23.54, 37.00), ("MOV2", "1")]))
    R(("BUS_CT", WS, [("R2", "2"), ("R1", "2")]))
    R(("BUS_CT", WS, [(28.50, 34.20), (28.50, 37.00)]))
    R(("BUS_CT", WS, [("C3", "1"), (28.50, 37.00), ("R3", "1")]))
    # CN1 is P N N P: the N pair is adjacent, the P pair is bridged under the
    # connector body, where nothing else runs.
    R(("BUS_N", WT, [("CN1", "2"), ("CN1", "3")]))
    R(("BUS_P", WT, [("CN1", "1"), (30.405, 44.00), (18.975, 44.00),
                     ("CN1", "4")]))

    # ================= QFN top edge, right-hand group =====================
    # One lane per pin; each lane turns up further right than the lane above
    # it, so no two cross.  Verified by construction, not by eye.
    for nm, pin, lane, turn in (("VDDAU", "25", 22.90, 44.00),
                                ("RBIAS", "26", 23.40, 41.00),
                                ("XTI",   "27", 23.90, 39.90),
                                ("XTO",   "28", 24.40, 35.65),
                                ("VDDA",  "29", 24.90, 33.00)):
        R((nm, WQ, [("U1", pin), (PADPOS[("U1", pin)][0], lane)]))
        R((nm, WQ, [(PADPOS[("U1", pin)][0], lane), (turn, lane)]))
    R(("VDDA", WP, [(33.00, 24.90), ("FB1", "2")]))
    R(("VDDA", WF, [(33.00, 26.40), ("C12", "1")]))
    R(("VDDA", WF, [(33.00, 28.00), ("C11", "1")]))
    # 3225 crystal: XTO takes pad 1 (lower left) straight off its lane and
    # goes round the left of the lid pad to C15; XTI runs up outboard of lid
    # pad 2 at x = 39.9 and comes in to pad 3 (upper right) from the side.
    R(("XTO", WS, [(35.65, 24.40), ("Y1", "1")]))
    R(("XTO", WS, [("Y1", "1"), (34.40, 25.35), (34.40, 28.625),
                   ("C15", "1")]))
    R(("XTI", WS, [(39.90, 23.90), (39.90, 27.05), ("Y1", "3")]))
    R(("XTI", WS, [("Y1", "3"), ("C16", "1")]))
    # Lid: pad 4 to pad 2 diagonally under the body (0.29 mm to the crystal
    # pads at the closest corner), then one via beside pad 2.
    R(("GND", WF, [("Y1", "4"), ("Y1", "2")]))
    R(("GND", WF, [("Y1", "2"), (39.15, 25.35)]))
    G(("GND", 39.15, 25.35, 0.45, 0.25))
    R(("RBIAS", WS, [(41.00, 23.40), ("R7", "1")]))
    R(("VDDAU", WP, [(44.00, 22.90), ("FB2", "2")]))
    R(("VDDAU", WF, [(44.00, 26.00), ("C14", "1")]))
    R(("VDDAU", WF, [(44.00, 27.60), ("C13", "1")]))

    # ================= QFN right edge =====================================
    # VDDP pin 17: its own island, caps at the pin, FB4 from the plane
    R(("VDDP_17", WF, [("U1", "17"), ("C10", "1")]))
    R(("VDDP_17", WF, [("C10", "1"), ("C9", "1"), ("FB4", "2")]))
    R(("+3V3", WF, [("FB4", "1"), (36.30, 14.10)]))
    G(("+3V3", 36.30, 14.10, 0.45, 0.25))
    R(("LED0_K", WS, [("U1", "18"), (43.213, 18.75), ("D1", "1")]))
    R(("LED1_K", WS, [("U1", "19"), (42.00, 19.25), (42.00, 21.50),
                      ("D2", "1")]))
    R(("CCOMP", 0.15, [("U1", "21"), (34.625, 20.25), ("C5", "1")]))
    R(("CCOMP", 0.15, [("C5", "1"), (34.625, 22.40), (38.550, 22.40),
                     ("C4", "1")]))
    # DIOA2/3/4 are unused and go straight to ground (data sheet); each gets
    # its own stitching via rather than relying on the F.Cu flood reaching
    # between 0.5 mm-pitch escapes, which it cannot.
    R(("GND", WQ, [("U1", "20"), (36.90, 19.75), (36.90, 20.20)]))
    R(("GND", WQ, [("U1", "22"), (33.30, 20.75)]))
    G(("GND", 33.30, 20.75, 0.45, 0.25))
    R(("GND", WQ, [("U1", "23"), (33.30, 21.25), (33.30, 21.75)]))
    G(("GND", 33.30, 21.75, 0.45, 0.25))

    # ================= QFN left edge: VDDP (pins 4, 7) and RESET_N ======
    # 0.01 uF (C8) then 0.1 uF (C7) straight off pin 7, FB3 behind them from
    # the +3V3 plane.  TEST (pin 4) must sit at VDDP; it joins pin 7 outboard
    # of the pads (pins 5/6 between carry no outward stub).
    R(("VDDP", WF, [("U1", "7"), (26.70, 18.75), ("C8", "1")]))
    R(("VDDP", WF, [("C8", "1"), ("C7", "1"), ("FB3", "2")]))
    R(("VDDP", WQ, [("U1", "4"), (26.70, 20.25), (26.70, 18.75)]))
    R(("+3V3", WF, [("FB3", "1"), (19.80, 18.725)]))
    G(("+3V3", 19.80, 18.725, 0.45, 0.25))
    # ================= host signals: QFN -> header ==========================
    # Header order (corrected numbering, x): SCLK/CS 28.5, MISO 31.1,
    # MOSI 33.6, RESET/IRQ 38.7 -- nearly the reverse of the QFN's
    # IRQ/MISO/CS/SCLK/MOSI.  So:
    #   F.Cu, no vias:  CS, SCLK, MOSI -- the three whose order already agrees
    #   B.Cu:           MISO straight down; IRQ and RESET in two lanes UNDER the
    #                   header's edge row (y 1.9 / 1.2), coming up between pad
    #                   columns -- the only way past MISO without a crossing.
    # Each pull-up sits on its own signal: R5 on the CS trace, R6 at the IRQ via,
    # R4 at the RESET via.
    Bc = pcbnew.B_Cu
    # --- F.Cu
    R(("SPI_CS_N", WQ, [("U1", "11"), (29.25, 12.00), (27.246, 10.00),
                        ("R5", "2")]))
    R(("SPI_CS_N", WS, [("R5", "2"), (27.246, 4.63), ("J1", "24")]))
    R(("+3V3", WS, [("R5", "1"), (25.595, 7.60)]))
    G(("+3V3", 25.595, 7.60, 0.45, 0.25))
    R(("SPI_SCLK", WQ, [("U1", "12"), (29.75, 8.00), (28.516, 7.00),
                        ("J1", "23")]))
    R(("SPI_MOSI", WQ, [("U1", "13"), (30.25, 9.00), (33.596, 7.00),
                        ("J1", "19")]))
    # --- fan-out vias for the B.Cu three
    R(("IRQ_N", WQ, [("U1", "9"), (27.90, 17.20), (27.246, 15.50)]))
    G(("IRQ_N", 27.246, 15.50, 0.45, 0.25))
    R(("IRQ_N", WS, [(27.246, 15.50), ("R6", "2")]))
    R(("+3V3", WS, [("R6", "1"), (24.975, 13.20)]))
    G(("+3V3", 24.975, 13.20, 0.45, 0.25))
    R(("SPI_MISO", WQ, [("U1", "10"), (28.75, 16.90), (28.35, 15.50)]))
    G(("SPI_MISO", 28.35, 15.50, 0.45, 0.25))
    R(("RESET_N", WS, [("U1", "8"), (27.00, 17.69), (24.60, 17.69),
                       (24.00, 16.90)]))
    G(("RESET_N", 24.00, 16.90, 0.45, 0.25))
    R(("RESET_N", WS, [(24.00, 16.90), ("R4", "2")]))
    R(("+3V3", WS, [("R4", "1"), (22.075, 15.20)]))
    G(("+3V3", 22.075, 15.20, 0.45, 0.25))
    # --- B.Cu
    R(("SPI_MISO", 0.20, [(28.35, 15.50), (28.35, 8.30), (31.056, 7.00),
                          ("J1", "21")], Bc))
    R(("IRQ_N", 0.20, [(27.246, 15.50), (27.246, 1.90), (38.676, 1.90),
                       ("J1", "16")], Bc))
    R(("RESET_N", 0.20, [(24.00, 16.90), (24.00, 8.00), (24.706, 7.30),
                         (24.706, 1.20), (39.946, 1.20), (39.946, 4.63),
                         ("J1", "15")], Bc))

    # ================= 3V3: the In2.Cu plane, one via per pad =============
    # Rev A ran a 3V3 rail round three sides of the board; every one of those
    # pads is now a via into the plane beside it.  The header's two 3V3 pins
    # are through-hole and meet the plane directly.
    for ref, pad, gx, gy in (("FB1", "1", 33.00, 32.05),
                             ("FB2", "1", 44.00, 31.15),
                             ("R8", "1", 49.40, 19.00),
                             ("R9", "1", 49.40, 21.50),
                             ("C6", "1", 15.05, 19.30)):
        R(("+3V3", WF, [(ref, pad), (gx, gy)]))
        G(("+3V3", gx, gy, 0.45, 0.25))
    R(("LED0_A", WS, [("D1", "2"), ("R8", "2")]))
    R(("LED1_A", WS, [("D2", "2"), ("R9", "2")]))

    # ================= ground stitching ===================================
    # Every GND pad that the F.Cu flood cannot reach gets its own via to the
    # B.Cu plane.  Two vias would be better per the data sheet's decoupling
    # rule; one is what the 2-layer flood geometry allows here.
    for ent in [
            ("U1", "32", 27.60, 23.20),
            ("C3", "2", 25.15, 35.60), ("R3", "2", 31.812, 35.60),
            ("MOV1", "2", 40.10, 37.00), ("MOV2", "2", 18.50, 37.00),
            ("C15", "2", 35.65, 30.90), ("C16", "2", 38.15, 30.90),
            ("R7", "2", 41.00, 28.70), ("C11", "2", 30.70, 28.00),
            ("C12", "2", 30.70, 26.40), ("C13", "2", 49.10, 27.60),
            ("C14", "2", 49.10, 26.00), ("C4", "2", 41.40, 20.55, 0.45, 0.25),
            ("C5", "2", 36.90, 20.20), ("C6", "2", 17.90, 18.00),
            ("C7", "2", 23.80, 21.25, 0.45, 0.25),
            ("C8", "2", 25.40, 21.25, 0.45, 0.25),
            ("C9", "2", 36.35, 15.70, 0.45, 0.25),
            ("C10", "2", 36.35, 17.30, 0.45, 0.25)]:
        ref, pad, gx, gy = ent[:4]
        G(("GND", gx, gy) + tuple(ent[4:]) if len(ent) > 4
          else ("GND", gx, gy, 0.60, 0.30))
        R(("GND", WF, [(ref, pad), (gx, gy)]))
    R(("GND", WF, [("U1", "32"), (28.00, 23.20), (27.60, 23.20)]))
    R(("GND", 0.20, [("U1", "16"), (31.75, 16.70)]))
    G(("GND", 31.75, 16.70, 0.45, 0.25))
    # GND pins 2, 3, 5 tie inward to the exposed pad, which is stitched to
    # the In1 plane by its own nine vias.
    for pin, y in (("2", 21.25), ("3", 20.75), ("5", 19.75)):
        R(("GND", WQ, [("U1", pin), (28.45, y)]))


# ---------------------------------------------------------------------------
# ZONES AND RULE AREAS
# ---------------------------------------------------------------------------
# The BIN block, for AN1718 guideline 5: a void in the OUTER-layer ground
# flood around the bus interface network, to keep trace capacitance down.
# On a 2-layer board only F.Cu is voided -- B.Cu is the reference plane the
# pair is routed over and must stay whole.
BIN_VOID = (21.0, 23.4, 40.5, 39.8)
# All-layer void under the common-mode choke (AN1718 guideline 6): this one
# DOES cut B.Cu, and it is the only place other than the antenna window that
# does.  AN1718 gives no dimension, so it is sized to the part and no further:
# the ACT1210's maximum body, 3.9 mm over the terminals x 2.7 mm wide
# (2.5 + tolerance), plus 0.25 mm a side for placement.  That also covers the
# 4.1 x 1.6 land with margin, and it is the whole of what couples to the
# plane -- the core.  L1 is rotated 90 deg, so the 3.9 runs along y.
CMC_MARGIN = 0.25
CMC_VOID = (28.5 - (2.7 / 2 + CMC_MARGIN), 26.6 - (3.9 / 2 + CMC_MARGIN),
            28.5 + (2.7 / 2 + CMC_MARGIN), 26.6 + (3.9 / 2 + CMC_MARGIN))


def add_zones(board):
    m = 0.30                                  # zone inset from the board edge
    outline = [(m, m), (BOARD_W - m, m), (BOARD_W - m, BOARD_H - m),
               (m, BOARD_H - m)]

    # antenna keepout: no copper on ANY layer, and nothing placed there
    x0, y0, x1, y1 = KEEPOUT
    ALL = [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]
    zone(board, None, ALL, rect(x0, y0, x1, y1),
         rule_area=True, no_pour=True, no_tracks=True, no_vias=True,
         no_pads=True, no_fp=True, name="ANTENNA KEEPOUT")
    # all-layer void under the CMC
    # (F.Cu keeps tracks allowed: L1's own escapes have to leave its pads;
    # verify() polices that layer by net instead)
    zone(board, None, ALL, rect(*CMC_VOID),
         rule_area=True, no_pour=True, no_tracks=False, no_vias=True,
         name="CMC ALL-LAYER VOID")
    zone(board, None, [pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu], rect(*CMC_VOID),
         rule_area=True, no_pour=True, no_tracks=True, no_vias=True,
         name="CMC VOID - NO TRACKS BELOW")
    # outer-layer ground-flood void around the BIN (F.Cu only)
    zone(board, None, [pcbnew.F_Cu], rect(*BIN_VOID),
         rule_area=True, no_pour=True, name="BIN GROUND-FLOOD VOID")

    # In1.Cu: the ground plane -- whole board, no tracks, the reference for
    # every F.Cu signal including the T1S pair.  In2.Cu: the +3V3 plane.
    z1 = zone(board, "GND", [pcbnew.In1_Cu], outline, priority=0,
              name="GND plane (In1.Cu)")
    z1.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    z2 = zone(board, "+3V3", [pcbnew.In2_Cu], outline, priority=0,
              name="+3V3 plane (In2.Cu)")
    z2.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    # B.Cu: ground pour around the few SPI escapes that run there.
    zb = zone(board, "GND", [pcbnew.B_Cu], outline, priority=0,
              name="GND pour (B.Cu)")
    zb.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    # the QFN exposed pad is a 3.4 mm land with nine thermal vias; thermal
    # spokes starve it, so both ground pours take it solid.
    # F.Cu: ground flood in what is left between the signals.
    zf = zone(board, "GND", [pcbnew.F_Cu], outline, priority=0,
         name="GND flood (F.Cu)")
    zf.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)


def draw_silk(board):
    # Board-level ink lives in the two areas that carry no pads: the strip
    # left of the bus network (x 2.5..20, y 27..44) and the top right corner
    # (x 36..60, y 45..48.5).  Everything is kept clear of CN1's own body
    # outline, which starts at x = 16.2 above y = 38.3.
    add_text(board, "T1S HAT   LAN8651", 48.0, 47.6, h=1.4, w=1.1, th=0.22)
    add_text(board, "10BASE-T1S for LilyGO T-ETH-Elite", 48.0, 45.9,
             h=0.8, w=0.65, th=0.13)

    # bus connector legend.  The four pads are 1.8 x 3.6 mm ovals with the
    # connector's own body outline 0.9 mm below them and R3/C3 0.6 mm below
    # that, so there is nowhere to letter each pad without printing on copper
    # or on CN1's outline: the order goes in the legend instead.
    add_text(board, "T1S BUS", 2.5, 43.6, h=1.3, w=1.05, th=0.22, just="left")
    add_text(board, "1-2 = 3-4", 2.5, 41.7, h=0.9, w=0.72, th=0.14,
             just="left")
    add_text(board, "same pair", 2.5, 40.5, h=0.9, w=0.72, th=0.14,
             just="left")
    add_text(board, "wire P-P, N-N", 2.5, 39.3, h=0.9, w=0.72, th=0.14,
             just="left")
    add_text(board, "CN1  1=P 2=N 3=N 4=P", 2.5, 37.3, h=0.9, w=0.70,
             th=0.14, just="left")

    # termination stuffing, the one thing an assembler must decide
    for i, line in enumerate((
            "R1 R2 = TERMINATION (DNP)",
            "END OF BUS ... 49R9 1% 1W",
            "DROP NODE .... 1K5  1%",
            "R3 C3 fitted")):
        add_text(board, line, 2.5, 31.4 - 1.55 * i, h=0.9, w=0.70, th=0.14,
                 just="left")

    # antenna keepout, marked so nobody fills it in later.  The window runs to
    # the board edge; the ink stops 0.32 mm short of it so the silk does not
    # sit on Edge.Cuts.
    x0, y0, x1, y1 = KEEPOUT
    xe = x1 - 0.32
    for a, b in (((x0, y0), (xe, y0)), ((x0, y1), (xe, y1)),
                 ((x0, y0), (x0, y1))):
        add_seg(board, pcbnew.F_SilkS, a, b, SILK_W)
    add_text(board, "ANTENNA", 62.0, 35.0, h=1.2, w=1.0, th=0.2, angle=90)
    add_text(board, "KEEPOUT - NO COPPER", 60.2, 33.0, h=0.9, w=0.75,
             th=0.14, angle=90)

    # pin-1 marker: a dot OUTBOARD of pin 1, not a triangle above it -- above
    # is where JP1 has to sit, and the two collided there.
    px1, py1 = pin_xy(1)
    add_dot(board, pcbnew.F_SilkS, px1 + 2.2, py1, 0.40)
    add_text(board, "PIN 1", 57.8, 8.6, h=0.9, th=0.14, just="right")
    # Probe points: the six host signals plus 3V3 and GND are adjacent pads of
    # J1 (pins 15-24), reachable from the top once the socket is soldered.
    # Labelling them is the test-point set the LAN8650/1 checklist asks for,
    # without new copper on a signal path.
    for pin, txt in ((15, "RST"), (17, "3V3"), (19, "MOSI"), (21, "MISO"), (23, "SCK")):
        x, y = pin_xy(pin)
        add_text(board, txt, x, y + 2.10, h=0.8, w=0.62, th=0.13)   # clear of J1's outline (+1.33)
    for pin, txt in ((16, "IRQ"), (20, "GND"), (24, "CS")):
        x, y = pin_xy(pin)
        add_text(board, txt, x, y - 2.10, h=0.8, w=0.62, th=0.13)

    add_text(board, "LED0 DIOA0", 51.5, 23.5, h=0.8, w=0.7, th=0.13,
             just="left")
    add_text(board, "LED1 DIOA1", 51.5, 25.5, h=0.8, w=0.7, th=0.13,
             just="left")

    # back side: which way up
    add_text(board, "T-ETH-ELITE SIDE", 33.0, 25.0, h=2.0, th=0.32,
             layer=pcbnew.B_SilkS, mirror=True)
    add_text(board, "socket faces down - use a 2x20 riser, "
                    "not one tall stacking header", 33.0, 22.0, h=1.0,
             th=0.16, layer=pcbnew.B_SilkS, mirror=True)

    place_references(board)


# ---------------------------------------------------------------------------
# REFERENCE DESIGNATORS
# ---------------------------------------------------------------------------
# Library footprints put their reference wherever the library author did,
# which on a board this dense means designators on pads and on each other.
# They are placed here instead: each one is tried in a ring of candidate
# positions around its own part and kept at the first that touches no pad, no
# via, no other ink and no board edge.  A part with nowhere legible left loses
# its designator rather than printing an unreadable pile -- cpl.csv and
# bom.csv carry it, and so does the assembly drawing.
SILK_GAP = 0.15            # mm, on top of the 0.10 mm board silk clearance
REF_SIZE = (0.70, 0.80)    # w, h
REF_TH = 0.12

# Two parts are big enough that the ring around them lands somewhere silly;
# these spots are tried first and still have to pass the same clearance test.
REF_HINT = {"J1": (9.0, 8.6), "U1": (25.6, 21.8), "C5": (35.40, 19.20),
            "R1": (28.2, 33.3), "Y1": (40.69, 22.82), "C12": (36.5, 23.5)}


def _box(item, grow=0.0):
    bb = item.GetBoundingBox()
    g = MM(grow)
    return (bb.GetLeft() - g, bb.GetTop() - g,
            bb.GetRight() + g, bb.GetBottom() + g)


def _hit(a, b):
    return not (a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1])


def silk_obstacles(board):
    """Everything a designator must not touch: copper that would show through
    the silk (pads, vias) and ink that is already placed."""
    obs = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            obs.append(_box(pad, SILK_GAP))
        for it in fp.GraphicalItems():
            if it.GetLayer() == pcbnew.F_SilkS:
                obs.append(_box(it, SILK_GAP))
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_VIA_T:
            obs.append(_box(t, SILK_GAP))
    for d in board.GetDrawings():
        if d.GetLayer() == pcbnew.F_SilkS:
            obs.append(_box(d, SILK_GAP))
    return obs


def place_references(board):
    obs = silk_obstacles(board)
    inset = 0.45                       # keep the ink off Edge.Cuts
    lim = (MM(ORIGIN_X + inset), MM(ORIGIN_Y - BOARD_H + inset),
           MM(ORIGIN_X + BOARD_W - inset), MM(ORIGIN_Y - inset))
    dirs = [(0, 1), (0, -1), (1, 0), (-1, 0),
            (1, 1), (-1, 1), (1, -1), (-1, -1)]
    dropped = []
    # hinted parts first: a hint names the one spot that fits, and an
    # alphabetically earlier neighbour must not take it
    for fp in sorted(board.GetFootprints(),
                     key=lambda f: (f.GetReference() not in REF_HINT,
                                    f.GetReference())):
        ref = fp.Reference()
        if not ref.IsVisible():
            continue
        ref.SetTextSize(SZ(*REF_SIZE))
        ref.SetTextThickness(MM(REF_TH))
        pads = list(fp.Pads())
        if not pads:
            continue
        pb = _box(pads[0])
        for pad in pads[1:]:
            b = _box(pad)
            pb = (min(pb[0], b[0]), min(pb[1], b[1]),
                  max(pb[2], b[2]), max(pb[3], b[3]))
        cx, cy = (pb[0] + pb[2]) // 2, (pb[1] + pb[3]) // 2
        hx, hy = (pb[2] - pb[0]) // 2, (pb[3] - pb[1]) // 2
        placed = False
        # Horizontal first; a part with no room for a horizontal designator
        # gets a sideways one before it gets none at all.  FP_TEXT angles are
        # relative to the footprint, so the part's own rotation is cancelled.
        for ang in (0, 90):
            ref.SetTextAngle(pcbnew.EDA_ANGLE(
                ang - fp.GetOrientationDegrees(), pcbnew.DEGREES_T))
            ref.SetPosition(pcbnew.VECTOR2I(int(cx), int(cy)))
            tb = _box(ref)
            tx, ty = (tb[2] - tb[0]) // 2, (tb[3] - tb[1]) // 2
            cand = []
            if ang == 0 and fp.GetReference() in REF_HINT:
                v = V(*REF_HINT[fp.GetReference()])
                cand.append((v.x, v.y))
            for gap in (0.30, 0.55, 0.85, 1.25, 1.75, 2.40):
                for sx, sy in dirs:
                    cand.append((cx + sx * (hx + tx + MM(gap)),
                                 cy + sy * (hy + ty + MM(gap))))
            for px, py in cand:
                ref.SetPosition(pcbnew.VECTOR2I(int(px), int(py)))
                tb = _box(ref, SILK_GAP)
                if (tb[0] < lim[0] or tb[1] < lim[1] or
                        tb[2] > lim[2] or tb[3] > lim[3]):
                    continue
                if any(_hit(tb, o) for o in obs):
                    continue
                obs.append(tb)
                placed = True
                break
            if placed:
                break
        if not placed:
            ref.SetVisible(False)
            dropped.append(fp.GetReference())
    print("references placed; %d dropped as illegible%s"
          % (len(dropped), (": " + ", ".join(dropped)) if dropped else ""))
    return dropped


# ---------------------------------------------------------------------------
# BUILD
# ---------------------------------------------------------------------------
def build_pcb():
    NETS.clear()
    board = pcbnew.CreateEmptyBoard()
    board.SetCopperLayerCount(4)          # Rev B: 4-layer, see README
    bds = board.GetDesignSettings()
    bds.SetBoardThickness(MM(BOARD_T))
    bds.SetAuxOrigin(V(0.0, 0.0))
    try:
        bds.m_CopperEdgeClearance = MM(0.30)
        bds.m_MinThroughDrill = MM(0.20)      # the QFN EP thermal vias
        bds.m_HoleClearance = MM(0.20)
        bds.m_HoleToHoleMin = MM(0.25)
        bds.m_MinClearance = MM(0.13)
        bds.m_ViasMinSize = MM(0.45)
        bds.m_TrackMinWidth = MM(0.15)
        bds.m_TrackMinWidth = MM(0.15)
        bds.m_SilkClearance = MM(0.10)
    except Exception as e:
        print("  note: some design constraints not settable:", e)
    board.SetLayerName(pcbnew.B_Cu, "B.Cu")
    board.SetLayerName(pcbnew.In1_Cu, "In1.Cu")
    board.SetLayerName(pcbnew.In2_Cu, "In2.Cu")

    draw_outline(board)
    for ref, gx, gy, descr in HOLES:
        add_npth(board, ref, gx, gy, HOLE_D, descr)
    place_header(board)
    for p in P:
        if p["ref"] == "J1":
            continue
        place_part(board, p)
    index_pads(board)

    build_routes()
    for r in ROUTES:
        route(board, *r)
    for v in VIAS:
        via(board, v[0], v[1], v[2], *(v[3:] if len(v) > 3 else ()))

    add_zones(board)
    draw_silk(board)
    pcbnew.SaveBoard(PCB_PATH, board)
    inject_stackup()
    print("wrote %s (zones not yet filled)" % PCB_PATH)
    fill_zones_gui()
    return pcbnew.LoadBoard(PCB_PATH)


def fill_zones_gui():
    """Fill the zones.

    pcbnew's ZONE_FILLER segfaults when called from these Python bindings
    (KiCad 7.0.11, headless), so the fill is done by the real pcbnew on an
    Xvfb display: "Fill All Zones" (B), then save.  Same engine the GUI user
    gets; the board file on disk afterwards carries real filled polygons, not
    an empty <polygon> stub."""
    subprocess.run(["sh", os.path.join(HERE, "fill_zones.sh")], check=True)
    s = open(PCB_PATH).read()
    if s.count("(filled_polygon") < 2:
        raise RuntimeError("zone fill did not produce filled polygons")
    print("zones filled (%d filled_polygon blocks)"
          % s.count("(filled_polygon"))


# JLCPCB's standard 4-layer 1.6 mm build, JLC04161H-7628: 7628 prepreg
# (0.2104 mm, er 4.4) between each outer layer and its plane.  Over that
# prepreg a 0.35 mm microstrip is 50 ohm (IPC-2141: 87/sqrt(er+1.41) *
# ln(5.98h/(0.8w+t)) = 50.1 ohm), which is the T1S pair's width here.
STACKUP = """    (stackup
      (layer "F.SilkS" (type "Top Silk Screen"))
      (layer "F.Paste" (type "Top Solder Paste"))
      (layer "F.Mask" (type "Top Solder Mask") (thickness 0.01))
      (layer "F.Cu" (type "copper") (thickness 0.035))
      (layer "dielectric 1" (type "prepreg") (thickness 0.2104) (material "7628")\
 (epsilon_r 4.4) (loss_tangent 0.02))
      (layer "In1.Cu" (type "copper") (thickness 0.0152))
      (layer "dielectric 2" (type "core") (thickness 1.065) (material "FR4")\
 (epsilon_r 4.6) (loss_tangent 0.02))
      (layer "In2.Cu" (type "copper") (thickness 0.0152))
      (layer "dielectric 3" (type "prepreg") (thickness 0.2104) (material "7628")\
 (epsilon_r 4.4) (loss_tangent 0.02))
      (layer "B.Cu" (type "copper") (thickness 0.035))
      (layer "B.Mask" (type "Bottom Solder Mask") (thickness 0.01))
      (layer "B.Paste" (type "Bottom Solder Paste"))
      (layer "B.SilkS" (type "Bottom Silk Screen"))
      (copper_finish "ENIG")
      (dielectric_constraints no)
    )
"""


def inject_stackup():
    """KiCad 7's Python bindings do not expose the stackup editor, so the
    stack is written into the board file directly: JLCPCB's JLC04161H-7628
    4-layer 1.6 mm build (see STACKUP)."""
    s = open(PCB_PATH).read()
    if "(stackup" in s:
        return
    i = s.find("  (setup\n")
    if i < 0:
        raise RuntimeError("no (setup block in the board file")
    j = i + len("  (setup\n")
    open(PCB_PATH, "w").write(s[:j] + STACKUP + s[j:])


# ===========================================================================
#  VERIFY / DRC / EXPORT
# ===========================================================================
def verify(board):
    """Read the saved board back and compare every mechanical feature against
    GEOMETRY.md.  Fails loudly; the generator gets fixed, not this."""
    tol = 0.01
    rows, worst, ok = [], 0.0, True

    def row(ref, spec, got, d):
        nonlocal worst, ok
        e = max(abs(x) for x in d)
        worst = max(worst, e)
        good = e <= tol
        ok = ok and good
        rows.append((ref, spec, got,
                     " ".join("%+.4f" % x for x in d), "OK" if good else "FAIL"))

    xs, ys = [], []
    for dr in board.GetDrawings():
        if dr.GetLayer() == pcbnew.Edge_Cuts:
            for p in (dr.GetStart(), dr.GetEnd()):
                g = gof(p)
                xs.append(g[0])
                ys.append(g[1])
    row("outline", "0.000..%.3f x 0.000..%.3f" % (BOARD_W, BOARD_H),
        "%.3f..%.3f x %.3f..%.3f" % (min(xs), max(xs), min(ys), max(ys)),
        (min(xs), min(ys), max(xs) - BOARD_W, max(ys) - BOARD_H))

    holes = {}
    for fp in board.GetFootprints():
        if re.fullmatch(r"H\d", fp.GetReference()):
            pad = list(fp.Pads())[0]
            g = gof(pad.GetPosition())
            holes[fp.GetReference()] = (g[0], g[1],
                                        pcbnew.ToMM(pad.GetDrillSize().x))
    for ref, sx, sy, _d in HOLES:
        ax, ay, ad = holes[ref]
        row(ref, "%.3f, %.3f  D%.2f" % (sx, sy, HOLE_D),
            "%.3f, %.3f  D%.2f" % (ax, ay, ad),
            (ax - sx, ay - sy, ad - HOLE_D))

    j1 = [f for f in board.GetFootprints() if f.GetReference() == "J1"][0]
    pads = {p.GetNumber(): p for p in j1.Pads()}
    for pin in range(1, 41):
        sx, sy = pin_xy(pin)
        ax, ay = gof(pads[str(pin)].GetPosition())
        row("J1.%d" % pin, "%.3f, %.3f" % (sx, sy),
            "%.3f, %.3f" % (ax, ay), (ax - sx, ay - sy))

    # antenna keepout: nothing at all inside it, on any layer
    x0, y0, x1, y1 = KEEPOUT
    intruders = []
    for t in board.GetTracks():
        for p in ((t.GetStart(),) if t.Type() == pcbnew.PCB_VIA_T
                  else (t.GetStart(), t.GetEnd())):
            g = gof(p)
            if x0 <= g[0] <= x1 and y0 <= g[1] <= y1:
                intruders.append("track/via at %.2f,%.2f" % g)
    for fp in board.GetFootprints():
        g = gof(fp.GetPosition())
        if x0 <= g[0] <= x1 and y0 <= g[1] <= y1:
            intruders.append("footprint %s" % fp.GetReference())
    for z in board.Zones():
        if z.GetIsRuleArea():
            continue
        for lay in (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu):
            if not z.IsOnLayer(lay):
                continue
            poly = z.GetFilledPolysList(lay)
            for i in range(poly.OutlineCount()):
                oc = poly.Outline(i)
                for k in range(oc.PointCount()):
                    g = gof(oc.CPoint(k))
                    if x0 + 0.01 <= g[0] <= x1 - 0.01 and \
                            y0 + 0.01 <= g[1] <= y1 - 0.01:
                        intruders.append(
                            "%s copper at %.2f,%.2f on %s"
                            % (z.GetNetname(), g[0], g[1],
                               board.GetLayerName(lay)))
                        break
    row("keepout", "x %.2f..%.2f y %.2f..%.2f empty" % KEEPOUT,
        "%d intruders" % len(intruders), (float(len(intruders)),))
    for s in intruders[:8]:
        print("    keepout intruder:", s)

    # Header ORIENTATION, checked against physical facts rather than pin_xy():
    # pin 1 must be the pad nearest the USB-C end (high x) and on the inner
    # row, pin 2 (5 V) on the edge row directly outboard of it.
    j1 = {p.GetNumber(): gof(p.GetPosition()) for p in
          next(f for f in board.GetFootprints() if f.GetReference() == "J1").Pads()}
    xs1 = max(v[0] for v in j1.values())
    row("J1 pin 1 end", "USB-C end x=%.3f, inner row" % xs1,
        "%.3f, %.3f" % j1["1"], (j1["1"][0] - xs1, j1["1"][1] - 5.900))
    row("J1 pin 2", "edge row under pin 1", "%.3f, %.3f" % j1["2"],
        (j1["2"][0] - j1["1"][0], j1["2"][1] - 3.360))

    # the planes carry no tracks at all: In1 is the reference for every
    # signal, In2 is +3V3
    inner = [t for t in board.GetTracks()
             if t.Type() != pcbnew.PCB_VIA_T and
             t.GetLayer() in (pcbnew.In1_Cu, pcbnew.In2_Cu)]
    row("plane tracks", "0 on In1/In2", "%d tracks" % len(inner),
        (float(len(inner)),))
    # the all-layer void under the choke: no track or via of any net crosses it
    # on any layer (L1's own pads sit inside it, their stubs leave it)
    vx0, vy0, vx1, vy1 = CMC_VOID

    def _in_void(pt):
        gx, gy = pt.x / 1e6 - 100.0, 100.0 - pt.y / 1e6
        return vx0 < gx < vx1 and vy0 < gy < vy1

    def _crosses(t):
        a, b = t.GetStart(), t.GetEnd()
        return any(_in_void(pcbnew.VECTOR2I(int(a.x + (b.x - a.x) * k / 20),
                                            int(a.y + (b.y - a.y) * k / 20)))
                   for k in range(21))
    # F.Cu: L1's own nets, plus one known exception -- the VDDA dogleg from U1
    # pin 29 runs up x = 29.75 to y 24.9, 0.25 mm inside the choke body's
    # maximum outline.  Rev B is frozen; moving it is queued for Rev C
    # (README).  Anything else appearing here fails.
    f_ok = {"TRXP", "TRXN", "CMC_P", "CMC_N", "VDDA"}
    void_hits = [t for t in board.GetTracks() if _crosses(t) and not
                 (t.Type() != pcbnew.PCB_VIA_T and t.GetLayer() == pcbnew.F_Cu
                  and t.GetNetname() in f_ok)]
    row("CMC void", "no track/via below F.Cu", "%d" % len(void_hits),
        (float(len(void_hits)),))
    pair = [t for t in board.GetTracks()
            if t.Type() != pcbnew.PCB_VIA_T and t.GetNetname() in
            ("CMC_P", "CMC_N", "BUS_P", "BUS_N")]
    widths = sorted({round(t.GetWidth() / 1e6, 3) for t in pair})
    row("pair width", "0.350 (50 ohm)", "%s" % widths,
        (max(abs(w - 0.35) for w in widths),))

    # the two project-local lands: pad count and the land dimensions that were
    # quoted from the data sheets, read back out of the board
    fps = {f.GetReference(): f for f in board.GetFootprints()}
    for ref, n, span_x, span_y in (("L1", 4, 4.1, 1.6),
                                   ("MOV1", 2, 1.2, 0.5),
                                   ("MOV2", 2, 1.2, 0.5)):
        f = fps[ref]
        pads = list(f.Pads())
        row("%s pads" % ref, "%d" % n, "%d" % len(pads),
            (float(len(pads) - n),))
        xs, ys = [], []
        for pad in pads:
            # undo the placement rotation: compare the land as drawn
            p0 = pad.GetPos0()
            sz = pad.GetSize()
            xs += [pcbnew.ToMM(p0.x) - pcbnew.ToMM(sz.x) / 2.0,
                   pcbnew.ToMM(p0.x) + pcbnew.ToMM(sz.x) / 2.0]
            ys += [pcbnew.ToMM(p0.y) - pcbnew.ToMM(sz.y) / 2.0,
                   pcbnew.ToMM(p0.y) + pcbnew.ToMM(sz.y) / 2.0]
        row("%s land" % ref, "%.2f x %.2f" % (span_x, span_y),
            "%.2f x %.2f" % (max(xs) - min(xs), max(ys) - min(ys)),
            (max(xs) - min(xs) - span_x, max(ys) - min(ys) - span_y))

    # the all-layer void must cover the choke and not much else
    vx0, vy0, vx1, vy1 = CMC_VOID
    row("CMC void", "%.2f x %.2f (part +%.2f)"
        % (2.7 + 2 * CMC_MARGIN, 3.9 + 2 * CMC_MARGIN, CMC_MARGIN),
        "%.2f x %.2f" % (vx1 - vx0, vy1 - vy0),
        (vx1 - vx0 - (2.7 + 2 * CMC_MARGIN),
         vy1 - vy0 - (3.9 + 2 * CMC_MARGIN)))

    w = [max(len(str(r[i])) for r in rows) for i in range(5)]
    hdr = ("feature", "spec (mm)", "in .kicad_pcb (mm)", "delta", "")
    w = [max(w[i], len(hdr[i])) for i in range(5)]
    line = "  ".join("-" * x for x in w)
    print(line)
    print("  ".join(hdr[i].ljust(w[i]) for i in range(5)))
    print(line)
    for r in rows:
        print("  ".join(str(r[i]).ljust(w[i]) for i in range(5)))
    print(line)
    print("worst error: %.5f mm   tolerance: %.3f mm   %s"
          % (worst, tol, "ALL OK" if ok else "FAILURES PRESENT"))
    return ok


def unrouted(board):
    """Count ratsnest connections still missing.  KiCad's own number, plus the
    pads the DRC engine names, so the figure can be checked against drc.rpt."""
    board.BuildConnectivity()
    n = board.GetConnectivity().GetUnconnectedCount(True)
    print("UNROUTED CONNECTIONS (ratsnest): %d" % n)
    return n


def run_drc(board):
    """kicad-cli 7.0.11 has no `pcb drc`; call the same engine through the
    bindings.  Nothing is excluded and nothing is filtered."""
    os.environ.setdefault("KICAD7_FOOTPRINT_DIR", KFP)
    rpt = os.path.join(HERE, "drc.rpt")
    units = getattr(pcbnew, "EDA_UNITS_MILLIMETRES",
                    getattr(pcbnew, "EDA_UNITS_MM", 1))
    okay = pcbnew.WriteDRCReport(board, rpt, units, True)
    print("WriteDRCReport -> %s (%s)" % (okay, rpt))
    txt = open(rpt).read()
    tail = [l for l in txt.splitlines() if l.startswith("**")]
    print("\n".join(tail))
    return txt


# ===========================================================================
#  BOM / CPL / GERBERS / PREVIEWS
# ===========================================================================
def write_bom():
    """One line per distinct value+footprint, JLCPCB column order."""
    groups = {}
    for p in P:
        key = (p["value"], p["fp"], p["dnp"], p["descr"], p["lcsc"])
        groups.setdefault(key, []).append(p["ref"])
    rows = []
    for (value, fp, dnp, descr, lcsc), refs in groups.items():
        refs = sorted(refs, key=lambda r: (re.sub(r"\d", "", r),
                                           int(re.sub(r"\D", "", r) or 0)))
        rows.append([value + (" [DNP]" if dnp else ""), ",".join(refs),
                     fp, len(refs), "DNP" if dnp else "fit", descr, lcsc])
    rows.sort(key=lambda r: r[1])
    with open(os.path.join(HERE, "bom.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["Comment", "Designator", "Footprint", "Quantity",
                     "Populate", "Description", "LCSC"])
        wr.writerows(rows)
    print("wrote bom.csv (%d lines, %d parts, %d DNP)"
          % (len(rows), len(P), sum(1 for p in P if p["dnp"])))


# Hand-soldered, so kept out of the JLCPCB assembly files: J1's body goes on the
# BOTTOM of this board (facing the riser) and a top-side CPL entry would get it
# fitted the wrong way up.  DNP parts are left out for the same reason.
JLC_EXCLUDE = {"J1"}


def write_jlc():
    """jlcpcb/bom_jlc.csv + jlcpcb/cpl_jlc.csv in JLCPCB's own column names,
    fitted SMT parts only, from the same part list and the kicad-cli cpl.csv."""
    out = os.path.join(HERE, "jlcpcb")
    os.makedirs(out, exist_ok=True)
    fit = [p for p in P if not p["dnp"] and p["ref"] not in JLC_EXCLUDE]
    missing = [p["ref"] for p in fit if not p["lcsc"]]
    assert not missing, "no LCSC number for %s" % missing
    groups = {}
    for p in fit:
        groups.setdefault((p["value"], p["fp"], p["lcsc"]), []).append(p["ref"])
    with open(os.path.join(out, "bom_jlc.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
        for (value, fp, lcsc), refs in sorted(groups.items(),
                                              key=lambda kv: kv[1][0]):
            wr.writerow([value, ",".join(sorted(refs)), fp.split(":")[-1], lcsc])
    keep = {p["ref"] for p in fit}
    rows = [jlc_place(r) for r in csv.DictReader(open(os.path.join(HERE, "cpl.csv")))]
    with open(os.path.join(out, "cpl_jlc.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        n = 0
        for r in rows:
            if r["Ref"] in keep:
                wr.writerow([r["Ref"], r["PosX"] + "mm", r["PosY"] + "mm",
                             "Top" if r["Side"] == "top" else "Bottom",
                             r["Rot"]])
                n += 1
    assert n == len(keep), "cpl has %d of %d fitted parts" % (n, len(keep))
    print("wrote jlcpcb/bom_jlc.csv (%d lines) + cpl_jlc.csv (%d parts)"
          % (len(groups), n))

    # Build variants: the only thing that differs between nodes is where they sit
    # on the bus, so each variant is the base plus R1/R2 fitted.
    base_cpl = {r["Ref"]: r for r in rows}
    for name, value, lcsc in (("end", "49R9 1% 0.75W", "C4014562"),
                              ("drop", "1K5 1%", "C26030")):
        d = os.path.join(out, name)
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "bom_jlc.csv"), "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
            for (v, fp, lc), refs in sorted(groups.items(), key=lambda kv: kv[1][0]):
                wr.writerow([v, ",".join(sorted(refs)), fp.split(":")[-1], lc])
            wr.writerow([value, "R1,R2", "R_1206_3216Metric", lcsc])
        with open(os.path.join(d, "cpl_jlc.csv"), "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
            for ref in sorted(keep | {"R1", "R2"}):
                r = base_cpl[ref]
                wr.writerow([ref, r["PosX"] + "mm", r["PosY"] + "mm",
                             "Top" if r["Side"] == "top" else "Bottom", r["Rot"]])
    print("wrote jlcpcb/end/ (R1/R2 49R9) and jlcpcb/drop/ (R1/R2 1K5)")


# JLC places each part with the footprint of its LCSC number (EasyEDA's library),
# not with ours, so the CPL has to be in THAT footprint's frame. Each part's LCSC
# footprint was compared pad by pad with ours on 2026-09-30 (check_jlc_cpl.py
# re-runs it); two differ:
#   L1  EasyEDA draws the ACT1210 with its terminals along y, ours along x:
#       our 90 deg placement is its 0 deg. At 90 it would sit across the pads.
#   CN1 EasyEDA's origin is the body centre, KiCad's Phoenix footprint's is pin 1:
#       5.715 mm apart. JLC's position has to be the pad centroid.
JLC_ROT_FIX = {"L1": -90.0}
JLC_CENTROID = {"CN1"}


def jlc_place(r):
    r = dict(r)
    if r["Ref"] in JLC_ROT_FIX:
        r["Rot"] = "%.6f" % ((float(r["Rot"]) + JLC_ROT_FIX[r["Ref"]]) % 360)
    if r["Ref"] in JLC_CENTROID:
        fp = pcbnew.LoadBoard(PCB_PATH).FindFootprintByReference(r["Ref"])
        ps = [q.GetPosition() for q in fp.Pads()]
        r["PosX"] = "%.6f" % (sum(q.x for q in ps) / len(ps) / 1e6 - 100.0)
        r["PosY"] = "%.6f" % (100.0 - sum(q.y for q in ps) / len(ps) / 1e6)
    return r


def export_all():
    """kicad-cli exports, all plotted against the drill/place origin, which
    build_pcb() put on GEOMETRY.md's own origin -- so the drill files and
    cpl.csv read directly in GEOMETRY.md coordinates."""
    gerb = os.path.join(HERE, "gerbers")
    if os.path.isdir(gerb):
        for f in os.listdir(gerb):
            os.remove(os.path.join(gerb, f))
    else:
        os.mkdir(gerb)

    def cli(*args):
        subprocess.run(["kicad-cli", "pcb", "export"] + list(args) + [PCB_PATH],
                       check=True, cwd=HERE, stdout=subprocess.DEVNULL)

    cli("gerbers", "-o", "gerbers/", "--use-drill-file-origin",
        "--layers", "F.Cu,In1.Cu,In2.Cu,B.Cu,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,"
                    "F.Paste,Edge.Cuts")
    cli("drill", "-o", "gerbers/", "--format", "excellon",
        "--drill-origin", "plot", "--excellon-separate-th",
        "--generate-map", "--map-format", "gerberx2")
    cli("pos", "-o", "cpl.csv", "--format", "csv", "--units", "mm",
        "--side", "both", "--use-drill-file-origin")
    for side, layers in (("front", "Edge.Cuts,F.Cu,F.Silkscreen,F.Mask"),
                         ("back", "Edge.Cuts,B.Cu,B.Silkscreen,B.Mask"),
                         ("in1", "Edge.Cuts,In1.Cu"), ("in2", "Edge.Cuts,In2.Cu")):
        out = "preview-%s.svg" % side
        cli("svg", "-o", out, "--page-size-mode", "2",
            "--exclude-drawing-sheet", "--drill-shape-opt", "2",
            "--layers", layers)   # not --mirror: it flips about the page
        #                            centre, which moves the board out of the
        #                            viewBox below.  The back view is drawn
        #                            as seen THROUGH the board.
        path = os.path.join(HERE, out)
        t = open(path).read()
        t = re.sub(r'width="[^"]+mm" height="[^"]+mm" viewBox="[^"]+"',
                   'width="%.2fmm" height="%.2fmm" viewBox="-1.5 -1.5 %.2f %.2f"'
                   % (BOARD_W + 3, BOARD_H + 3, BOARD_W + 3, BOARD_H + 3),
                   t, count=1)
        open(path, "w").write(t)
        subprocess.run(["rsvg-convert", "-w", "1600", "-b", "#101418",
                        out, "-o", "preview-%s.png" % side],
                       check=True, cwd=HERE)
    # kicad-cli drops local state next to the board; not a deliverable
    for junk in (NAME + ".kicad_prl",):
        fp = os.path.join(HERE, junk)
        if os.path.exists(fp):
            os.remove(fp)

    zf = os.path.join(gerb, NAME + "_gerbers.zip")
    with zipfile.ZipFile(zf, "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(os.listdir(gerb)):
            if f.endswith(".zip"):
                continue
            z.write(os.path.join(gerb, f), f)
    print("wrote gerbers/, cpl.csv, preview-front/back.svg+.png, %s"
          % os.path.basename(zf))


def main():
    write_symbol_lib()
    write_footprint_lib()
    write_schematic()
    write_project()
    write_lib_tables()
    print()
    board = build_pcb()
    print()
    ok = verify(board)
    print()
    n = unrouted(board)
    print()
    run_drc(board)
    write_bom()
    export_all()
    write_jlc()
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
