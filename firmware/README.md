# T1S HAT firmware

`t1s_node/` brings the HAT's LAN8651 up on a LilyGO T-ETH-Elite as a normal
`esp_netif` Ethernet interface, so lwIP (ping, UDP, sockets) runs over
10BASE-T1S unchanged. It is the bring-up tool for the first board and the
base that a larger node firmware can fold in beside the W5500.

**Status: compiles, never run.** No HAT has been built yet, and no Elite was
attached when this was written. Every stage a first power-up can fail at
prints what it saw; see "First power-up" below.

## Build and flash

Run from this `firmware/` folder. Arduino core `esp32:esp32` 3.3.0 (ESP-IDF 5.5), the same toolchain as
the W5500 bench firmware this was developed next to:

```bash
FQBN="esp32:esp32:esp32s3:PSRAM=opi,USBMode=hwcdc,CDCOnBoot=cdc,FlashSize=16M"
arduino-cli compile --fqbn "$FQBN" t1s_node
arduino-cli upload  --fqbn "$FQBN" -p /dev/ttyACM0 t1s_node
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

Built here: 580 KB flash (44 %). No warnings from the sketch, the bridge or
the LAN865x driver. The copied `w5500_spi.h` shows its original volatile-`++`
warnings.

Check which board `/dev/ttyACM0` actually is before uploading: on this
machine it is also the LAN9662, and `arduino-cli board list` has been wrong
about it before.

## What is in it

| | |
|---|---|
| `t1s_node.ino` | bring-up, PLCA, UDP echo on port 7, serial console |
| `pins.h` | HAT signal → Elite GPIO, read off LilyGo's schematic (and why CS_N on IO0 is fine) |
| `bridge.h` | bridge mode: W5500 bring-up (no IP, promiscuous) + the two-port learning forwarder |
| `w5500_spi.h` | from the W5500 bench firmware: the W5500 SPI layer that splits reads at the RX buffer wrap (a real IDF driver bug found there) |
| `src/lan865x/` | Espressif's `lan865x` 0.2.0 + `lan86xx_common` MAC-PHY driver, Apache-2.0, vendored with one marked patch (raw register access). See `VENDORED.md` |

The driver talks OPEN Alliance TC6 over **SPI3** (MOSI IO11, MISO IO9,
SCLK IO10, CS IO0) with IRQ_N on IO39. It uses SPI3 because the W5500
firmware already uses SPI2, so the two can later share one image as a
T1S ↔ 100BASE-TX bridge.

## Modes

`mode node|bridge|sniff`, then `save` and `reboot`. The mode lives in NVS.

| mode | what the board is | IP | status |
|---|---|---|---|
| **node** (default) | a 10BASE-T1S endpoint: lwIP on the LAN8651, ping / UDP echo / `blast` | yes, 192.168.50.x | compiles |
| **bridge** | a **learning Ethernet bridge** between the Elite's W5500 (100BASE-TX) and the LAN8651 (10BASE-T1S), `bridge.h` | none, managed over USB serial | compiles |
| **sniff** | a **receive-only bus analyser**: every T1S frame is copied out of the W5500 to a capture PC (Wireshark). PLCA is held off, there is no IP, and nothing is sent on T1S, so the bus under test is not disturbed | none | compiles, not run |

Sniff is the converter's "ID ≥ 1, count 0" mode on this board. Wire the W5500 port
straight to the capture PC, not through a switch (the switch would learn the T1S
stations on that port). `status` shows frames seen / copied / dropped. The saved PLCA
settings are kept and come back when the mode is switched back to node.

Bridge mode is what turns the board into a D10-backbone ↔ T1S-edge gateway.
T1S nodes behind it reach `zenohd` on the PC as if they were on the switch.
Zenoh itself doesn't need to know a bridge exists, which is why this is L2 and
not a Zenoh-level gateway: zenoh-pico is a client/peer and cannot route, so a
Zenoh "gateway" would mean re-publishing named keys by hand.

How it works, and its limits:

- **Both drivers already deliver raw frames.** The IDF W5500 driver runs the
  chip in MACRAW, and the LAN865x is a MAC-PHY. Bridging means taking each
  driver's input path away from lwIP and transmitting on the other port. Both
  ports run in promiscuous mode.
- **It learns MAC addresses** (64 stations, 300 s ageing). Unicast between two
  hosts on the 100 Mbit side is never copied onto the 10 Mbit bus.
  Broadcast/multicast is forwarded both ways, because ARP needs it. So connect
  the bridge to an edge port, not a trunk.
- **Throughput is bounded by the slow side and the SPI hops.** The T1S bus is
  10 Mbit/s shared and half duplex. The W5500 path measured ~8 Mbit/s in
  the W5500 bench. When the LAN8651 has no TX credits the frame is dropped
  and counted (`status` → "dropped (no TX room)"). TCP backs off; UDP floods
  lose frames.
- **One bridge per T1S segment.** There is no spanning tree. Two bridges from
  the same bus into the same switch fabric make a loop.
- The ESP32 in the middle is also a place to add filtering, VLAN handling,
  delay/loss injection or timestamps later. None of that is written yet.

IP routing is also possible (`CONFIG_LWIP_IP_FORWARD` is on in this core, with
two subnets and a static route on the PC). It isn't implemented, because the
bridge already covers the test-bench use.

## Console

```
status                     link, ip, PLCA as read back, chip id, PADCTRL
plca <id> [count]          PLCA node id (0 = coordinator) and node count
csma                       PLCA off, plain CSMA/CD
ip <a.b.c.d> [mask]        static address (default 192.168.50.10+id)
spi <mhz>                  starting SPI clock 1..25, on reboot (default 25, steps down by itself)
ping <ip> [n]              ICMP over the bus
blast <ip> [sec] [bytes]   UDP to port 9, reports offered Mbit/s
sink [reset]               what arrived on this node's port 9, and at what rate
counters [reset]           MAC frame / collision / error counters, TC6 status, PLCA beacons
reg r|w <mms> <addr> [val] raw LAN8651 register (hex)
save / reboot
```

Settings live in NVS (`t1s` namespace). A fresh board comes up with **PLCA off
(CSMA/CD), 192.168.50.9, SPI at 25 MHz**.

**SPI clock fallback.** Bring-up tries the saved clock, then 20, 12 and 4 MHz, and keeps the
first one the driver installs at (the driver checks parity on every control reply, so a clock
the wiring can't carry fails install instead of corrupting data). The boot log prints each
attempt, and `status` shows the clock it ended up on. A board that only came up at 12 MHz
says so without anyone setting `spi 12`.

**Counters.** The LAN8651's MAC is a Cadence GEM. Its statistics clear on read, so the
firmware adds them up every 5 s and on `counters`. It shows frames tx/rx, single /
multiple / excessive / late collisions, deferred, carrier errors, and rx FCS / symbol /
alignment / no-buffer / overrun. It also prints TC6 STATUS0/1, the current TX credits /
RX chunks, and PLCA_STS (beacons seen or not). The register addresses come from GEM's
layout, and the Linux driver's MAC registers match that layout. **They have not been read
on a real part yet.** On first hardware, check that `tx frames` / `rx frames` rise with
`ping`.

## First power-up: what each line means

| log line | meaning |
|---|---|
| `t1s: spi bus: ESP_OK` | bus pins claimed. Anything else is a firmware conflict, not the HAT |
| `t1s: driver install: ESP_OK` + `t1s: chip LAN8651 rev N` | **SPI reaches the chip.** This is the main hardware test |
| `driver install: ESP_ERR_TIMEOUT` / `ESP_ERR_INVALID_CRC` | SPI is not getting through: riser seated? 3V3 on header pins 1/17? try `spi 4`, `save`, `reboot` |
| `driver install: ESP_ERR_INVALID_VERSION` | a chip answered with the wrong ID: bit errors, lower the clock |
| `plca: on, id …` | read back from the PHY, not echoed from config |
| `t1s: link up` | the PHY reports link. On T1S that needs a correctly terminated bus (see below) |

## Two-node bench test

T1S is a bus, and a two-node bus is **two end nodes**: both HATs get
R1/R2 = 49R9 (the README's termination table). Then:

```
node A:  plca 0 2      save  reboot     # coordinator, 2 nodes -> 192.168.50.10
node B:  plca 1 2      save  reboot     # -> 192.168.50.11
node A:  ping 192.168.50.11
node A:  sink reset
node B:  blast 192.168.50.10 10
node A:  sink                            # what actually arrived
```

Wire CN1 P to P and N to N (pins 1/4 are P, 2/3 are N). With PLCA on both,
`blast` reports what left node B, `sink` on node A what arrived; the gap
between them is loss on the bus. Targets: **3–5 Mbit/s** delivered at first,
**7–9 Mbit/s** once tuned (SPI clock, frame size). `blast` should approach the bus rate; with `csma` on both it will still work,
with random backoff instead of fixed slots.

A PC can only join with its own T1S interface (e.g. a USB 10BASE-T1S
adapter). With the Linux `lan865x` / `onsemi` T1S drivers the PC side is
set with `ethtool --set-plca-cfg <if> enable on node-id <n> node-cnt <m>`.

## Not done / not verified

- **Run on hardware with a third-party LAN8651 HAT, not yet with this board.**
  TSN Lab's 10BASE-T1S HAT uses the same header pins (SPI0 CE0, IRQ on pin 16), and
  this firmware drove it unmodified on a T-ETH-Elite: DEVID, PLCA, ping, 9.5 Mbit/s,
  Zenoh. This HAT's own boards are not built yet.
- **Status LED:** the LAN8651 has no LED function (its DIOA pins are event
  capture/generator only), so the firmware drives the Elite's own LED (IO38)
  from PLCA_STS: solid = beacons seen, fast blink = no beacons, slow blink = no
  link. Rev C boards have no LEDs of their own.
- **SPI clock:** 25 MHz is now the default, with the automatic step-down above. A LAN8651
  HAT on this firmware ran clean at 25 MHz: 9.0 Mbit/s to the node and 9.5 from it,
  against 6.0 / 6.3 at 12 MHz. The step-down itself has not run on hardware.
- **Written without hardware (2026-10-01), builds with and without zenoh, never run:**
  the SPI step-down, `counters`, sniff mode, and Zenoh remote config.

## Zenoh-pico over T1S (optional)

Build with zenoh-pico on the library path and two flags, and the node opens a Zenoh
session over the LAN8651's network interface once the bus is up:

```bash
F="-DZENOH_ARDUINO_ESP32 -DT1S_WITH_ZENOH"
arduino-cli compile --fqbn esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,USBMode=hwcdc,CDCOnBoot=cdc \
  --library <path>/zenoh-pico \
  --build-property "compiler.c.extra_flags=$F" --build-property "compiler.cpp.extra_flags=$F" t1s_node
```

It publishes `t1s/t1s-hat-<id>/hello` (2 Hz), `t1s/t1s-hat-<id>/signal` (20 Hz),
`test/ping/t1s-hat-<id>` (5 Hz, echoed by a peer on `test/pong/…`) and
`test/stats/t1s-hat-<id>` (RTT, 1 Hz), and prints anything sent to
`t1s/t1s-hat-<id>/cmd`.

**Remote config:** put a console command on `t1s/t1s-hat-<id>/config`. Allowed commands
are `plca`, `csma`, `ip`, `spi`, `mode`, `save`, `reboot`, `status` and `counters`;
`ping` and `blast` are refused, because they would hold the console for seconds. The node
runs it in the console task and answers on `t1s/t1s-hat-<id>/config/ack` with the state
read back from the chip, for example
`ok plca | mode=node plca=1/8 spi=25(saved 25) ip=192.168.50.11 link=up`. `reboot`
waits a second so its ack gets out first. Changing the PLCA ID moves the node to a
different key name only after a reboot (the session keeps the name it opened with).

Router locator: `ZENOH_LOCATOR` (default
`udp/192.168.100.50:7447`). `zenoh` on the console shows the session and RTT.
Without the flags the file is a stub and the build is unchanged.

Measured (2026-10-01, PC ─ 100BASE-TX/10BASE-T1S converter ═ T1S ═ LAN8651 HAT,
PLCA 2 nodes): SPI 25 MHz gives RTT 0.85 ms (64 B ping, p99 0.99), 9.0 Mbit/s
PC → node without loss and 9.5 Mbit/s node → PC; SPI 12 MHz gives 6.0 / 6.3 Mbit/s.
Zenoh ping through the router: ~3.1 ms.

