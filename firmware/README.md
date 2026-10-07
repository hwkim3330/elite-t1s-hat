# T1S HAT firmware

`t1s_node/` brings the HAT's LAN8651 up on a LilyGO T-ETH-Elite as a normal
`esp_netif` Ethernet interface, so lwIP (ping, UDP, sockets) runs over
10BASE-T1S unchanged. It is the bring-up tool for the first board and the
base that a larger node firmware can fold in beside the W5500.

**Status: runs on hardware** (three T-ETH-Elite boards, with a third-party LAN8651 HAT on
the same header pins; see "Not done / not verified" for exactly what ran). This HAT's own
board has not been built yet. Every stage a first power-up can fail at prints what it saw;
see "First power-up" below.

## Build and flash

Run from this `firmware/` folder. Arduino core `esp32:esp32` 3.3.0 (ESP-IDF 5.5), the same toolchain as
the W5500 bench firmware this was developed next to:

```bash
FQBN="esp32:esp32:esp32s3:PSRAM=opi,USBMode=hwcdc,CDCOnBoot=cdc,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB"
arduino-cli compile --fqbn "$FQBN" t1s_node
arduino-cli upload  --fqbn "$FQBN" -p /dev/ttyACM0 t1s_node
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

**Partition scheme:** `app3M_fat9M_16MB` gives two 3 MB app slots, so OTA has room. The
build with zenoh is 1.16 MB, which would be 88 % of the default 1.25 MB slot. The first flash
with this scheme has to go over USB, because it rewrites the partition table. After that,
every update can go over WiFi.

### Over WiFi: console and OTA

Every board brings up WiFi last in `setup()` and offers two services:

| | |
|---|---|
| console | TCP port 23 (`nc <ip> 23`). It is the same console as USB serial, and everything printed goes to both |
| OTA | ArduinoOTA, hostname `t1s-<last 4 hex of MAC>`, announced over mDNS. The password defaults to `t1s-ota`; change it with `ota <pass>` + reboot |

```
wifi <ssid> <pass>    join that network as a station (saved; on reboot)
wifi ap               no network: open its own AP "t1s-<id>" (pass t1s-bench), 192.168.4.1
wifi                  show WiFi / OTA / console state
```

Update over WiFi:

```bash
arduino-cli upload --fqbn "$FQBN" --protocol network --port <board-ip> \
  --upload-field password=t1s-ota t1s_node
# or: python3 ~/.arduino15/packages/esp32/hardware/esp32/*/tools/espota.py \
#       -i <board-ip> -a t1s-ota -f <build>/t1s_node.ino.bin
```

The WiFi console uses a different subnet from the T1S / W5500 side, so test traffic never
goes over WiFi by accident. A bench console can find the boards by mDNS (`_telnet._tcp`,
`t1s-*`), or take them by IP.

Built here (2026-10-02): 1.07 MB without zenoh, 1.17 MB with it (34 % / 37 % of a 3 MB slot). No warnings from the sketch, the bridge or
the LAN865x driver. The copied `w5500_spi.h` shows its original volatile-`++`
warnings.

Check which board `/dev/ttyACM0` actually is before uploading: on this
machine it is also the LAN9662, and `arduino-cli board list` has been wrong
about it before.

## What is in it

| | |
|---|---|
| `t1s_node.ino` | bring-up, PLCA, UDP echo on port 7, serial console |
| `pins.h` | HAT signal → Elite GPIO, read off LilyGo's schematic (CS_N on IO0 for Rev C / TSN Lab's HAT, IO8 for Rev D; 1PPS in on IO40) |
| `bridge.h` | bridge mode: W5500 bring-up (no IP, promiscuous) + the two-port learning forwarder |
| `w5500_spi.h` | from the W5500 bench firmware: the W5500 SPI layer that splits reads at the RX buffer wrap (a real IDF driver bug found there) |
| `src/lan865x/` | Espressif's `lan865x` 0.2.0 + `lan86xx_common` MAC-PHY driver, Apache-2.0, vendored with one marked patch (raw register access). See `VENDORED.md` |

The driver talks OPEN Alliance TC6 over **SPI3** (MOSI IO11, MISO IO9,
SCLK IO10, CS IO0, or IO8 on HAT Rev D: bring-up reads DEVID through each and keeps the one
that answers) with IRQ_N on IO39. It uses SPI3 because the W5500
firmware already uses SPI2, so the two can later share one image as a
T1S ↔ 100BASE-TX bridge.

## Modes

`mode node|bridge|sniff|tx`, then `save` and `reboot`. The mode lives in NVS.

| mode | what the board is | IP | status |
|---|---|---|---|
| **node** (default) | a 10BASE-T1S endpoint: lwIP on the LAN8651, ping / UDP echo / `blast`; the Elite's own RJ45 (W5500) comes up next to it as a second interface, see below | yes, 192.168.50.x (+ LAN: DHCP) | **run** (LAN: driver and PHY up, no cable yet) |
| **bridge** | a **learning Ethernet bridge** between the Elite's W5500 (100BASE-TX) and the LAN8651 (10BASE-T1S), `bridge.h` | none, managed over USB serial | compiles |
| **tx** | **no LAN8651:** the Elite's W5500 as an ordinary IP endpoint (echo, sink, `blast`, `ping`), default 192.168.100.66. This is the second board of a two-ESP bench, in the PC's place on the converter's 100BASE-TX port | yes, W5500 | **run** |
| **sniff** | a **receive-only bus analyser**: every T1S frame is copied out of the W5500 to a capture PC (Wireshark). PLCA is held off, there is no IP, and nothing is sent on T1S, so the bus under test is not disturbed | none | compiles, not run |

**The RJ45 in node mode.** Until 2026-10-06 node mode never started the W5500, so the board's
own LAN port was dead while the HAT ran -- not an SPI conflict (W5500 on SPI2, LAN8651 on SPI3;
bridge mode runs both). Now `lan dhcp` (default), `lan <ip> [mask]` or `lan off`, then `save` +
`reboot`. It is a separate interface on its own subnet, nothing is forwarded to T1S (that is
bridge mode), and its route priority is below T1S's, so the default route and Zenoh's multicast
stay on the bus. Echo (7), sink (9), the TCP console (23) and OTA answer on it too. `status`
prints `lan: up|down ip ...` and the W5500's PHYCFGR. Keep its subnet off T1S's (192.168.100.x on
the bench). The W5500 has no auto-MDIX: to another W5500 board use a crossover cable.

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
blast <ip> [sec] [bytes] [port] [mbit]  UDP (port 9), seq-numbered; mbit paces it
sink [reset]               what arrived on port 9: rate, then `sinkx:` seq loss / reorder / dup, arrival-gap p50/p90/p99/max
rtt <ip> [n] [bytes] [ms]  UDP echo round trip timed in us (one in flight), summary + every sample (`rtts:`)
counters                   TC6 status/errors, TX credits, RX chunks, PLCA beacons
identify [sec]             strobe the board LED to find the board
promisc on|off             accept every frame (tells "nothing arrives" from "filtered")
phyreset                   (mode tx) W5500 PHY reset with autonegotiation
phy auto|10h|10f|100h|100f (mode tx) W5500 link mode, written to PHYCFGR
rxlog                      last received frames: source, destination, type
reg r|w <mms> <addr> [val] raw LAN8651 register (hex)
mode node|bridge|sniff|tx  what the board is (save + reboot)
wifi [<ssid> <pass>|ap|off] WiFi station / own AP / no radio (on reboot)
ota <pass>                 OTA password (on reboot)
zenoh [status|pause|resume|ping <hz>|rtts|blast <s> <B>|sink]
save / reboot
```

Settings live in NVS (`t1s` namespace). A fresh board comes up with **PLCA off
(CSMA/CD), 192.168.50.9, SPI at 25 MHz**.

**Asked vs actual SPI clock.** The ESP32-S3's SPI peripheral divides 80 MHz by an integer and rounds to the
nearest divider, so `spi 25` used to run at **26.67 MHz** -- above the LAN8651's 25 MHz maximum (DS60001734F,
Table 9-9, f<sub>SCK</sub>). Since 2026-10-06 bring-up lowers the request until the clock that runs is
<= 25 MHz: `spi 25` and `spi 22` run at **20 MHz** (80/4, the fastest in-spec clock on this chip), `spi 18`
and `spi 15` at 16, `spi 12` at 11.43. Bring-up prints both (`t1s: SPI 25 MHz asked, 20.00 MHz actual`) and
`status` shows the actual one. Every "25 MHz" result in these docs and the reports before 2026-10-06 was
measured at 26.67 MHz, out of spec.

**SPI clock fallback.** Bring-up tries the saved clock, then 20, 12 and 4 MHz, and keeps the
first one the driver installs at (the driver checks parity on every control reply, so a clock
the wiring can't carry fails install instead of corrupting data). The boot log prints each
attempt, and `status` shows the clock it ended up on. A board that only came up at 12 MHz
says so without anyone setting `spi 12`.

**Counters.** `counters` prints only registers the driver itself uses: TC6 STATUS0/1, the
current TX credits / RX chunks, and PLCA_STS. A first version also read a guessed MAC
statistics block (Cadence-GEM layout, MMS 1 words 0x42…0x69). On a real LAN8651 every
value read 0, and **reading it every 5 s stopped the chip from transmitting**, so it is gone.
Frame counts come from `rx t1s` / `rx w5500` in `status`, which count what the driver hands
up and need no register map.

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
- **SPI clock:** 25 MHz is the default request (20 MHz actual, see above), with the automatic step-down. A LAN8651
  HAT ran clean at 25 MHz asked (26.67 actual, out of spec, before the clamp): 9.0 Mbit/s to the node and
  9.5 from it, against 6.0 / 6.3 at 12 MHz asked (11.43 actual). The to-node rates were read
  by the board's sink, which then ran ≈0.5 % high (fixed in cf0ba44); see the
  [errata](https://github.com/hwkim3330/t1s-eval/blob/main/ERRATA.md). The step-down itself has not run on hardware.
- **Run on hardware (2026-10-02, three Elite boards):** `mode tx`, the SPI step-down (no HAT:
  25 → 20 → 12 → 4, fails cleanly, console stays up), the WiFi AP + TCP console (a PC joined
  `t1s-9c3c` and ran `status` over `nc`), **OTA** (espota over the AP, 1.17 MB in 14.7 s,
  `ota: done` → reboot → LAN8651 up → Zenoh session up), Zenoh peer-to-peer, `rtt`, sink
  statistics. **Not run:** sniff mode, bridge mode with this firmware, Zenoh remote config.

## Zenoh-pico over T1S (optional)

Build with zenoh-pico on the library path and two flags. The node then joins Zenoh over its wired
interface (T1S on a HAT node, the W5500 in `mode tx`):

```bash
F="-DZENOH_ARDUINO_ESP32 -DT1S_WITH_ZENOH"     # add -DZENOH_ROUTER='"udp/<ip>:7447"' to use a zenohd
arduino-cli compile --fqbn "$FQBN" --library <path>/zenoh-pico \
  --build-property "compiler.c.extra_flags=$F" --build-property "compiler.cpp.extra_flags=$F" t1s_node
```

**Default: peer to peer over UDP multicast (`udp/224.0.0.224:7447#iface=eth`), no router.** Two
boards form a Zenoh network on their own. (`#iface=` is required by zenoh-pico's multicast locator
check; the ESP32 port ignores its value and lwIP sends from the default netif, the wired one.)

**Wireshark.** [`tools/zenoh.lua`](../tools/zenoh.lua) decodes it: copy it to `~/.local/lib/wireshark/plugins/`
(or `tshark -X lua_script:tools/zenoh.lua`). UDP 7447 then shows `JOIN peer <zid>`, `PUT t1s/t1s-hat-0/signal '23.69'`,
DECLAREs, and a tree down to the key expression and payload; written against zenoh-pico 1.10.1's codec.

**TTL 1.** 224.0.0.0/24 is link-local (RFC 3171/5771) and must be sent with TTL 1; lwIP's default is 64,
which Wireshark colours red on every Zenoh datagram ("TTL low or unexpected"). zenoh-pico sets no
multicast TTL, so the vendored copy is patched: [`zenoh-pico-multicast-ttl1.patch`](zenoh-pico-multicast-ttl1.patch)
(`patch -p1` inside the zenoh-pico folder). Checked on a capture: all 551 datagrams TTL 1, no expert notes.

| key | |
|---|---|
| `t1s/<node>/hello`, `/signal`, `test/stats/<node>` | 2 Hz, 20 Hz, 1 Hz |
| `test/ping/<node>` → `test/pong/<node>` | every board echoes every other board's ping; each measures its own RTT |
| `t1s/<node>/bulk` | `zenoh blast`, counted by `zenoh sink` on the other board |
| `t1s/<node>/cmd`, `/config` → `/config/ack` | console text, remote config |

`<node>` is `t1s-hat-<PLCA id>` on a HAT node and `t1s-eth-<mac>` on a W5500-only board.

```
zenoh [status] | pause | resume | ping <hz> | rtts [reset] | blast <sec> <bytes> [batch] | sink [reset]
```

Measured on the two-ESP bench (2026-10-02, report in `t1s-eval/two-esp/`): pub/sub RTT 5.6–6.0 ms
median; up to 5.5 Mbit/s of payload. Small messages stop near 1000/s — on the **publisher**: unbatched,
every put is its own UDP datagram and the put call takes ~1 ms; the subscriber received every one it was
sent. `zenoh blast <s> <B> batch` wraps the run in `zp_batch_start/stop`: 64 B puts then take ~130–150 µs
and reach ~5500–6000 msg/s (2026-10-06, SPI 20 MHz); 1024 B gains nothing (one message fills a datagram).
Batching delays each put until its batch is flushed, so it is for bulk, not control traffic.

Where the Zenoh round trip goes: raw UDP between the same two boards is 2.97 ms (most of it the
W5500's 1 ms receive poll on the ESP-B side; PC ↔ HAT measured 0.85 ms). Echoing a ping the moment
its callback fires (task notification instead of the 2 ms loop) and publishing ping/pong as
`is_express` brought the Zenoh median from 6.14 to **5.20 ms** pinged from ESP-B and from 5.58 to
**5.26 ms** pinged from the HAT (2026-10-02, 480/449 samples at 50 Hz). The remaining ~2.3 ms over
raw UDP is inside zenoh-pico. (A first measurement showed no change; the HAT was still running an
image an earlier OTA had put in the other app slot -- see "Flashing after an OTA".)

### Time sync (`sync`, `ptp`)

`sync <ip> [n] [ms]` runs the two-way time-transfer exchange PTP and NTP use (t1..t4) against
`<ip>`'s UDP 5007 and prints clock offset and one-way delay per exchange; the console's *Time sync*
panel runs it and plots the result. With software timestamps on both boards (ESP-B's W5500 adds
its 1 ms receive poll on one direction) the offset scatters by **σ ≈ 150–220 µs**, **≈ 50 µs**
using the lowest-delay 10 %, **≈ 40–50 µs** taking one min-delay exchange per 2 s; the two crystals
differ by ~1–3 ppm (2026-10-06). Precision only: there is no reference clock on the bench.

**Hardware timestamps (`ptp on`, build with `-DLAN865X_FRAME_TIMESTAMPS`).** The LAN8651 has a 1588 wall
clock (TSU, 40 ns ticks), prepends an ingress stamp to received frames and captures a frame's egress
time (OA_CONFIG0 FTSE/FTSS, TC6 header TSC, TTSCA). The driver carries hooks for both
(`esp_eth_mac_lan865x_set_ts_hooks`); `ptp on` connects them and `sync` then also reports offsets
from the node's hardware stamps, sent in a follow-up as PTP does. Two things had to be found:

- FTSE/FTSS take effect only when written **with SYNC at init** (later writes read back set but do
  nothing, and SYNC cannot be cleared) -- hence the build flag, which costs 8 bytes of SPI per
  received frame.
- The stamp is triggered by the **PHY's packet matcher** at the SFD on the wire (DS60001734F 4.5.2.2),
  which out of reset is **off** and set to gPTP Sync only. Symptom: STATUS1.TTSCMA ("capture requested,
  not triggered"), no RTSA. `ptp on` sets it to match every frame (mask 0xFFFFFF, location 0,
  TXMCTL/RXMCTL enable) as Microchip's drivers do; the vendored driver has those writes on a path
  this chip revision does not take.

Measured 2026-10-06, node hardware vs ESP-B software (W5500): offset σ per 2 s **16.7 µs** (software on
both sides 43 µs), lowest-delay 10 % 31 µs; the TSU runs off the HAT's 25 MHz crystal, ~20 ppm from
the ESP32's. The far side is the limit here (W5500 poll, converter, PLCA wait): hardware-to-hardware
needs a second LAN8651 node.

**Clock servo (`ptp lock <ip> [sec] [kp] [ki]`).** The node disciplines its LAN8651 clock to a
master's, as ptp4l steers a PHC: 16 exchanges a second with its own request and the reply stamped
in hardware, the min-delay one per second, one step (TA register below 1 s, else TSL/TN), then a PI
loop on the TSU frequency -- TI (ns) plus TISUBN (2^-24 ns; its 24 bits are split: low byte in
31:24, high 16 bits in 15:0; write TISUBN before TI). The request spacing is jittered by 0–1 ms:
at a whole-ms spacing every request meets the master's 1 ms W5500 poll at the same phase and the
min-delay pick has nothing to choose from. Against ESP-B (software stamps, W5500) on 2026-10-06:
offset after 20 s **mean −3.4 µs, σ 32 µs** (max 130), frequency held at **−22.3 ppm** (the HAT's
25 MHz crystal against the ESP32's); ptp4l's gains (0.7 / 0.3) amplify this bench's ~50 µs
measurement noise into ±40 ppm swings, 0.2 / 0.02 is the default.

Then, on the master's side (ESP-B), step by step, 120 s runs, offset after 20 s:

| change | σ | max | one-way delay p50 |
|---|---|---|---|
| W5500 polled every 1 ms (above) | 32 µs | 130 µs | 750 µs |
| **W5500 interrupt on IO14** (the Elite wires INTn there) | 18.5 µs | 52 µs | 736 µs |
| **+ t2 stamped at the driver** (RX tap, before lwIP and the socket task) | 11.6–14.9 µs | 28–48 µs | 520 µs |
| t3 taken after the send returned (two-step) -- reverted | 23 µs | 78 µs | 193 µs |

Mean offset −0.3 … −1 µs, frequency −20.8 … −21.0 ppm ± 2.5–3 ppm. The rest is the master's
software stamps and the converter; two LAN8651 nodes (hardware on both ends) are the next step,
and the servo already uses a master's hardware follow-up when it gets one.

**PPS (`ptp pps on` / `off` / `status`, not yet run on hardware).** The LAN8651's dedicated 1PPS on
DIOA4: a 19.9 µs pulse at every whole second of its clock (PADCTRL A4SEL = 01, PPSCTL PPSEN). HAT
Rev D routes it to test pad TP1 and header 13 (IO40), where the firmware counts the edges as a
self-check; two locked nodes' pulses on a scope show the sync error directly. It drives DIOA4 as an
output, so on a board found with CS on IO0 (Rev C, TSN Lab's HAT: DIOA tied to ground) it wants
`force`. `ptp pps eg0 force` is the older path, event generator 0 on DIOA0 with a 100 ms pulse; no HAT
routes DIOA0. Until 2026-10-07 that path wrote EG0 one register too high (0x222…0x227 instead of
DS60001734F's 0x221…0x226) -- never run, since every board so far grounds DIOA.

**Zenoh and the servo (2026-10-07).** Zenoh's 5 Hz ping shares the bus with the exchanges: paired
60 s runs gave σ 32.7 µs with it, 19.2 µs with `zenoh pause` on both boards (3 of 3 pairs,
`esp32-t1s-bridge/docs/t1s_results/ptp_wifi`). Pause it for a measurement; the tablet app does.

**Throughput cost.** `-DLAN865X_FRAME_TIMESTAMPS` adds an 8-byte stamp to every received frame on
SPI: onto T1S (1472 B, 9.5 offered) 8.75 → 8.20 Mbit/s (−6 %); transmit unchanged. Hence a build
option, not the default. The W5500 interrupt costs nothing (8.76 polled, 8.75 interrupt) and lifts
64 B frames from 1.97 to 2.26 Mbit/s; `-DW5500_POLL` keeps the old 1 ms poll for comparisons with
runs taken before 2026-10-06 evening.

### Flashing after an OTA

An OTA writes the *other* app slot and points `otadata` at it. A later USB flash of the app at
0x10000 alone then does nothing visible: the board keeps booting the OTA'd image. Erase
`otadata` with it (`esptool erase-region 0xe000 0x2000`), or flash the merged image from 0x0.
