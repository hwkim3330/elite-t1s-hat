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

`mode node|bridge`, then `save` and `reboot`. The mode lives in NVS.

| mode | what the board is | IP | status |
|---|---|---|---|
| **node** (default) | a 10BASE-T1S endpoint: lwIP on the LAN8651, ping / UDP echo / `blast` | yes, 192.168.50.x | compiles |
| **bridge** | a **learning Ethernet bridge** between the Elite's W5500 (100BASE-TX) and the LAN8651 (10BASE-T1S), `bridge.h` | none, managed over USB serial | compiles |

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
spi <mhz>                  SPI clock 1..25, on reboot (default 12)
ping <ip> [n]              ICMP over the bus
blast <ip> [sec] [bytes]   UDP to port 9, reports offered Mbit/s
reg r|w <mms> <addr> [val] raw LAN8651 register (hex)
save / reboot
```

Settings live in NVS (`t1s` namespace). A fresh board comes up with **PLCA off
(CSMA/CD), 192.168.50.9, SPI at 12 MHz**.

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
node B:  blast 192.168.50.10 10
```

Wire CN1 P to P and N to N (pins 1/4 are P, 2/3 are N). With PLCA on both,
`blast` should approach the bus rate; with `csma` on both it will still work,
with random backoff instead of fixed slots.

A PC can only join with its own T1S interface (e.g. a USB 10BASE-T1S
adapter). With the Linux `lan865x` / `onsemi` T1S drivers the PC side is
set with `ethtool --set-plca-cfg <if> enable on node-id <n> node-cnt <m>`.

## Not done / not verified

- **Nothing has run on hardware.** The pin map is taken from LilyGo's schematic,
  and the driver is Espressif's, but neither has been tested against this HAT.
- **The status LEDs (DIOA0/1) are not configured.** `status` prints PADCTRL
  so its default mux can be seen on real silicon. Which select value turns a
  DIOA pad into an LED output is a data sheet lookup that was not done here,
  and guessing a write is not worth the risk.
- **The 12 MHz default SPI clock is a cautious starting point, not a measurement.**
  Raise it on the bench and watch for parity errors, the way the W5500 clock
  was settled on the W5500 bench.
