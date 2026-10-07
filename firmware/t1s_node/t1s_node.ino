// T1S HAT bring-up and node firmware for the LilyGO T-ETH-Elite.
//
// Brings the HAT's LAN8651 up as an ordinary esp_netif Ethernet interface, so lwIP -- ping,
// UDP, anything else -- runs over 10BASE-T1S unchanged. The MAC-PHY driver is Espressif's
// own (src/lan865x, vendored, see VENDORED.md); this file only does what is specific to this
// board: the pin map (pins.h), the reset line, the IRQ line's pull, PLCA, and a serial
// console for the bench.
//
// Runs on T-ETH-Elite boards (with a third-party LAN8651 HAT; this board is not built yet).
// Every place a first power-up could go wrong prints
// what it saw rather than failing silently -- the chip ID, the SPI error, PLCA as read back.
#include <Arduino.h>
#include <algorithm>
#include <Preferences.h>
#include <esp_eth.h>
#include <esp_event.h>
#include <esp_mac.h>
#include <esp_timer.h>
#include <esp_netif.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <lwip/sockets.h>
#include <ping/ping_sock.h>
#include "pins.h"
#include "src/lan865x/esp_eth_mac_lan865x.h"
#include "src/lan865x/esp_eth_phy_lan865x.h"
#include "bridge.h"
#include "zenoh_t1s.h"
#include "net_console.h"

// w5500_spi.h (shared with the W5500 bench firmware) declares this extern; one definition per image.
W5500Spi *gW5500Spi = nullptr;

// The LAN8651 is rated to 25 MHz SCLK (DS60001734F, Table 9-9), and every frame crosses this link,
// so the clock IS the throughput. The ESP32-S3 divides 80 MHz by an integer and rounds to the
// nearest divider, so asking 25 used to run 80/3 = 26.67 MHz -- out of spec; bring-up now lowers
// the request until the clock that actually runs is <= 25, which on this chip means 20 MHz (80/4).
// Measured with a LAN8651 HAT: 11.43 MHz gives 6.0/6.3 Mbit/s, 20 MHz 8.0/8.4, and the out-of-spec
// 26.67 MHz 9.0/9.5 (to/from the node). 25 stays the default request, and bring-up steps down
// (25 -> 20 -> 12 -> 4) by itself if the driver cannot install at a clock -- the driver checks
// the parity of every control reply, so a clock the wiring cannot carry fails install rather
// than corrupting silently. `spi <mhz>` + `save` sets the starting clock.
constexpr uint8_t kDefaultSpiMhz = 25;
constexpr int kSpiMaxHz = 25 * 1000 * 1000;   // LAN8651 fSCK max

// PLCA node ID 255 means "PLCA off, plain CSMA/CD". 0 is the coordinator, which sends the
// BEACON and whose node count sets the cycle; every bus with PLCA needs exactly one.
constexpr uint8_t kPlcaOff = 255;

// node:   this board is a T1S endpoint with its own IP (lwIP on the LAN8651)
// bridge: W5500 100BASE-TX <-> LAN8651 10BASE-T1S learning bridge, no IP (bridge.h)
// sniff:  receive-only bus analyser -- every T1S frame copied out of the W5500, PLCA off, no
//         IP, never transmits on T1S (the converter's "ID >= 1, count 0" mode, on this board)
// tx:     no LAN8651 at all -- the Elite's W5500 as an ordinary IP endpoint (echo, sink,
//         blast, ping). The second board of a two-ESP bench, standing in for the PC on the
//         converter's 100BASE-TX port.
enum Mode : uint8_t { kModeNode = 0, kModeBridge = 1, kModeSniff = 2, kModeTx = 3 };
static const char *modeName(uint8_t m) {
  return m == kModeBridge ? "bridge" : m == kModeSniff ? "sniff" : m == kModeTx ? "tx" : "node";
}

struct Config {
  uint8_t mode = kModeNode;
  uint8_t plcaId = kPlcaOff;
  uint8_t plcaCount = 8;
  uint8_t spiMhz = kDefaultSpiMhz;
  uint32_t ip = 0;     // 0 = derive from the node ID, see defaultIp()
  uint32_t mask = 0;   // 0 = 255.255.255.0
  uint8_t lan = 1;     // mode node: also bring up the board's own RJ45 (W5500)
  uint32_t lanIp = 0;  // 0 = DHCP
  uint32_t lanMask = 0;
};

static Config gCfg;
static Preferences gPrefs;
static esp_eth_handle_t gEth = nullptr;
static esp_eth_mac_t *gMac = nullptr;
static esp_netif_t *gNetif = nullptr;
static esp_netif_t *gLan = nullptr;   // mode node: the W5500 next to T1S
static volatile bool gLanUp = false;
static volatile bool gLinkUp = false;
static uint8_t gSpiMhzRunning = 0;   // the clock bring-up actually settled on
static int gPinCs = kPinT1sCs;       // CS_N as found at boot: IO0 (Rev C / TSN Lab HAT) or IO8 (Rev D)
static bool hatRevD() { return gPinCs == kPinT1sCsRevD; }
static float gSpiActualMhz = 0;      // what the SPI peripheral really runs: 80 MHz / an integer divider
static volatile uint32_t gEchoCount = 0;
// port 9 (discard) sink: what a peer's `blast` actually delivered here
static volatile uint32_t gSinkPackets = 0;
static volatile uint64_t gSinkBytes = 0;
static volatile uint32_t gSinkT0 = 0, gSinkLast = 0;
static uint32_t gIdentifyUntil = 0;  // `identify`: LED strobes until then
static uint32_t gRebootAtMs = 0;   // a remote `reboot`: delayed so its ack can leave first

// 192.168.50.(10 + id), or .9 with PLCA off. A separate /24 from the W5500 bench
// (192.168.1.x) so the two interfaces can be up together later without a routing question.
static uint32_t defaultIp() {
  // tx mode sits on the bench LAN next to the HAT (192.168.100.65 there), as the PC did
  if (gCfg.mode == kModeTx) return (uint32_t)IPAddress(192, 168, 100, 66);
  const uint8_t last = gCfg.plcaId == kPlcaOff ? 9 : 10 + gCfg.plcaId;
  return (uint32_t)IPAddress(192, 168, 50, last);
}

static void loadConfig() {
  gPrefs.begin("t1s", true);
  gCfg.mode = gPrefs.getUChar("mode", gCfg.mode);
  gCfg.plcaId = gPrefs.getUChar("id", gCfg.plcaId);
  gCfg.plcaCount = gPrefs.getUChar("cnt", gCfg.plcaCount);
  gCfg.spiMhz = gPrefs.getUChar("mhz", gCfg.spiMhz);
  gCfg.ip = gPrefs.getULong("ip", 0);
  gCfg.mask = gPrefs.getULong("mask", 0);
  gCfg.lan = gPrefs.getUChar("lan", gCfg.lan);
  gCfg.lanIp = gPrefs.getULong("lanip", 0);
  gCfg.lanMask = gPrefs.getULong("lanmask", 0);
  gPrefs.end();
}

static void saveConfig() {
  gPrefs.begin("t1s", false);
  gPrefs.putUChar("mode", gCfg.mode);
  gPrefs.putUChar("id", gCfg.plcaId);
  gPrefs.putUChar("cnt", gCfg.plcaCount);
  gPrefs.putUChar("mhz", gCfg.spiMhz);
  gPrefs.putULong("ip", gCfg.ip);
  gPrefs.putULong("mask", gCfg.mask);
  gPrefs.putUChar("lan", gCfg.lan);
  gPrefs.putULong("lanip", gCfg.lanIp);
  gPrefs.putULong("lanmask", gCfg.lanMask);
  gPrefs.end();
}

// Two Ethernet drivers can run at once (T1S + the board's RJ45), so the event says which one.
static void onEthEvent(void *, esp_event_base_t, int32_t id, void *data) {
  const esp_eth_handle_t h = data ? *(esp_eth_handle_t *)data : nullptr;
  const bool lan = gCfg.mode == kModeNode && h && h == bridge::gNodeTx;
  const bool up = id == ETHERNET_EVENT_CONNECTED;
  if (id != ETHERNET_EVENT_CONNECTED && id != ETHERNET_EVENT_DISCONNECTED) return;
  if (lan) gLanUp = up;
  else gLinkUp = up;
  Con.printf("%s: link %s\n", lan ? "lan" : gCfg.mode == kModeTx ? "w5500" : "t1s", up ? "up" : "down");
}

// ---------------------------------------------------------------- PLCA

// A sniffer keeps the saved PLCA settings but never runs them: it must not take a slot.
static bool plcaWanted() { return gCfg.plcaId != kPlcaOff && gCfg.mode != kModeSniff; }

static esp_err_t applyPlca() {
  bool enable = plcaWanted();
  esp_err_t err = ESP_OK;
  if (enable) {
    uint8_t id = gCfg.plcaId, cnt = gCfg.plcaCount;
    if ((err = esp_eth_ioctl(gEth, (esp_eth_io_cmd_t)LAN86XX_ETH_CMD_S_PLCA_ID, &id)) != ESP_OK) return err;
    if ((err = esp_eth_ioctl(gEth, (esp_eth_io_cmd_t)LAN86XX_ETH_CMD_S_PLCA_NCNT, &cnt)) != ESP_OK) return err;
  }
  return esp_eth_ioctl(gEth, (esp_eth_io_cmd_t)LAN86XX_ETH_CMD_S_EN_PLCA, &enable);
}

// What the PHY says, not what was asked for -- the only version worth printing.
static void printPlca() {
  bool en = false;
  uint8_t id = 0, cnt = 0;
  esp_eth_ioctl(gEth, (esp_eth_io_cmd_t)LAN86XX_ETH_CMD_G_EN_PLCA, &en);
  esp_eth_ioctl(gEth, (esp_eth_io_cmd_t)LAN86XX_ETH_CMD_G_PLCA_ID, &id);
  esp_eth_ioctl(gEth, (esp_eth_io_cmd_t)LAN86XX_ETH_CMD_G_PLCA_NCNT, &cnt);
  if (en) Con.printf("plca: on, id %u of %u%s\n", id, cnt, id == 0 ? " (coordinator)" : "");
  else Con.println("plca: off (CSMA/CD)");
}

// ---------------------------------------------------------------- bring-up

// Read the LAN8651's DEVID (MMS 10, 0x94) with one raw OPEN Alliance TC6 control read, before
// any driver exists, to find out which way round MOSI/MISO really are. LilyGo's own schematic
// and example code agree on MOSI=IO11 / MISO=IO9, but their printed pinout image had IO9/IO11
// swapped (LilyGO-T-ETH-Series issue #100) -- so the firmware checks instead of trusting either.
// Frame: 4-byte header + 4 data + 4 dummy out; 4 dummy + echoed header + register back.
static uint32_t tc6RawReadDevid(int mosi, int miso, int cs) {
  spi_bus_config_t bus = {};
  bus.mosi_io_num = mosi;
  bus.miso_io_num = miso;
  bus.sclk_io_num = kPinT1sSclk;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  if (spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_DISABLED) != ESP_OK) return 0;
  spi_device_interface_config_t dev = {};
  dev.mode = 0;
  dev.clock_speed_hz = 4 * 1000 * 1000;  // slow on purpose: this is a wiring test
  dev.spics_io_num = cs;
  dev.queue_size = 1;
  spi_device_handle_t h = nullptr;
  uint32_t devid = 0;
  if (spi_bus_add_device(SPI3_HOST, &dev, &h) == ESP_OK) {
    uint32_t hdr = (0u << 31) | (0u << 30) | (0u << 29) | (1u << 28) | (10u << 24) | (0x94u << 8) | (0u << 1);
    uint32_t ones = __builtin_popcount(hdr >> 1);
    if (!(ones & 1)) hdr |= 1;  // odd parity over bits 31..0
    uint8_t tx[12] = {(uint8_t)(hdr >> 24), (uint8_t)(hdr >> 16), (uint8_t)(hdr >> 8), (uint8_t)hdr};
    uint8_t rx[12] = {};
    spi_transaction_t t = {};
    t.length = sizeof(tx) * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    if (spi_device_polling_transmit(h, &t) == ESP_OK)
      devid = ((uint32_t)rx[8] << 24) | ((uint32_t)rx[9] << 16) | ((uint32_t)rx[10] << 8) | rx[11];
    spi_bus_remove_device(h);
  }
  spi_bus_free(SPI3_HOST);
  return devid;
}

static bool isLan865x(uint32_t devid) {
  const uint32_t model = (devid >> 4) & 0xFFFF;
  return model == 0x8650 || model == 0x8651;
}

static bool t1sStart(bool withNetif) {
  // Deselect the TF card: it shares MOSI/MISO/SCLK with the HAT.
  pinMode(kPinSdCs, OUTPUT);
  digitalWrite(kPinSdCs, HIGH);

  // Hardware reset. The HAT pulls RESET_N up (R4), so this is belt and braces: whatever the
  // GPIO did during the ESP32's own boot, the chip starts from a known state. The driver
  // then does its own software reset on top.
  pinMode(kPinT1sReset, OUTPUT);
  digitalWrite(kPinT1sReset, LOW);
  delay(2);
  digitalWrite(kPinT1sReset, HIGH);
  delay(10);

  esp_netif_init();
  esp_event_loop_create_default();
  gpio_install_isr_service(0);  // the LAN865x driver is interrupt-driven

  // Which CS: IO0 (Rev C, TSN Lab's HAT) or IO8 (Rev D). A CS that reaches no chip reads 0 or
  // all ones, so the first candidate that returns a LAN865x DEVID is the board's.
  int mosi = kPinT1sMosi, miso = kPinT1sMiso;
  uint32_t probed = 0;
  for (int cs : {kPinT1sCs, kPinT1sCsRevD}) {
    probed = tc6RawReadDevid(mosi, miso, cs);
    if (isLan865x(probed)) { gPinCs = cs; break; }
  }
  if (!isLan865x(probed)) {
    uint32_t swapped = 0;
    for (int cs : {kPinT1sCs, kPinT1sCsRevD}) {
      swapped = tc6RawReadDevid(miso, mosi, cs);
      if (isLan865x(swapped)) { gPinCs = cs; break; }
    }
    Con.printf("t1s: probe MOSI=IO%d MISO=IO%d -> 0x%08lx, swapped -> 0x%08lx (CS IO%d)\n", mosi, miso,
                  (unsigned long)probed, (unsigned long)swapped, gPinCs);
    if (isLan865x(swapped)) {
      mosi = kPinT1sMiso;
      miso = kPinT1sMosi;
      Con.println("t1s: MOSI/MISO are the other way round on this board -- using the swapped mapping");
    }
  } else {
    Con.printf("t1s: probe OK, DEVID 0x%08lx (MOSI=IO%d MISO=IO%d CS=IO%d, %s)\n", (unsigned long)probed, mosi,
               miso, gPinCs, hatRevD() ? "HAT Rev D" : "CS on IO0: HAT Rev C / TSN Lab");
  }

  spi_bus_config_t bus = {};
  bus.mosi_io_num = mosi;
  bus.miso_io_num = miso;
  bus.sclk_io_num = kPinT1sSclk;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  // SPI3, not SPI2: SPI2 is what the W5500 firmware already uses, so the
  // two can later run in one image as a T1S <-> 100BASE-TX bridge.
  esp_err_t err = spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO);
  Con.printf("t1s: spi bus: %s\n", esp_err_to_name(err));
  if (err != ESP_OK) return false;
  // Softer edges instead of series damping resistors: ~20 mm of riser stack between the ESP32
  // and the HAT, and the ESP32-S3's default drive (~20 mA) rings on it for no benefit at
  // <= 25 MHz. CAP_1 is ~10 mA. Raise it back with `spi` tests if edges ever look too slow.
  for (int pin : {kPinT1sSclk, mosi, gPinCs}) gpio_set_drive_capability((gpio_num_t)pin, GPIO_DRIVE_CAP_1);

  // Install at the configured clock; on failure free the driver and step down.
  static spi_device_interface_config_t dev = {};   // static: the driver keeps a pointer
  const uint8_t steps[] = {gCfg.spiMhz, 20, 12, 4};
  eth_mac_config_t macCfg = ETH_MAC_DEFAULT_CONFIG();
  eth_phy_config_t phyCfg = ETH_PHY_DEFAULT_CONFIG();
  phyCfg.reset_gpio_num = -1;  // done above
  for (uint8_t mhz : steps) {
    if (mhz > gCfg.spiMhz) continue;           // only ever step down from the configured clock
    // Never above the LAN8651's 25 MHz: walk the request down until the divider it lands on is in spec.
    int hz = mhz * 1000 * 1000;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    while (hz > 1000000 && spi_get_actual_clock(80 * 1000 * 1000, hz, 128) > kSpiMaxHz) hz -= 500000;
    hz = spi_get_actual_clock(80 * 1000 * 1000, hz, 128);
#pragma GCC diagnostic pop
    dev = {};
    dev.mode = 0;
    dev.clock_speed_hz = hz;
    dev.spics_io_num = gPinCs;
    dev.queue_size = 20;
    // Field by field rather than ETH_LAN865X_DEFAULT_CONFIG: that macro lists its designators
    // out of declaration order, which C accepts and C++ (this file) rejects.
    eth_lan865x_config_t lanCfg = {};
    lanCfg.spi_host_id = SPI3_HOST;
    lanCfg.spi_devcfg = &dev;
    lanCfg.int_gpio_num = kPinT1sIrq;
    lanCfg.poll_period_ms = 0;  // interrupt-driven; the driver rejects both or neither
    lanCfg.custom_spi_driver = ETH_DEFAULT_SPI;
    gMac = esp_eth_mac_new_lan865x(&lanCfg, &macCfg);
    esp_eth_phy_t *phy = esp_eth_phy_new_lan865x(&phyCfg);
    if (!gMac || !phy) { Con.println("t1s: driver alloc failed"); return false; }
    esp_eth_config_t ethCfg = ETH_DEFAULT_CONFIG(gMac, phy);
    err = esp_eth_driver_install(&ethCfg, &gEth);
    // This is the line that says whether the board works at all: install runs the chip reset,
    // reads DEVID and refuses anything but 0x8650/0x8651. ESP_ERR_TIMEOUT or ESP_ERR_INVALID_CRC
    // means SPI did not reach the chip at this clock: check the riser and CS (IO0, or IO8 on Rev D).
    Con.printf("t1s: driver install at %u MHz: %s\n", mhz, esp_err_to_name(err));
    if (err == ESP_OK) {
      gSpiMhzRunning = mhz;
      // 25 and 22 asked both run 20 (80/4), 18 and 15 give 16, 12 gives 11.43
      gSpiActualMhz = hz / 1e6f;
      Con.printf("t1s: SPI %u MHz asked, %.2f MHz actual\n", mhz, gSpiActualMhz);
      break;
    }
    gMac->del(gMac);
    phy->del(phy);
    gMac = nullptr;
    gEth = nullptr;
    gpio_isr_handler_remove((gpio_num_t)kPinT1sIrq);
  }
  if (err != ESP_OK) return false;
  uint32_t devid = 0;
  if (esp_eth_mac_lan865x_read_reg(gMac, 10, 0x94, &devid) == ESP_OK)
    Con.printf("t1s: chip LAN%04lx rev %lu\n", (unsigned long)((devid >> 4) & 0xFFFF),
                  (unsigned long)(devid & 0xF));

  // The driver turns the ESP32's internal pull-DOWN on for the IRQ pin (it assumes nothing
  // else is there). This board has a 10k pull-up (R6) on IRQ_N, and ~45k internal down against
  // 10k up parks the idle level near 2.7 V -- above VIH, but for no reason. Undo it.
  gpio_pulldown_dis((gpio_num_t)kPinT1sIrq);
  gpio_pullup_en((gpio_num_t)kPinT1sIrq);

  // The LAN8651 has no MAC address of its own; take the ESP32's Ethernet efuse address so
  // two nodes cannot collide.
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_ETH);
  esp_eth_ioctl(gEth, ETH_CMD_S_MAC_ADDR, mac);
  Con.printf("t1s: mac %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  err = applyPlca();
  Con.printf("t1s: plca config: %s\n", esp_err_to_name(err));
  printPlca();

  if (!withNetif) {  // bridge mode: bridge.h takes the input path over, nothing for lwIP
    err = esp_eth_start(gEth);
    Con.printf("t1s: start (no IP, bridge port): %s\n", esp_err_to_name(err));
    return err == ESP_OK;
  }
  esp_netif_config_t nifCfg = ESP_NETIF_DEFAULT_ETH();
  gNetif = esp_netif_new(&nifCfg);
  esp_netif_dhcpc_stop(gNetif);  // closed bench bus, static addressing
  esp_netif_ip_info_t ip = {};
  ip.ip.addr = gCfg.ip ? gCfg.ip : defaultIp();
  ip.netmask.addr = gCfg.mask ? gCfg.mask : (uint32_t)IPAddress(255, 255, 255, 0);
  esp_netif_set_ip_info(gNetif, &ip);
  esp_netif_attach(gNetif, esp_eth_new_netif_glue(gEth));
  bridge::countInput(gEth, gNetif);
  esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, onEthEvent, nullptr);

  err = esp_eth_start(gEth);
  Con.printf("t1s: start: %s, ip " IPSTR "\n", esp_err_to_name(err), IP2STR(&ip.ip));
  return err == ESP_OK;
}

// ---------------------------------------------------------------- UDP echo (port 7)

// The other end of every test below: anything sent to port 7 comes straight back, so a PC
// or another node can measure round-trip time over the bus with nothing on this side but
// this task.
static void echoTask(void *) {
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in a = {};
  a.sin_family = AF_INET;
  a.sin_port = htons(7);
  bind(s, (sockaddr *)&a, sizeof(a));
  static uint8_t buf[1536];
  for (;;) {
    sockaddr_in from;
    socklen_t fl = sizeof(from);
    int n = recvfrom(s, buf, sizeof(buf), 0, (sockaddr *)&from, &fl);
    if (n > 0) { sendto(s, buf, n, 0, (sockaddr *)&from, fl); gEchoCount = gEchoCount + 1; }
  }
}

// The receiving half of `blast`: count what arrives on port 9 and drop it. `sink` on this
// node then says what the far side's offered load turned into after the bus.
// Sink statistics, written by the sink task only. A `sink reset` is a request the task carries
// out before its next packet, so the console never races it.
struct SinkSeq {
  uint32_t minSeq = UINT32_MAX, maxSeq = 0, last = 0, reordered = 0, dup = 0;
  int64_t tPrev = 0, gapMax = 0, t0us = 0, t1us = 0;
  uint64_t bytesAfterFirst = 0;  // the rate is (everything after the first datagram) / (first..last)
  // inter-arrival gaps: [0..99] 20 us bins up to 2 ms, [100..147] 1 ms bins up to 50 ms, [148] beyond
  uint32_t gapHist[149] = {};
  static int gapBin(int64_t g) { return g < 2000 ? (int)(g / 20) : g < 50000 ? 100 + (int)((g - 2000) / 1000) : 148; }
  static uint32_t binMid(int i) { return i < 100 ? i * 20 + 10 : i < 148 ? 2000 + (i - 100) * 1000 + 500 : 50000; }
  // a member, not a free function: Arduino's prototype pass would declare it before this struct
  uint32_t gapPct(uint32_t total, float p) const {
    uint32_t acc = 0, want = (uint32_t)(total * p);
    for (int i = 0; i < 149; i++) {
      acc += gapHist[i];
      if (acc > want) return binMid(i);
    }
    return 50000;
  }
};
static SinkSeq gSeq;
static volatile bool gSinkResetReq = false;
static uint32_t gSinkSeen[64];  // bitmap of the last 2048 seqs, for duplicates

static void sinkTask(void *) {
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in a = {};
  a.sin_family = AF_INET;
  a.sin_port = htons(9);
  bind(s, (sockaddr *)&a, sizeof(a));
  static uint8_t buf[1536];
  for (;;) {
    int n = recv(s, buf, sizeof(buf), 0);
    if (n <= 0) continue;
    const int64_t tus = esp_timer_get_time();
    if (gSinkResetReq) {
      gSinkPackets = 0;
      gSinkBytes = 0;
      gSeq = SinkSeq();
      memset(gSinkSeen, 0, sizeof(gSinkSeen));
      gSinkResetReq = false;
    }
    const uint32_t now = millis();
    if (gSinkPackets == 0) { gSinkT0 = now; gSeq.t0us = tus; }
    else gSeq.bytesAfterFirst += n;
    gSinkLast = now;
    gSeq.t1us = tus;
    gSinkPackets = gSinkPackets + 1;
    gSinkBytes = gSinkBytes + n;
    // blast puts a sequence number in the first 4 bytes (little endian)
    if (n >= 4) {
      uint32_t seq;
      memcpy(&seq, buf, 4);
      SinkSeq &q = gSeq;
      uint32_t &w = gSinkSeen[(seq >> 5) & 63];
      if (w & (1u << (seq & 31))) q.dup++;
      w |= 1u << (seq & 31);
      gSinkSeen[((seq >> 5) + 32) & 63] = 0;  // clear the window half ahead
      if (gSinkPackets > 1 && seq < q.last) q.reordered++;
      q.last = seq;
      if (seq < q.minSeq) q.minSeq = seq;
      if (seq > q.maxSeq) q.maxSeq = seq;
    }
    if (gSeq.tPrev) {
      const int64_t g = tus - gSeq.tPrev;
      gSeq.gapHist[SinkSeq::gapBin(g)]++;
      if (g > gSeq.gapMax) gSeq.gapMax = g;
    }
    gSeq.tPrev = tus;
  }
}

static void cmdSink(bool reset) {
  const uint32_t pk = gSinkPackets;
  const uint64_t by = gSinkBytes;
  const SinkSeq q = gSeq;
  // n datagrams arrive over n-1 intervals: count the n-1 that arrived after the first, over the
  // first-to-last span (us). Counting all n over that span read ~0.5 % high at 1472 B.
  const double secs = pk > 1 ? (q.t1us - q.t0us) / 1e6 : 0.0;
  // same accounting as blast: payload + 42 B of Ethernet/IP/UDP headers per datagram
  const double mbit = secs > 0 ? (q.bytesAfterFirst + 42.0 * (pk - 1)) * 8 / secs / 1e6 : 0.0;
  Con.printf("sink: %lu packets, %llu B in %.3f s = %.3f Mbit/s on the wire\n",
                (unsigned long)pk, (unsigned long long)by, secs, mbit);
  if (pk && q.maxSeq >= q.minSeq) {
    const uint32_t expect = q.maxSeq - q.minSeq + 1, gaps = pk > 1 ? pk - 1 : 0;
    Con.printf("sinkx: seq %lu..%lu expected %lu got %lu lost %ld reordered %lu dup %lu "
               "gap_us p50 %lu p90 %lu p99 %lu max %lld\n",
               (unsigned long)q.minSeq, (unsigned long)q.maxSeq, (unsigned long)expect, (unsigned long)pk,
               (long)expect - (long)pk + (long)q.dup, (unsigned long)q.reordered, (unsigned long)q.dup,
               (unsigned long)min<int64_t>(q.gapPct(gaps, 0.5f), q.gapMax),
               (unsigned long)min<int64_t>(q.gapPct(gaps, 0.9f), q.gapMax),
               (unsigned long)min<int64_t>(q.gapPct(gaps, 0.99f), q.gapMax), (long long)q.gapMax);
  }
  if (reset) { gSinkResetReq = true; gSinkPackets = 0; gSinkBytes = 0; Con.println("sink: counters reset"); }
}

// Round trip over UDP echo (port 7 on every t1s_node) with the us timer: the latency figure
// `ping` cannot give (its timer is 1 ms). Prints a summary, then every sample, 25 per line.

// ---------------------------------------------------------------- control messages (`w`, `ctl`)
// A live operator input (a G923 wheel read on the PC, fed in over USB as `w` lines) carried as
// small CAN-like messages: 16 B UDP to the peer's echo port 7, either every <period> ms or only
// when the input changes. The echo comes straight back, so the sender measures round trip,
// loss and deadline misses with its own clock -- no clock sync between boards.
struct CtlMsg { uint32_t seq, tUs; uint16_t steer; uint8_t y, z, rz, flags; uint16_t btn; };
static_assert(sizeof(CtlMsg) == 16, "16 B control message");
static volatile uint16_t gWSteer = 32768;
static volatile uint8_t gWY = 255, gWZ = 255, gWRz = 255;
static volatile uint16_t gWBtn = 0;
static volatile uint32_t gWChanges = 0, gWUs = 0, gWLines = 0;
struct CtlRun { char host[16]; int periodMs, secs, deadlineUs; };
static CtlRun gCtl;
static volatile bool gCtlRunning = false, gCtlStop = false;
static TaskHandle_t gCtlTask = nullptr;

static void cmdWheel(const char *rest) {   // `w <steer> <y> <z> <rz> [btn]`: silent, ~1 kHz
  unsigned st = 32768, y = 255, z = 255, rz = 255, btn = 0;
  if (sscanf(rest, "%u %u %u %u %u", &st, &y, &z, &rz, &btn) < 1) return;
  gWLines = gWLines + 1;
  if (st == gWSteer && y == gWY && z == gWZ && rz == gWRz && btn == gWBtn) return;
  gWSteer = st; gWY = y; gWZ = z; gWRz = rz; gWBtn = btn;
  gWUs = (uint32_t)esp_timer_get_time();
  gWChanges = gWChanges + 1;
  if (gCtlTask && gCtlRunning && gCtl.periodMs == 0) xTaskNotifyGive(gCtlTask);
}

// Sender: woken by a hardware timer (periodic) or by a changed `w` line (on change; 100 ms
// heartbeat), so the period does not depend on lwIP's 1 ms-tick select. Receiver: its own task,
// blocking on the echo socket.
static int gCtlSock = -1;
static int32_t *gCtlRtt = nullptr;      // per seq, us; -1 = no echo
static uint32_t *gCtlAge = nullptr;     // input age at send (last `w` change -> send), us
static volatile uint32_t gCtlSent = 0, gCtlEchoed = 0, gCtlLate = 0, gCtlSendFail = 0;
static volatile int64_t gCtlLastEcho = 0, gCtlMaxGap = 0;
static esp_timer_handle_t gCtlTimer = nullptr;
static constexpr int kCtlMax = 200000;

static void ctlTick(void *) { if (gCtlTask) xTaskNotifyGive(gCtlTask); }

static void ctlRecvTask(void *) {
  CtlMsg in;
  timeval tv = {0, 100000};
  setsockopt(gCtlSock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  while (gCtlRunning) {
    if (recv(gCtlSock, &in, sizeof(in), 0) != (int)sizeof(in)) continue;
    const int64_t t = esp_timer_get_time();
    if (in.seq < gCtlSent && gCtlRtt[in.seq] < 0) {
      gCtlRtt[in.seq] = (int32_t)((uint32_t)t - in.tUs);
      gCtlEchoed = gCtlEchoed + 1;
      if (gCtlRtt[in.seq] > gCtl.deadlineUs) gCtlLate = gCtlLate + 1;
      if (gCtlLastEcho && t - gCtlLastEcho > gCtlMaxGap) gCtlMaxGap = t - gCtlLastEcho;
      gCtlLastEcho = t;
    }
  }
  vTaskDelete(nullptr);
}

static void ctlTask(void *) {
  const CtlRun r = gCtl;
  sockaddr_in to = {};
  to.sin_family = AF_INET;
  to.sin_port = htons(7);
  inet_aton(r.host, &to.sin_addr);
  gCtlRtt = (int32_t *)ps_malloc(kCtlMax * 4);
  gCtlAge = (uint32_t *)ps_malloc(kCtlMax * 4);
  gCtlSock = socket(AF_INET, SOCK_DGRAM, 0);
  if (!gCtlRtt || !gCtlAge || gCtlSock < 0) {
    Con.println("ctl: no memory/socket");
    gCtlRunning = false; gCtlTask = nullptr; vTaskDelete(nullptr);
  }
  gCtlSent = gCtlEchoed = gCtlLate = gCtlSendFail = 0;
  gCtlLastEcho = gCtlMaxGap = 0;
  TaskHandle_t rx;
  xTaskCreatePinnedToCore(ctlRecvTask, "ctl_rx", 4096, nullptr, 6, &rx, 1);
  esp_timer_create_args_t ta = {};
  ta.callback = ctlTick;
  ta.name = "ctl";
  esp_timer_create(&ta, &gCtlTimer);
  esp_timer_start_periodic(gCtlTimer, r.periodMs > 0 ? r.periodMs * 1000 : 100000);
  const int64_t t0 = esp_timer_get_time(), tEnd = t0 + (int64_t)r.secs * 1000000;
  uint32_t seen = gWChanges;
  int64_t lastSend = 0;
  CtlMsg m = {};
  while (!gCtlStop && esp_timer_get_time() < tEnd && gCtlSent < (uint32_t)kCtlMax) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
    const int64_t now = esp_timer_get_time();
    if (r.periodMs == 0 && gWChanges == seen && now - lastSend < 95000) continue;   // on change + heartbeat
    seen = gWChanges;
    const uint32_t i = gCtlSent;
    m.seq = i;
    m.tUs = (uint32_t)now;
    m.steer = gWSteer; m.y = gWY; m.z = gWZ; m.rz = gWRz; m.btn = gWBtn;
    m.flags = r.periodMs > 0 ? 1 : 2;
    gCtlRtt[i] = -1;
    gCtlAge[i] = (uint32_t)now - gWUs;
    gCtlSent = i + 1;
    if (sendto(gCtlSock, &m, sizeof(m), 0, (sockaddr *)&to, sizeof(to)) != (int)sizeof(m)) gCtlSendFail = gCtlSendFail + 1;
    lastSend = now;
  }
  esp_timer_stop(gCtlTimer);
  esp_timer_delete(gCtlTimer);
  gCtlTimer = nullptr;
  delay(300);                       // stragglers
  const double secs = (esp_timer_get_time() - t0) / 1e6 - 0.3;
  gCtlRunning = false;              // the receiver exits within its 100 ms timeout
  delay(150);
  close(gCtlSock);
  gCtlSock = -1;
  const uint32_t sent = gCtlSent;
  int32_t *v = (int32_t *)ps_malloc(kCtlMax * 4);
  int m2 = 0;
  for (uint32_t i = 0; i < sent; i++) if (gCtlRtt[i] >= 0) v[m2++] = gCtlRtt[i];
  std::sort(v, v + m2);
  std::sort(gCtlAge, gCtlAge + sent);
  Con.printf("ctl: %s %d ms, %.1f s: sent %lu (%.0f/s) echoed %lu lost %lu late>%d us %lu max echo gap %lld us"
             " send-fail %lu\n",
             r.periodMs > 0 ? "periodic" : "on-change", r.periodMs, secs, (unsigned long)sent, sent / secs,
             (unsigned long)gCtlEchoed, (unsigned long)(sent - gCtlEchoed), r.deadlineUs, (unsigned long)gCtlLate,
             (long long)gCtlMaxGap, (unsigned long)gCtlSendFail);
  if (m2)
    Con.printf("ctl rtt: min %ld p50 %ld p90 %ld p99 %ld p99.9 %ld max %ld us\n", (long)v[0], (long)v[m2 / 2],
               (long)v[m2 * 9 / 10], (long)v[m2 * 99 / 100], (long)v[m2 * 999 / 1000], (long)v[m2 - 1]);
  if (sent)
    Con.printf("ctl age: input change -> send p50 %lu p99 %lu max %lu us; wheel lines %lu, changes %lu\n",
               (unsigned long)gCtlAge[sent / 2], (unsigned long)gCtlAge[sent * 99 / 100],
               (unsigned long)gCtlAge[sent - 1], (unsigned long)gWLines, (unsigned long)gWChanges);
  free(v); free(gCtlRtt); free(gCtlAge);
  gCtlRtt = nullptr; gCtlAge = nullptr;
  gCtlTask = nullptr;
  vTaskDelete(nullptr);
}

static void cmdCtl(const char *rest) {
  char host[16] = "";
  int period = 2, secs = 10, deadlineMs = 5;
  if (sscanf(rest, "%15s %d %d %d", host, &period, &secs, &deadlineMs) < 1) {
    Con.println("ctl <ip> [period ms, 0 = on change] [sec] [deadline ms] | ctl stop");
    return;
  }
  if (!strcmp(host, "stop")) { gCtlStop = true; return; }
  if (gCtlRunning) { Con.println("ctl: already running (`ctl stop`)"); return; }
  in_addr t;
  if (!inet_aton(host, &t)) { Con.println("ctl: bad address"); return; }
  strncpy(gCtl.host, host, sizeof(gCtl.host));
  gCtl.periodMs = constrain(period, 0, 1000);
  gCtl.secs = constrain(secs, 1, 600);
  gCtl.deadlineUs = constrain(deadlineMs, 1, 1000) * 1000;
  gCtlStop = false;
  gCtlRunning = true;
  Con.printf("ctl: %s every %d ms for %d s, deadline %d ms\n", host, gCtl.periodMs, gCtl.secs, deadlineMs);
  // core 1 at the echo task's priority: the console (loop) keeps parsing `w` lines meanwhile
  xTaskCreatePinnedToCore(ctlTask, "ctl", 6144, nullptr, 5, &gCtlTask, 1);
}

// ---------------------------------------------------------------- body events (`evt`)
// Sporadic event messages next to the periodic control stream, the way a body domain mixes them:
// the wheel's paddles become turn signals. `evt <ip> <code>` sends 12 B to the peer's UDP 5006;
// the receiver acts on it (the board LED shows the indicator) and answers with the same bytes,
// so the sender times event -> acknowledged. Codes: 0 off, 1 left, 2 right, 3 hazard (the
// indicator state), 4 horn and 5 headlight flash (one-shot LED patterns over whatever runs).
struct EvtMsg { uint32_t magic, seq, tUs; };
static constexpr uint32_t kEvtMagic = 0x45565431;   // "EVT1" + code in the low byte
static volatile uint8_t gIndicator = 0;
static volatile uint32_t gHornUntil = 0, gFlashUntil = 0;
static const char *evtName(uint8_t c) {
  return c == 1 ? "left" : c == 2 ? "right" : c == 3 ? "hazard" : c == 4 ? "horn" : c == 5 ? "flash" : "off";
}

static void evtTask(void *) {
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in a = {};
  a.sin_family = AF_INET;
  a.sin_port = htons(5006);
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  bind(s, (sockaddr *)&a, sizeof(a));
  EvtMsg m;
  for (;;) {
    sockaddr_in from = {};
    socklen_t fl = sizeof(from);
    if (recvfrom(s, &m, sizeof(m), 0, (sockaddr *)&from, &fl) != (int)sizeof(m)) continue;
    if ((m.magic & 0xFFFFFF00u) != (kEvtMagic & 0xFFFFFF00u)) continue;
    sendto(s, &m, sizeof(m), 0, (sockaddr *)&from, fl);   // ack first, then act
    const uint8_t code = m.magic & 0xFF;
    if (code == 4) gHornUntil = millis() + 600;
    else if (code == 5) gFlashUntil = millis() + 500;
    else if (code <= 3) gIndicator = code;
    Con.printf("evt: %s (from " IPSTR ", seq %lu)\n", evtName(code),
               IP2STR((esp_ip4_addr_t *)&from.sin_addr.s_addr), (unsigned long)m.seq);
  }
}

static void cmdEvt(const char *rest) {
  char host[16] = "";
  unsigned code = 0;
  if (sscanf(rest, "%15s %u", host, &code) < 2 || code > 5) {
    Con.println("evt <ip> <0 off|1 left|2 right|3 hazard|4 horn|5 flash>");
    return;
  }
  static uint32_t seq = 0;
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in to = {};
  to.sin_family = AF_INET;
  to.sin_port = htons(5006);
  if (!inet_aton(host, &to.sin_addr)) { Con.println("evt: bad address"); close(s); return; }
  timeval tv = {0, 50000};
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  EvtMsg m = {kEvtMagic & 0xFFFFFF00u | (code & 0xFF), ++seq, (uint32_t)esp_timer_get_time()}, in;
  sendto(s, &m, sizeof(m), 0, (sockaddr *)&to, sizeof(to));
  long rtt = -1;
  while (recv(s, &in, sizeof(in), 0) == (int)sizeof(in))
    if (in.seq == m.seq) { rtt = (long)((uint32_t)esp_timer_get_time() - m.tUs); break; }
  close(s);
  if (rtt >= 0) Con.printf("evt: %s seq %lu acked in %ld us\n", evtName(code), (unsigned long)m.seq, rtt);
  else Con.printf("evt: %s seq %lu no ack within 50 ms\n", evtName(code), (unsigned long)m.seq);
}

// ---------------------------------------------------------------- time transfer (`sync`)
// Two-way time transfer, the exchange PTP and NTP use, with software timestamps: the asker sends
// t1 (its clock), the answerer stamps t2 on receipt and t3 on reply (its clock), the asker stamps
// t4. offset = ((t2 - t1) + (t3 - t4)) / 2 (answerer minus asker), delay = ((t4 - t1) - (t3 - t2)) / 2.
// Stamps are esp_timer (us) taken in the UDP task, so they include lwIP and, on a W5500 board,
// its 1 ms receive poll: this measures what software time sync over this bench can reach.
struct SyncMsg { uint32_t magic, seq; int64_t t1, t2, t3; };
static constexpr uint32_t kSyncMagic = 0x53594e43;   // "SYNC"
static constexpr uint32_t kSyncFollow = 0x53594e46;  // "SYNF": the answerer's hardware stamps (ns), sent after

// Hardware stamps (`ptp on`): the LAN8651 stamps a frame's ingress and, when asked, its egress
// with its own wall clock (TSU, 25 MHz, 40 ns per tick), seconds:nanoseconds. The driver hooks
// below pick out the sync exchange by UDP port 5007 and file the stamps by sequence number.
static volatile bool gPtpOn = false;
static volatile uint32_t gPtpRx = 0, gPtpTxAsk = 0, gPtpTxGot = 0, gPtpTxMiss = 0, gPtpFu = 0, gPtpRxMiss = 0, gPtpAnyRts = 0;
struct HwStamp { volatile uint32_t seq; volatile int64_t ns; };
static HwStamp gRxStamp[64];
static int64_t tsToNs(uint64_t ts) { return (int64_t)(ts >> 32) * 1000000000LL + (int64_t)(ts & 0xffffffffu); }
// IPv4/UDP frame -> pointer to the UDP payload and the ports, or nullptr
static const uint8_t *udpOf(const uint8_t *f, uint32_t len, uint16_t &sport, uint16_t &dport) {
  if (len < 14 + 20 + 8 || f[12] != 0x08 || f[13] != 0x00) return nullptr;
  const uint8_t *ip = f + 14;
  const uint32_t ihl = (ip[0] & 0x0f) * 4;
  if (ip[9] != 17 || len < 14 + ihl + 8 + 8) return nullptr;
  const uint8_t *u = ip + ihl;
  sport = (u[0] << 8) | u[1];
  dport = (u[2] << 8) | u[3];
  return u + 8;
}
static void onRxStamp(const uint8_t *f, uint32_t len, uint64_t ts) {
  gPtpAnyRts = gPtpAnyRts + 1;
  uint16_t sp, dp;
  const uint8_t *p = udpOf(f, len, sp, dp);
  if (!p || (dp != 5007 && sp != 5007)) return;
  uint32_t magic, seq;
  memcpy(&magic, p, 4);
  memcpy(&seq, p + 4, 4);
  if (magic != kSyncMagic) return;
  gRxStamp[seq & 63].ns = tsToNs(ts);
  gRxStamp[seq & 63].seq = seq;
  gPtpRx = gPtpRx + 1;
}
// Software stamp at the driver (any board, W5500 or LAN8651): taken when the frame comes off SPI, so it
// skips lwIP and the socket task's wake-up. The answerer uses it as t2 when it has no hardware stamp.
static HwStamp gDrvRx[64];
static void onDrvRx(esp_eth_handle_t, const uint8_t *f, uint32_t len) {
  uint16_t sp, dp;
  const uint8_t *p = udpOf(f, len, sp, dp);
  if (!p || dp != 5007) return;
  uint32_t magic, seq;
  memcpy(&magic, p, 4);
  memcpy(&seq, p + 4, 4);
  if (magic != kSyncMagic) return;
  gDrvRx[seq & 63].ns = esp_timer_get_time();   // us
  gDrvRx[seq & 63].seq = seq;
}

static bool wantTxStamp(const uint8_t *f, uint32_t len) {
  uint16_t sp, dp;
  const uint8_t *p = udpOf(f, len, sp, dp);
  if (!p || (sp != 5007 && dp != 5007)) return false;
  uint32_t magic;
  memcpy(&magic, p, 4);
  if (magic != kSyncMagic) return false;   // the reply, not the follow-up
  gPtpTxAsk = gPtpTxAsk + 1;
  return true;
}

static void cmdPtp(const char *a) {
  if (!gMac) { Con.println("ptp: no LAN8651"); return; }
  if (!strcmp(a, "on") || !strcmp(a, "off")) {
    const bool on = !strcmp(a, "on");
    // TSU: 40 ns per 25 MHz tick (DS60001734F 4.5.1); OA_CONFIG0 bit 7 FTSE, bit 6 FTSS (64-bit stamps)
    // FTSE/FTSS are set by the driver at init together with SYNC (writing them later has no effect).
    // The stamp itself comes from the PHY's packet matcher, which signals the SFD on the wire
    // (DS60001734F 4.5.2.2); out of reset it is OFF and matches only gPTP Sync (0x88F710 at nibble 30).
    // Match every frame at the SFD, as Microchip's drivers do: mask 0xFFFFFF, location 0, enable.
    // (The vendored driver has these writes, but on a path this chip revision does not take.)
    esp_eth_mac_lan865x_write_reg(gMac, 1, 0x77, 40);
    const uint16_t mm[][2] = {{0x43, 0x00FF}, {0x44, 0xFFFF}, {0x45, 0}, {0x53, 0x00FF}, {0x54, 0xFFFF}, {0x55, 0},
                              {0x40, on ? 2 : 0}, {0x50, on ? 2 : 0}};   // TXMCTL.TXME, RXMCTL.RXME
    for (auto &r : mm) esp_eth_mac_lan865x_write_reg(gMac, 4, r[0], r[1]);
    esp_eth_mac_lan865x_set_ts_hooks(on ? onRxStamp : nullptr, on ? wantTxStamp : nullptr);
    gPtpOn = on;
  }
  uint32_t c0 = 0, ti = 0, sl = 0, ns = 0;
  esp_eth_mac_lan865x_read_reg(gMac, 0, 0x04, &c0);
  esp_eth_mac_lan865x_read_reg(gMac, 1, 0x77, &ti);
  esp_eth_mac_lan865x_read_reg(gMac, 1, 0x74, &sl);
  esp_eth_mac_lan865x_read_reg(gMac, 1, 0x75, &ns);
  Con.printf("ptp: %s, OA_CONFIG0 0x%08lx (FTSE %lu FTSS %lu), TSU increment %lu ns, wall clock %lu.%09lu s\n",
             gPtpOn ? "hardware stamps on" : "off", (unsigned long)c0, (unsigned long)((c0 >> 7) & 1),
             (unsigned long)((c0 >> 6) & 1), (unsigned long)ti, (unsigned long)sl, (unsigned long)ns);
  uint32_t st = 0;
  esp_eth_mac_lan865x_read_reg(gMac, 0, 0x08, &st);
  Con.printf("ptp: frames with an ingress stamp %lu (sync %lu, not found %lu), egress asked %lu got %lu missed %lu, "
             "follow-ups %lu, STATUS0 0x%08lx\n", (unsigned long)gPtpAnyRts, (unsigned long)gPtpRx,
             (unsigned long)gPtpRxMiss, (unsigned long)gPtpTxAsk, (unsigned long)gPtpTxGot, (unsigned long)gPtpTxMiss,
             (unsigned long)gPtpFu, (unsigned long)st);
}

// egress stamp of the frame just sent (capture register A), or -1
static int64_t takeTxStamp() {
  uint32_t st = 0;
  for (int i = 0; i < 50; i++) {
    if (esp_eth_mac_lan865x_read_reg(gMac, 0, 0x08, &st) == ESP_OK && (st & (1u << 8))) {
      uint32_t hi = 0, lo = 0;
      esp_eth_mac_lan865x_read_reg(gMac, 0, 0x10, &hi);
      esp_eth_mac_lan865x_read_reg(gMac, 0, 0x11, &lo);
      esp_eth_mac_lan865x_write_reg(gMac, 0, 0x08, 1u << 8);   // TTSCAA is write-1-to-clear
      gPtpTxGot = gPtpTxGot + 1;
      return (int64_t)hi * 1000000000LL + lo;
    }
    delayMicroseconds(100);
  }
  gPtpTxMiss = gPtpTxMiss + 1;
  return -1;
}

// ---------------------------------------------------------------- clock servo (`ptp lock`)
// The node disciplines its LAN8651 wall clock to a master's clock, the way ptp4l steers a PHC:
// it asks the master (UDP 5007, the `sync` exchange) 16 times a second, stamping its own request
// and the reply in hardware (t1, t4, ns) against the master's t2/t3 (us); every second it takes the
// exchange with the smallest delay, steps once, then runs a PI loop on the TSU frequency
// (TI + TISUBN: 40 ns per 25 MHz tick in 2^-24 ns units). Offset printed = master - node, ns.
static volatile bool gLockRun = false;
static char gLockHost[16];
static int gLockSecs = 60;
static float gLockKp = 0.2f, gLockKi = 0.02f;   // measured best on this bench (W5500 master): see README

static int64_t tsuNow() {                     // TN first, then TSL (the order the datasheet latches)
  uint32_t ns = 0, sl = 0;
  esp_eth_mac_lan865x_read_reg(gMac, 1, 0x75, &ns);
  esp_eth_mac_lan865x_read_reg(gMac, 1, 0x74, &sl);
  return (int64_t)sl * 1000000000LL + ns;
}
static void tsuStep(int64_t d) {              // add d ns: TA below 1 s, else set seconds/ns directly
  if (d > -1000000000LL && d < 1000000000LL) {
    const uint32_t v = (d < 0 ? 0x80000000u : 0) | (uint32_t)((d < 0 ? -d : d) & 0x3FFFFFFF);
    esp_eth_mac_lan865x_write_reg(gMac, 1, 0x76, v);
  } else {
    const int64_t t = tsuNow() + d;
    esp_eth_mac_lan865x_write_reg(gMac, 1, 0x75, (uint32_t)(t % 1000000000LL));
    esp_eth_mac_lan865x_write_reg(gMac, 1, 0x74, (uint32_t)(t / 1000000000LL));
  }
}
static void tsuFreq(double ppb) {             // increment = 40 ns x (1 + ppb e-9), 2^-24 ns resolution
  const uint64_t inc = (uint64_t)llround(40.0 * 16777216.0 * (1.0 + ppb * 1e-9));
  const uint32_t ti = (uint32_t)(inc >> 24), sub = (uint32_t)(inc & 0xFFFFFF);
  esp_eth_mac_lan865x_write_reg(gMac, 1, 0x6F, ((sub & 0xFF) << 24) | ((sub >> 8) & 0xFFFF));   // TISUBN first
  esp_eth_mac_lan865x_write_reg(gMac, 1, 0x77, ti);
}

// ---------------------------------------------------------------- PPS (`ptp pps`)
// Two ways to put the LAN8651 wall clock on a pin (DS60001734F 4.5.4; register map MMS 10):
//   DIOA4: the dedicated 1PPS (PADCTRL 0x88 A4SEL = 01, PPSCTL 0x239: PPSEN bit 0, PPSDIS bit 1,
//          PPSPW bits 6:2, width (16 * PPSPW + 1) * 40 ns). HAT Rev D routes DIOA4 to TP1 and
//          header 13 (IO40); the firmware counts its edges on IO40 as a self-check.
//   DIOA0: event generator 0 (EG0STNS 0x221, EG0STSECL 0x222, EG0STSECH 0x223, EG0PW 0x224,
//          EG0IT 0x225, EG0CTL 0x226: START 0, STOP 1, AH 2, REP 3, ISREL 4; PADCTRL A0SEL = 01).
//          A 100 ms pulse, easier on a slow scope; no HAT revision routes DIOA0.
// Both make the pin an OUTPUT. Rev C and TSN Lab's HAT tie DIOA to ground, so on any board but
// Rev D (CS found on IO8) the command wants `force`.
static volatile uint32_t gPpsEdges = 0;
static volatile int64_t gPpsLastUs = 0;
static void IRAM_ATTR ppsIsr(void *) {
  gPpsEdges = gPpsEdges + 1;
  gPpsLastUs = esp_timer_get_time();
}

static void cmdPps(const char *rest) {
  char w1[8] = "", w2[8] = "", w3[8] = "";
  sscanf(rest, "%*s %7s %7s %7s", w1, w2, w3);
  if (!gMac) { Con.println("pps: no LAN8651"); return; }
  uint32_t pad = 0;
  esp_eth_mac_lan865x_read_reg(gMac, 10, 0x88, &pad);
  if (!strcmp(w1, "off")) {
    esp_eth_mac_lan865x_write_reg(gMac, 10, 0x239, 1u << 1);           // PPSDIS (stops at the end of the second)
    esp_eth_mac_lan865x_write_reg(gMac, 10, 0x226, 1u << 1);           // EG0 STOP
    esp_eth_mac_lan865x_write_reg(gMac, 10, 0x88, pad & ~0x303u);       // A0SEL, A4SEL = 00: inputs again
    gpio_isr_handler_remove((gpio_num_t)kPinT1sPps);
    Con.println("pps: off, DIOA0/DIOA4 inputs again");
    return;
  }
  if (!strcmp(w1, "") || !strcmp(w1, "status")) {
    Con.printf("pps: PADCTRL 0x%08lx (A0SEL %lu, A4SEL %lu), %s; IO40 edges %lu, last %lld us ago\n",
               (unsigned long)pad, (unsigned long)(pad & 3), (unsigned long)((pad >> 8) & 3),
               hatRevD() ? "HAT Rev D" : "CS on IO0 (DIOA grounded on Rev C / TSN Lab)",
               (unsigned long)gPpsEdges, gPpsLastUs ? (long long)(esp_timer_get_time() - gPpsLastUs) : -1LL);
    return;
  }
  const bool eg0 = !strcmp(w1, "eg0");
  const bool forced = !strcmp(w2, "force");                             // `on force` / `eg0 force`
  if ((strcmp(w1, "on") && !eg0) || (eg0 && !forced) || (!eg0 && !hatRevD() && !forced)) {
    Con.println(eg0 ? "pps: `ptp pps eg0 force` drives DIOA0 -- no HAT revision routes it, Rev C grounds it"
                    : "pps: this board has CS on IO0 (Rev C / TSN Lab HAT), which tie DIOA4 to ground. "
                      "`ptp pps on force` only if you know DIOA4 is free.");
    return;
  }
  if (!eg0) {
    esp_eth_mac_lan865x_write_reg(gMac, 10, 0x239, 1u << 1);            // PPSDIS before changing the width
    delay(1100);                                                         // it stops at the end of the second
    esp_eth_mac_lan865x_write_reg(gMac, 10, 0x239, 31u << 2);           // PPSPW 31: (16*31+1)*40 ns = 19.9 us
    esp_eth_mac_lan865x_write_reg(gMac, 10, 0x88, (pad & ~0x300u) | (1u << 8));   // A4SEL = 1PPS
    esp_eth_mac_lan865x_write_reg(gMac, 10, 0x239, (31u << 2) | 1u);    // PPSEN
    gpio_reset_pin((gpio_num_t)kPinT1sPps);
    gpio_set_direction((gpio_num_t)kPinT1sPps, GPIO_MODE_INPUT);
    gpio_set_intr_type((gpio_num_t)kPinT1sPps, GPIO_INTR_POSEDGE);
    gpio_isr_handler_remove((gpio_num_t)kPinT1sPps);
    gpio_isr_handler_add((gpio_num_t)kPinT1sPps, ppsIsr, nullptr);
    gPpsEdges = 0;
    uint32_t ctl = 0;
    esp_eth_mac_lan865x_read_reg(gMac, 10, 0x239, &ctl);
    Con.printf("pps: 1PPS on DIOA4 (TP1, header 13), 19.9 us wide on each whole second of the LAN8651 clock "
               "(PPSCTL 0x%08lx). `ptp pps` counts the edges seen on IO40.\n", (unsigned long)ctl);
    return;
  }
  const int64_t now = tsuNow();
  const uint64_t start = (uint64_t)(now / 1000000000LL) + 2;              // a whole second, 1-2 s ahead
  esp_eth_mac_lan865x_write_reg(gMac, 10, 0x226, 1u << 1);              // stop anything running
  esp_eth_mac_lan865x_write_reg(gMac, 10, 0x221, 0);                    // start: ns
  esp_eth_mac_lan865x_write_reg(gMac, 10, 0x222, (uint32_t)start);      // seconds low
  esp_eth_mac_lan865x_write_reg(gMac, 10, 0x223, (uint32_t)(start >> 32) & 0xFFFF);
  esp_eth_mac_lan865x_write_reg(gMac, 10, 0x224, 100000000);            // 100 ms high
  esp_eth_mac_lan865x_write_reg(gMac, 10, 0x225, 900000000);            // 900 ms low: period 1 s
  esp_eth_mac_lan865x_write_reg(gMac, 10, 0x88, (pad & ~3u) | 1u);       // A0SEL = event generator 0
  esp_eth_mac_lan865x_write_reg(gMac, 10, 0x226, (1u << 3) | (1u << 2) | 1u);   // REP | AH | START, absolute
  uint32_t ctl = 0;
  esp_eth_mac_lan865x_read_reg(gMac, 10, 0x226, &ctl);
  Con.printf("pps: DIOA0 rising on every whole second of the LAN8651 clock from %llu s (EG0CTL 0x%08lx)\n",
             (unsigned long long)start, (unsigned long)ctl);
}

static void lockTask(void *) {
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in to = {};
  to.sin_family = AF_INET;
  to.sin_port = htons(5007);
  inet_aton(gLockHost, &to.sin_addr);
  timeval tv = {0, 30000};
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  const double kp = gLockKp, ki = gLockKi;    // ppb per ns, per 1 s update; ptp4l's defaults are 0.7 / 0.3
  double integ = 0, freq = 0;
  bool stepped = false;
  uint32_t seq = 0x40000000u;
  tsuFreq(0);
  Con.printf("ptpl: locking the LAN8651 clock to %s for %d s (kp %.2f ki %.3f)\n", gLockHost, gLockSecs, kp, ki);
  for (int sec = 0; sec < gLockSecs && gLockRun; sec++) {
    int64_t bestOff = 0, bestDly = INT64_MAX;
    int ok = 0;
    for (int k = 0; k < 16; k++) {
      SyncMsg m = {kSyncMagic, ++seq, esp_timer_get_time(), 0, 0}, in;
      sendto(s, &m, sizeof(m), 0, (sockaddr *)&to, sizeof(to));
      const int64_t t1 = takeTxStamp();      // our request's egress, hardware
      bool got = false;
      while (recv(s, &in, sizeof(in), 0) == (int)sizeof(in)) {
        if (in.magic == kSyncMagic && in.seq == m.seq) { got = true; break; }
      }
      const HwStamp r = gRxStamp[m.seq & 63];
      int64_t t2 = in.t2 * 1000, t3 = in.t3 * 1000;
      if (got) {   // a master with `ptp on` (a LAN8651) sends its hardware t2/t3 (ns) in a follow-up
        SyncMsg fu;
        timeval tf = {0, 5000};
        setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tf, sizeof(tf));
        while (recv(s, &fu, sizeof(fu), 0) == (int)sizeof(fu)) {
          if (fu.seq != m.seq) continue;
          if (fu.magic == kSyncFollow && fu.t2 >= 0 && fu.t3 >= 0) { t2 = fu.t2; t3 = fu.t3; }
          break;
        }
        setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
      }
      if (got && t1 >= 0 && r.seq == m.seq) {
        const int64_t t4 = r.ns;
        const int64_t off = ((t2 - t1) + (t3 - t4)) / 2, dly = ((t4 - t1) - (t3 - t2)) / 2;
        if (dly < bestDly) { bestDly = dly; bestOff = off; }
        ok++;
      }
      // spread the requests over the far side's 1 ms W5500 receive poll: a fixed whole-ms spacing
      // keeps every request at the same poll phase, and then the min-delay pick has nothing to pick
      delayMicroseconds(esp_random() % 1000);
      delay(54);
    }
    if (!ok) { Con.printf("ptpl: %d s no answers\n", sec); continue; }
    if (!stepped) {                           // first second: jump to the master's time
      tsuStep(bestOff);
      stepped = true;
      Con.printf("ptpl: %d step %lld ns\n", sec, (long long)bestOff);
      continue;
    }
    integ += ki * bestOff;
    freq = kp * bestOff + integ;              // ppb, positive = node runs slow -> speed up
    if (freq > 200000) freq = 200000;
    if (freq < -200000) freq = -200000;
    tsuFreq(freq);
    Con.printf("ptpl: %d offset %lld ns delay %lld ns freq %+.0f ppb (%d/16)\n", sec, (long long)bestOff,
               (long long)bestDly, freq, ok);
  }
  close(s);
  Con.printf("ptpl: done, holding %+.0f ppb\n", freq);
  gLockRun = false;
  vTaskDelete(nullptr);
}

static void cmdPtpLock(const char *rest) {
  char host[16] = "";
  int secs = 60;
  float kp = 0.2f, ki = 0.02f;
  if (sscanf(rest, "%*s %15s %d %f %f", host, &secs, &kp, &ki) < 1) {
    Con.println("ptp lock <master ip> [sec] [kp] [ki] | ptp stop");
    return;
  }
  gLockKp = kp;
  gLockKi = ki;
  if (!gPtpOn) { Con.println("ptpl: run `ptp on` first"); return; }
  if (gLockRun) { Con.println("ptpl: already running"); return; }
  strncpy(gLockHost, host, sizeof(gLockHost));
  gLockSecs = constrain(secs, 3, 3600);
  gLockRun = true;
  xTaskCreatePinnedToCore(lockTask, "ptp_lock", 6144, nullptr, 5, nullptr, 1);
}

static void syncTask(void *) {
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in a = {};
  a.sin_family = AF_INET;
  a.sin_port = htons(5007);
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  bind(s, (sockaddr *)&a, sizeof(a));
  SyncMsg m;
  for (;;) {
    sockaddr_in from = {};
    socklen_t fl = sizeof(from);
    if (recvfrom(s, &m, sizeof(m), 0, (sockaddr *)&from, &fl) != (int)sizeof(m)) continue;
    int64_t t2 = esp_timer_get_time();
    if (m.magic != kSyncMagic) continue;
    const HwStamp dr = gDrvRx[m.seq & 63];
    if (dr.seq == m.seq && dr.ns > 0 && dr.ns <= t2) t2 = dr.ns;   // the driver-level stamp, if it saw it
    m.t2 = t2;
    m.t3 = esp_timer_get_time();
    sendto(s, &m, sizeof(m), 0, (sockaddr *)&from, fl);
    // No software two-step: a t3 taken after the send returned (SPI write and task switch included)
    // scattered more than the one taken just before it -- measured 2026-10-06, sigma 23 vs 14 us.
    if (gPtpOn) {   // like PTP's Follow_Up: the precise times go out after the message they describe
      const int64_t t3h = takeTxStamp();
      const HwStamp r = gRxStamp[m.seq & 63];
      if (r.seq != m.seq) gPtpRxMiss = gPtpRxMiss + 1;
      SyncMsg fu = {kSyncFollow, m.seq, m.t1, r.seq == m.seq ? r.ns : -1, t3h};
      gPtpFu = gPtpFu + 1;
      sendto(s, &fu, sizeof(fu), 0, (sockaddr *)&from, fl);
    }
  }
}

static void cmdSync(const char *rest) {
  char host[16] = "";
  int count = 500, ivMs = 20;
  if (sscanf(rest, "%15s %d %d", host, &count, &ivMs) < 1) { Con.println("sync <ip> [n] [ms]"); return; }
  count = constrain(count, 1, 2000);
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in to = {};
  to.sin_family = AF_INET;
  to.sin_port = htons(5007);
  if (!inet_aton(host, &to.sin_addr)) { Con.println("sync: bad address"); close(s); return; }
  timeval tv = {0, 100000};
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  // per exchange: t1 (asker, us since boot), offset and delay (us); printed after the run
  static int64_t *T1 = nullptr, *HOFF = nullptr;     // PSRAM: 56 KB would crowd internal RAM
  static int32_t *OFF = nullptr, *DLY = nullptr, *HDLY = nullptr;
  if (!T1) {
    T1 = (int64_t *)ps_malloc(2000 * 8); HOFF = (int64_t *)ps_malloc(2000 * 8);
    OFF = (int32_t *)ps_malloc(2000 * 4); DLY = (int32_t *)ps_malloc(2000 * 4); HDLY = (int32_t *)ps_malloc(2000 * 4);
  }
  int hw = 0;
  int got = 0;
  SyncMsg m, in;
  for (int i = 0; i < count; i++) {
    m = {kSyncMagic, (uint32_t)i, esp_timer_get_time(), 0, 0};
    sendto(s, &m, sizeof(m), 0, (sockaddr *)&to, sizeof(to));
    for (;;) {
      if (recv(s, &in, sizeof(in), 0) != (int)sizeof(in)) break;   // timeout: lost
      const int64_t t4 = esp_timer_get_time();
      if (in.seq != m.seq) continue;
      T1[got] = in.t1;
      OFF[got] = (int32_t)(((in.t2 - in.t1) + (in.t3 - t4)) / 2);
      DLY[got] = (int32_t)(((t4 - in.t1) - (in.t3 - in.t2)) / 2);
      HOFF[got] = INT64_MIN;
      // a follow-up with the answerer's hardware stamps, if it runs `ptp on`
      SyncMsg fu;
      timeval tf = {0, 20000};
      setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tf, sizeof(tf));
      while (recv(s, &fu, sizeof(fu), 0) == (int)sizeof(fu)) {
        if (fu.magic != kSyncFollow || fu.seq != m.seq) continue;
        if (fu.t2 >= 0 && fu.t3 >= 0) {   // ns on the LAN8651 clock against us on this board
          HOFF[got] = ((fu.t2 - in.t1 * 1000) + (fu.t3 - t4 * 1000)) / 2;
          HDLY[got] = (int32_t)(((t4 - in.t1) * 1000 - (fu.t3 - fu.t2)) / 2);
          hw++;
        }
        break;
      }
      setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
      got++;
      break;
    }
    if (ivMs > 0) delay(ivMs);
  }
  close(s);
  Con.printf("sync: n %d answered %d (interval %d ms), hardware stamps on %d\n", count, got, ivMs, hw);
  // t (us), offset (us), delay (us) from software stamps; with hardware stamps two more columns:
  // offset (ns, minus the first hardware offset) and delay (ns)
  int64_t h0 = INT64_MIN;
  for (int k = 0; k < got && h0 == INT64_MIN; k++) h0 = HOFF[k];
  char line[300];
  for (int i = 0; i < got; i += 8) {
    int o = snprintf(line, sizeof(line), "syncs:");
    for (int k = i; k < got && k < i + 8; k++) {
      o += snprintf(line + o, sizeof(line) - o, " %lld,%ld,%ld", (long long)(T1[k] - T1[0]), (long)OFF[k], (long)DLY[k]);
      if (HOFF[k] != INT64_MIN) o += snprintf(line + o, sizeof(line) - o, ",%lld,%ld", (long long)(HOFF[k] - h0), (long)HDLY[k]);
    }
    Con.println(line);
  }
  Con.println("sync: done");
}

static void cmdRtt(const char *host, int count, int size, int intervalMs) {
  count = constrain(count, 1, 1000);
  size = constrain(size, 12, 1472);
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in to = {};
  to.sin_family = AF_INET;
  to.sin_port = htons(7);
  if (!inet_aton(host, &to.sin_addr)) { Con.println("rtt: bad address"); close(s); return; }
  timeval tv = {0, 100000};
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  static uint8_t buf[1472], in[1536];
  static int32_t rtt[1000];
  memset(buf, 0x5A, sizeof(buf));
  int lost = 0;
  for (int i = 0; i < count; i++) {
    const uint32_t seq = 0x52540000u + i;
    memcpy(buf, &seq, 4);
    const int64_t t0 = esp_timer_get_time();
    sendto(s, buf, size, 0, (sockaddr *)&to, sizeof(to));
    rtt[i] = -1;
    for (;;) {
      int n = recv(s, in, sizeof(in), 0);
      if (n <= 0) break;  // timeout
      uint32_t got;
      memcpy(&got, in, 4);
      if (got == seq) { rtt[i] = (int32_t)(esp_timer_get_time() - t0); break; }
    }
    if (rtt[i] < 0) lost++;
    if (intervalMs > 0) delay(intervalMs);
  }
  close(s);
  static int32_t sorted[1000];
  int m = 0;
  int64_t sum = 0;
  for (int i = 0; i < count; i++)
    if (rtt[i] >= 0) { sorted[m++] = rtt[i]; sum += rtt[i]; }
  std::sort(sorted, sorted + m);
  if (m)
    Con.printf("rtt: n %d size %d lost %d min %ld avg %lld p50 %ld p90 %ld p99 %ld max %ld us\n", count, size, lost,
               (long)sorted[0], (long long)(sum / m), (long)sorted[m / 2], (long)sorted[m * 9 / 10],
               (long)sorted[m * 99 / 100], (long)sorted[m - 1]);
  else
    Con.printf("rtt: n %d size %d lost %d (no replies)\n", count, size, lost);
  char line[256];
  for (int i = 0; i < count; i += 25) {
    int o = snprintf(line, sizeof(line), "rtts:");
    for (int k = i; k < count && k < i + 25; k++) o += snprintf(line + o, sizeof(line) - o, " %ld", (long)rtt[k]);
    Con.println(line);
  }
}

// ---------------------------------------------------------------- bus counters

// No MAC statistics here on purpose. A first version read a guessed Cadence-GEM statistics
// block (MMS 1, words 0x42..0x69); on a real LAN8651 every value read 0 and the periodic read
// stopped the chip transmitting (2026-10-02). Frame counts come from the driver input path
// instead (`rx t1s` in status), which needs no register map.
static void cmdCounters(bool) {
  if (!gMac) { Con.println("counters: no LAN8651"); return; }
  // TC6 STATUS0/1 (MMS 0): sticky error flags; BUFSTS: TX credits / RX chunks right now;
  // PLCA_STS (MMS 4 0xCA03) bit 15 = PST, beacons are being seen.
  uint32_t st0 = 0, st1 = 0, buf = 0, pst = 0;
  esp_eth_mac_lan865x_read_reg(gMac, 0, 0x08, &st0);
  esp_eth_mac_lan865x_read_reg(gMac, 0, 0x09, &st1);
  esp_eth_mac_lan865x_read_reg(gMac, 0, 0x0B, &buf);
  esp_eth_mac_lan865x_read_reg(gMac, 4, 0xCA03, &pst);
  Con.printf("tc6 status0 0x%08lx status1 0x%08lx  tx credits %lu rx chunks %lu  plca_sts 0x%04lx (%s)\n",
                (unsigned long)st0, (unsigned long)st1, (unsigned long)((buf >> 8) & 0xFF),
                (unsigned long)(buf & 0xFF), (unsigned long)(pst & 0xFFFF),
                !plcaWanted() ? "plca off" : (pst & 0x8000) ? "beacons seen" : "NO beacons");
}

// One line for Zenoh: what the node is running now, read back from the chip.
static void stateLine(char *out, size_t n, const char *head) {
  bool en = false;
  uint8_t id = 0, cnt = 0;
  if (gEth) {
    esp_eth_ioctl(gEth, (esp_eth_io_cmd_t)LAN86XX_ETH_CMD_G_EN_PLCA, &en);
    esp_eth_ioctl(gEth, (esp_eth_io_cmd_t)LAN86XX_ETH_CMD_G_PLCA_ID, &id);
    esp_eth_ioctl(gEth, (esp_eth_io_cmd_t)LAN86XX_ETH_CMD_G_PLCA_NCNT, &cnt);
  }
  esp_netif_ip_info_t ip = {};
  if (gNetif) esp_netif_get_ip_info(gNetif, &ip);
  char plca[24];
  if (en) snprintf(plca, sizeof(plca), "%u/%u", id, cnt);
  else snprintf(plca, sizeof(plca), "off");
  snprintf(out, n, "%s | mode=%s plca=%s spi=%u(saved %u) ip=" IPSTR " link=%s", head, modeName(gCfg.mode), plca,
           gSpiMhzRunning, gCfg.spiMhz, IP2STR(&ip.ip), gLinkUp ? "up" : "down");
}

// ---------------------------------------------------------------- console commands

static void cmdPing(const char *host, int count) {
  ip_addr_t target = {};
  if (!ipaddr_aton(host, &target)) { Con.println("ping: bad address"); return; }
  esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
  cfg.target_addr = target;
  cfg.count = count;
  cfg.interval_ms = 200;
  esp_ping_callbacks_t cb = {};
  cb.on_ping_success = [](esp_ping_handle_t h, void *) {
    uint32_t t, seq;
    esp_ping_get_profile(h, ESP_PING_PROF_TIMEGAP, &t, sizeof(t));
    esp_ping_get_profile(h, ESP_PING_PROF_SEQNO, &seq, sizeof(seq));
    Con.printf("ping: seq %lu %lu ms\n", (unsigned long)seq, (unsigned long)t);
  };
  cb.on_ping_timeout = [](esp_ping_handle_t h, void *) {
    uint32_t seq;
    esp_ping_get_profile(h, ESP_PING_PROF_SEQNO, &seq, sizeof(seq));
    Con.printf("ping: seq %lu timeout\n", (unsigned long)seq);
  };
  cb.on_ping_end = [](esp_ping_handle_t h, void *) {
    uint32_t tx, rx;
    esp_ping_get_profile(h, ESP_PING_PROF_REQUEST, &tx, sizeof(tx));
    esp_ping_get_profile(h, ESP_PING_PROF_REPLY, &rx, sizeof(rx));
    Con.printf("ping: %lu sent, %lu replied\n", (unsigned long)tx, (unsigned long)rx);
    esp_ping_delete_session(h);
  };
  esp_ping_handle_t h;
  if (esp_ping_new_session(&cfg, &cb, &h) == ESP_OK) esp_ping_start(h);
}

// Push UDP at a peer's discard port for a few seconds and report what left. That is the
// offered load the MAC-PHY accepted, not what arrived -- `sink` on the far node says that.
// Each datagram starts with a 32-bit sequence number (little endian), so the receiver can
// count loss and reordering and time the gaps between frames.
static void cmdBlast(const char *host, int seconds, int size, int port = 9, float mbit = 0) {
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in to = {};
  to.sin_family = AF_INET;
  to.sin_port = htons(port);
  if (!inet_aton(host, &to.sin_addr)) { Con.println("blast: bad address"); close(s); return; }
  size = constrain(size, 18, 1472);
  static uint8_t buf[1472];
  memset(buf, 0xA5, sizeof(buf));
  uint32_t sent = 0, failed = 0;
  const uint32_t t0 = millis();
  // mbit > 0: pace to that wire rate. A W5500 at 100 Mbit/s can offer ~15 Mbit/s, more than a
  // 10 Mbit/s T1S bus behind a converter can take, so an unpaced blast measures the
  // converter's buffer, not the bus.
  const int64_t gapUs = mbit > 0 ? (int64_t)((size + 42) * 8 / mbit) : 0;
  int64_t next = esp_timer_get_time();
  while (millis() - t0 < (uint32_t)seconds * 1000) {
    if (gapUs) {
      while (esp_timer_get_time() < next) {
        if (next - esp_timer_get_time() > 2000) vTaskDelay(1);
      }
      next += gapUs;
    }
    memcpy(buf, &sent, 4);
    if (sendto(s, buf, size, 0, (sockaddr *)&to, sizeof(to)) == size) sent++;
    else { failed++; delay(1); }  // ENOMEM when TX credits run out: back off, not spin
  }
  close(s);
  const float secs = (millis() - t0) / 1000.0f;
  // 42 = Ethernet + IP + UDP headers, what the bus actually carries per datagram (plus
  // preamble and FCS, which are not counted here).
  Con.printf("blast: %lu x %d B in %.1f s = %.2f Mbit/s on the wire, %lu refused\n",
                (unsigned long)sent, size, secs, sent * (size + 42) * 8 / secs / 1e6f,
                (unsigned long)failed);
}

static void cmdReg(bool write, const char *args) {
  unsigned mms = 0, addr = 0, val = 0;
  int n = sscanf(args, "%u %x %x", &mms, &addr, &val);
  if (n < 2 || (write && n < 3)) { Con.println("reg r <mms> <hexaddr> | reg w <mms> <hexaddr> <hexval>"); return; }
  if (!gMac) { Con.println("reg: no LAN8651 on this board (mode tx, or bring-up failed)"); return; }
  uint32_t v = val;
  esp_err_t err = write ? esp_eth_mac_lan865x_write_reg(gMac, mms, addr, v)
                        : esp_eth_mac_lan865x_read_reg(gMac, mms, addr, &v);
  Con.printf("reg %s mms %u 0x%04x = 0x%08lx (%s)\n", write ? "w" : "r", mms, addr,
                (unsigned long)v, esp_err_to_name(err));
}

static void cmdStatus() {
  esp_netif_ip_info_t ip = {};
  if (gNetif) esp_netif_get_ip_info(gNetif, &ip);
  Con.printf("mode: %s\n", gCfg.mode == kModeBridge ? "bridge (W5500 <-> T1S, no IP)"
                              : gCfg.mode == kModeSniff ? "sniff (T1S -> W5500 only, PLCA off, never transmits)"
                              : gCfg.mode == kModeTx    ? "tx (W5500 endpoint, no LAN8651)"
                                                        : "node");
  Con.printf("link: %s  ip " IPSTR "/" IPSTR "  spi %u MHz (%.2f actual)  echo %lu\n", gLinkUp ? "up" : "down",
                IP2STR(&ip.ip), IP2STR(&ip.netmask), gSpiMhzRunning, gSpiActualMhz, (unsigned long)gEchoCount);
  if (gCfg.mode == kModeBridge || gCfg.mode == kModeSniff) { bridge::printStats(); bridge::printPhy(); }
  else cmdSink(false);
  if (gCfg.mode == kModeTx) { bridge::printPhy(); bridge::printRx(0, "w5500"); bridge::printW5500Rx(); }
  else if (gCfg.mode == kModeNode) {
    bridge::printRx(1, "t1s");
    if (gLan) {
      esp_netif_ip_info_t li = {};
      esp_netif_get_ip_info(gLan, &li);
      Con.printf("lan: %s  ip " IPSTR "/" IPSTR "%s\n", gLanUp ? "up" : "down", IP2STR(&li.ip), IP2STR(&li.netmask),
                 gCfg.lanIp ? "" : " (DHCP)");
      bridge::printPhy();
      bridge::printRx(0, "lan");
    } else Con.printf("lan: %s\n", gCfg.lan ? "bring-up FAILED" : "off (`lan dhcp` or `lan <ip>`, save, reboot)");
  }
  netConsolePrintStatus();
  if (gEth) printPlca();
  // DEVID (misc 0x94) and PADCTRL (misc 0x88): the chip's identity, and how its DIOA pads are
  // muxed. Printed, never written: A0SEL/A1SEL offer only event capture / event generator
  // (DS60001734F 11.6.3) -- the LAN8651 has no LED function, so Rev C ties the pads to ground.
  if (gMac) {
    uint32_t v;
    if (esp_eth_mac_lan865x_read_reg(gMac, 10, 0x94, &v) == ESP_OK)
      Con.printf("devid: LAN%04lx rev %lu\n", (unsigned long)((v >> 4) & 0xFFFF), (unsigned long)(v & 0xF));
    if (esp_eth_mac_lan865x_read_reg(gMac, 10, 0x88, &v) == ESP_OK)
      Con.printf("padctrl: 0x%08lx (DIOA0 sel %lu, DIOA1 sel %lu)\n", (unsigned long)v,
                    (unsigned long)(v & 3), (unsigned long)((v >> 2) & 3));
  }
}

static void help() {
  Con.println(
      "status                     mode, link, ip, PLCA as read back, chip id, bridge counters\n"
      "mode node|bridge|sniff|tx  T1S endpoint, W5500<->T1S bridge, T1S sniffer, or W5500-only endpoint (save + reboot)\n"
      "plca <id> [count]          PLCA node id (0 = coordinator) and node count\n"
      "csma                       PLCA off, plain CSMA/CD\n"
      "ip <a.b.c.d> [mask]        static address (default 192.168.50.10+id)\n"
      "lan dhcp|<ip> [mask]|off   (mode node) the board's own RJ45 next to T1S, own subnet (save + reboot)\n"
      "spi <mhz>                  SPI clock, 1..25 (applied on reboot)\n"
      "ping <ip> [n]              ICMP over the T1S bus\n"
      "blast <ip> [sec] [bytes] [port] [mbit]  UDP (port 9), seq-numbered; mbit paces it (0 = flat out)\n"
      "sink [reset]               what arrived on port 9 here: rate, seq loss/reorder, arrival gaps\n"
      "rtt <ip> [n] [bytes] [ms]  UDP echo round trip in us (default 100 x 64 B, 5 ms apart)\n"
      "w <steer> <y> <z> <rz> [btn]  latest operator input (the PC's wheel feed; silent)\n"
      "ctl <ip> [ms] [sec] [dl ms]  16 B control messages to <ip>'s echo: every ms, or 0 = on change\n"
      "sync <ip> [n] [ms]         two-way time transfer (PTP/NTP exchange): offset, delay; + hardware if the peer runs ptp on\n"
      "ptp [on|off]               LAN8651 hardware frame stamps (TSU wall clock) for the sync exchange\n"
      "ptp lock <ip> [sec] [kp] [ki] | stop  discipline the LAN8651 clock to <ip>'s (PI, 1 s updates)\n"
      "ptp pps on | off | status  1PPS on DIOA4 (HAT Rev D: TP1, header 13 / IO40); `force` on other boards\n"
      "ptp pps eg0 force          1 Hz, 100 ms on DIOA0 from event generator 0 (no HAT routes DIOA0)\n"
      "evt <ip> <code>            body event to <ip> (0 off 1 left 2 right 3 hazard 4 horn 5 flash), timed to its ack\n"
      "counters                   TC6 status/errors, TX credits, RX chunks, PLCA beacons\n"
      "identify [sec]             strobe the board LED (default 15 s) to find this board\n"
      "phyreset                   (mode tx) reset the W5500 PHY with autonegotiation, print its link\n"
      "rxlog                      the last frames received: source, destination, type (and this board's MACs)\n"
      "capture start [n]|stop|dump  record received frames (us time, 128 B) for a pcap\n"
      "phy auto|10f|10h|100f|100h (mode tx) W5500 link mode\n"
      "promisc on|off             accept every frame on the wire (W5500 in tx mode, LAN8651 otherwise)\n"
      "zenoh [pause|resume|ping <hz>|rtts|blast <s> <B>|sink]  zenoh-pico peer over multicast\n"
      "reg r|w <mms> <addr> [val] raw LAN8651 register (hex addr/val)\n"
      "wifi [<ssid> <pass> | ap | off]  join a network, own AP t1s-<id>, or no radio (on reboot)\n"
      "ota <pass>                 OTA password (default t1s-ota, on reboot)\n"
      "save / reboot              write config to flash / restart\n"
      "(UDP echo on port 7 and the discard sink on port 9 always run in node mode)");
}

static void handleLine(char *line) {
  char *cmd = strtok(line, " ");
  if (!cmd) return;
  char *rest = strtok(nullptr, "");
  if (!rest) rest = (char *)"";
  char a[32] = {}, b[32] = {};
  int n = sscanf(rest, "%31s %31s", a, b);

  if (netConsoleCommand(cmd, a, b, n)) return;
  if (!strcmp(cmd, "status")) cmdStatus();
  else if (!strcmp(cmd, "zenoh")) zenohT1sCommand(rest);
  else if (!strcmp(cmd, "sink")) cmdSink(n >= 1 && !strcmp(a, "reset"));
  else if (!strcmp(cmd, "counters")) cmdCounters(n >= 1 && !strcmp(a, "reset"));
  else if (!strcmp(cmd, "phyreset")) bridge::phyReset();
  else if (!strcmp(cmd, "phy") && n >= 1 && bridge::gNodeTx) {
    // phy auto | 10f | 10h | 100f | 100h: written straight into PHYCFGR (the driver's speed /
    // duplex ioctls did not change it on hardware). OPMDC: 000 10H, 001 10F, 010 100H, 011 100F,
    // 111 all with autonegotiation; the mode loads across a PHY reset (bit 7 low, then high).
    uint8_t opmdc = !strcmp(a, "10h") ? 0 : !strcmp(a, "10f") ? 1 : !strcmp(a, "100h") ? 2 : !strcmp(a, "100f") ? 3 : 7;
    uint8_t v = 0x40 | (opmdc << 3);
    w5500SpiWrite(gW5500Spi, 0x002E, 1 << 2, &v, 1);  // common block, write
    delay(10);
    v |= 0x80;
    w5500SpiWrite(gW5500Spi, 0x002E, 1 << 2, &v, 1);
    delay(2500);
    bridge::printPhy();
  }
  else if (!strcmp(cmd, "capture")) {
    // capture start [n] | stop | status | dump [from] [count]
    if (n >= 1 && !strcmp(a, "start")) {
      const int cnt = n >= 2 ? atoi(b) : 4000;
      Con.printf("capture: %s, up to %d frames\n", bridge::captureStart(cnt) ? "armed" : "no PSRAM", cnt);
    } else if (n >= 1 && !strcmp(a, "stop")) {
      bridge::gCapOn = false;
      Con.printf("capture: stopped, %lu frames, %lu not stored\n", (unsigned long)bridge::gCapN,
                 (unsigned long)bridge::gCapDropped);
    } else if (n >= 1 && !strcmp(a, "dump")) {
      unsigned from = 0, count = 100000;
      sscanf(rest, "%*s %u %u", &from, &count);
      bridge::captureDump(from, count);
    } else {
      Con.printf("capture: %s, %lu frames, %lu not stored\n", bridge::gCapOn ? "running" : "idle",
                 (unsigned long)bridge::gCapN, (unsigned long)bridge::gCapDropped);
    }
  }
  else if (!strcmp(cmd, "rxlog")) {
    bridge::printRxLog(0, "w5500");
    bridge::printRxLog(1, "t1s");
    uint8_t m[6];
    if (gEth) { esp_eth_ioctl(gEth, ETH_CMD_G_MAC_ADDR, m); Con.printf("mac t1s   %02x:%02x:%02x:%02x:%02x:%02x\n", m[0], m[1], m[2], m[3], m[4], m[5]); }
    if (bridge::gNodeTx) { esp_eth_ioctl(bridge::gNodeTx, ETH_CMD_G_MAC_ADDR, m); Con.printf("mac w5500 %02x:%02x:%02x:%02x:%02x:%02x\n", m[0], m[1], m[2], m[3], m[4], m[5]); }
  }
  else if (!strcmp(cmd, "promisc") && n >= 1) {
    // take every frame the wire carries, not only ours + broadcast: tells "nothing arrives"
    // from "the MAC filter dropped it"
    bool on = !strcmp(a, "on");
    esp_eth_handle_t h = gCfg.mode == kModeTx ? bridge::gNodeTx : gEth;
    Con.printf("promisc %s: %s\n", on ? "on" : "off",
               h ? esp_err_to_name(esp_eth_ioctl(h, ETH_CMD_S_PROMISCUOUS, &on)) : "no interface");
  }
  else if (!strcmp(cmd, "identify")) {
    const int secs = n >= 1 ? constrain(atoi(a), 1, 120) : 15;
    gIdentifyUntil = millis() + secs * 1000;
    Con.printf("identify: LED strobing for %d s\n", secs);
  }
  else if (!strcmp(cmd, "plca") && n >= 1) {
    gCfg.plcaId = atoi(a);
    if (n >= 2) gCfg.plcaCount = atoi(b);
    Con.printf("plca: %s\n", esp_err_to_name(applyPlca()));
    printPlca();
    Con.println("(`save` to keep it; the default ip follows the id only after reboot)");
  } else if (!strcmp(cmd, "csma")) {
    gCfg.plcaId = kPlcaOff;
    Con.printf("plca: %s\n", esp_err_to_name(applyPlca()));
    printPlca();
  } else if (!strcmp(cmd, "ip") && n >= 1) {
    IPAddress ip, mask(255, 255, 255, 0);
    if (!ip.fromString(a) || (n >= 2 && !mask.fromString(b))) { Con.println("ip: bad address"); return; }
    gCfg.ip = (uint32_t)ip;
    gCfg.mask = (uint32_t)mask;
    esp_netif_ip_info_t info = {};
    info.ip.addr = gCfg.ip;
    info.netmask.addr = gCfg.mask;
    if (gNetif) esp_netif_set_ip_info(gNetif, &info);
    cmdStatus();
  } else if (!strcmp(cmd, "lan") && n >= 1) {
    IPAddress ip, mask(255, 255, 255, 0);
    if (!strcmp(a, "off")) gCfg.lan = 0;
    else if (!strcmp(a, "dhcp")) { gCfg.lan = 1; gCfg.lanIp = 0; }
    else if (ip.fromString(a) && (n < 2 || mask.fromString(b))) {
      gCfg.lan = 1;
      gCfg.lanIp = (uint32_t)ip;
      gCfg.lanMask = (uint32_t)mask;
      if (gLan) {   // address applies now; on/off needs a reboot
        esp_netif_dhcpc_stop(gLan);
        esp_netif_ip_info_t info = {};
        info.ip.addr = gCfg.lanIp;
        info.netmask.addr = gCfg.lanMask;
        esp_netif_set_ip_info(gLan, &info);
      }
    } else { Con.println("lan dhcp | lan <ip> [mask] | lan off"); return; }
    Con.printf("lan: %s (`save`; on/off after reboot)\n", !gCfg.lan ? "off" : gCfg.lanIp ? a : "dhcp");
  } else if (!strcmp(cmd, "spi") && n >= 1) {
    gCfg.spiMhz = constrain(atoi(a), 1, 25);
    Con.printf("spi: %u MHz after save + reboot\n", gCfg.spiMhz);
  } else if (!strcmp(cmd, "ping") && n >= 1) cmdPing(a, n >= 2 ? atoi(b) : 5);
  else if (!strcmp(cmd, "w")) cmdWheel(rest);
  else if (!strcmp(cmd, "ctl")) cmdCtl(rest);
  else if (!strcmp(cmd, "evt")) cmdEvt(rest);
  else if (!strcmp(cmd, "sync")) cmdSync(rest);
  else if (!strcmp(cmd, "ptp") && !strcmp(a, "lock")) cmdPtpLock(rest);
  else if (!strcmp(cmd, "ptp") && !strcmp(a, "stop")) gLockRun = false;
  else if (!strcmp(cmd, "ptp") && !strcmp(a, "pps")) cmdPps(rest);
  else if (!strcmp(cmd, "ptp")) cmdPtp(a);
  else if (!strcmp(cmd, "rtt") && n >= 1) {
    int cnt = 100, size = 64, iv = 5;
    sscanf(rest, "%*s %d %d %d", &cnt, &size, &iv);
    cmdRtt(a, cnt, size, iv);
  }
  else if (!strcmp(cmd, "blast") && n >= 1) {
    int sec = 5, size = 1472, port = 9;
    float mbit = 0;
    sscanf(rest, "%*s %d %d %d %f", &sec, &size, &port, &mbit);
    cmdBlast(a, sec, size, constrain(port, 1, 65535), mbit);
  } else if (!strcmp(cmd, "reg") && n >= 1 && (a[0] == 'r' || a[0] == 'w')) cmdReg(a[0] == 'w', rest + strlen(a));
  else if (!strcmp(cmd, "mode") && n >= 1) {
    if (!strcmp(a, "bridge")) gCfg.mode = kModeBridge;
    else if (!strcmp(a, "node")) gCfg.mode = kModeNode;
    else if (!strcmp(a, "sniff")) gCfg.mode = kModeSniff;
    else if (!strcmp(a, "tx")) gCfg.mode = kModeTx;
    else { Con.println("mode node|bridge|sniff|tx"); return; }
    Con.printf("mode: %s after save + reboot\n", a);
  }
  else if (!strcmp(cmd, "save")) { saveConfig(); Con.println("saved"); }
  else if (!strcmp(cmd, "reboot")) ESP.restart();
  else help();
}

// A command from t1s/<node>/config (Zenoh). Only settings commands -- a remote `blast` or
// `ping` would hold this task for seconds. The ack is the state read back afterwards, not an
// echo; `reboot` is delayed a second so the ack leaves first.
static void runRemoteConfig(const char *text) {
  static const char *kAllowed[] = {"plca", "csma", "ip", "spi", "mode", "save", "reboot", "status", "counters"};
  char line[100], first[16] = {};
  strncpy(line, text, sizeof(line) - 1);
  line[sizeof(line) - 1] = 0;
  sscanf(line, "%15s", first);
  bool ok = false;
  for (const char *c : kAllowed) ok |= !strcmp(first, c);
  Con.printf("zenoh: config \"%s\"%s\n", line, ok ? "" : " -- refused");
  char ack[200];
  if (!ok) {
    snprintf(ack, sizeof(ack), "refused \"%s\" (allowed: plca csma ip spi mode save reboot status counters)", first);
  } else if (!strcmp(first, "reboot")) {
    stateLine(ack, sizeof(ack), "ok reboot in 1 s");
    gRebootAtMs = millis() + 1000;
  } else {
    handleLine(line);
    char head[48];
    snprintf(head, sizeof(head), "ok %s", first);
    stateLine(ack, sizeof(ack), head);
  }
  zenohT1sAck(ack);
}

// ---------------------------------------------------------------- Arduino

void setup() {
  Serial.begin(115200);
  Con.begin();
  delay(1500);  // let USB CDC enumerate so the bring-up log is not lost
  Con.println("\n== T1S HAT node (LAN8651 on T-ETH-Elite) ==");
  pinMode(kPinBoardLed, OUTPUT);
  loadConfig();
  const bool noIp = gCfg.mode != kModeNode;
  if (gCfg.mode == kModeTx) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, onEthEvent, nullptr);
    gNetif = bridge::startNode(gCfg.ip ? gCfg.ip : defaultIp(),
                               gCfg.mask ? gCfg.mask : (uint32_t)IPAddress(255, 255, 255, 0));
    if (!gNetif) Con.println("w5500: bring-up FAILED");
    else {
      xTaskCreate(echoTask, "udp_echo", 4096, nullptr, 5, nullptr);
      xTaskCreate(sinkTask, "udp_sink", 4096, nullptr, 5, nullptr);
    xTaskCreate(evtTask, "udp_evt", 4096, nullptr, 5, nullptr);
    bridge::gRxTap = onDrvRx;
    xTaskCreate(syncTask, "udp_sync", 4096, nullptr, 6, nullptr);
      // Zenoh peer on the W5500 too, named after the chip: t1s-eth-<last 2 MAC bytes>
      static char zname[24];
      uint8_t m[6];
      esp_read_mac(m, ESP_MAC_WIFI_STA);
      snprintf(zname, sizeof(zname), "t1s-eth-%02x%02x", m[4], m[5]);
      zenohT1sStartTask(&gLinkUp, &gCfg.plcaId, &gCfg.plcaCount, kPlcaOff, zname);
    }
  } else if (!t1sStart(!noIp)) {
    Con.println("t1s: bring-up FAILED -- console still runs; `spi 4`, `save`, `reboot` to retry slower");
  } else if (noIp) {
    if (!bridge::start(gEth, gCfg.mode == kModeSniff))
      Con.println("bridge: W5500 side FAILED -- T1S side is up, nothing forwarded");
  } else {
    // The board's own RJ45 as a second interface (before WiFi: it needs internal DMA RAM)
    if (gCfg.lan) {
      gLan = bridge::startLan(gCfg.lanIp, gCfg.lanMask ? gCfg.lanMask : (uint32_t)IPAddress(255, 255, 255, 0));
      if (!gLan) Con.println("lan: W5500 bring-up FAILED -- T1S is unaffected");
    }
    xTaskCreate(echoTask, "udp_echo", 4096, nullptr, 5, nullptr);
    xTaskCreate(sinkTask, "udp_sink", 4096, nullptr, 5, nullptr);
    xTaskCreate(evtTask, "udp_evt", 4096, nullptr, 5, nullptr);
    bridge::gRxTap = onDrvRx;
    xTaskCreate(syncTask, "udp_sync", 4096, nullptr, 6, nullptr);
    // Zenoh over T1S in its own task (a no-op unless built with T1S_WITH_ZENOH)
    zenohT1sStartTask(&gLinkUp, &gCfg.plcaId, &gCfg.plcaCount, kPlcaOff);
  }
  // WiFi console + OTA last (tasks created after WiFi starts can fail for lack of internal RAM).
  // Named by the efuse MAC's last two bytes, so two boards never share a hostname.
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char tag[8];
  snprintf(tag, sizeof(tag), "%02x%02x", mac[4], mac[5]);
  netConsoleBegin(tag);
  help();
}

void loop() {
  static char line[128];
  static size_t len = 0;
  // USB serial and the WiFi console feed the same line editor
  for (;;) {
    int ci = Serial.available() ? Serial.read() : netConsoleRead();
    if (ci < 0) break;
    const char c = (char)ci;
    if (c == '\r') continue;
    if (c == '\n') { line[len] = 0; handleLine(line); len = 0; }
    else if (len < sizeof(line) - 1) line[len++] = c;
  }
  char remote[100];
  if (zenohT1sTakeConfig(remote, sizeof(remote))) runRemoteConfig(remote);
  if (gRebootAtMs && (int32_t)(millis() - gRebootAtMs) >= 0) ESP.restart();
  // No background register polling beyond PLCA_STS: an automatic read of the guessed GEM
  // statistics block every 5 s stopped this LAN8651 from transmitting (2026-10-02, on hardware).
  // Board LED (the Elite's own, IO38) shows T1S state -- the LAN8651 has no LED function
  // of its own (its DIOA pins are event capture/generator only):
  //   PLCA on:  solid = beacons seen (PLCA_STS.PST), fast blink = no beacons
  //   PLCA off: solid = link up
  //   no link:  slow blink
  static uint32_t tPst = 0;
  static bool pst = false;
  const bool plcaOn = plcaWanted();
  if (plcaOn && gMac && millis() - tPst > 250) {
    tPst = millis();
    uint32_t v = 0;
    if (esp_eth_mac_lan865x_read_reg(gMac, 4, 0xCA03, &v) == ESP_OK) pst = v & 0x8000;
  }
  bool on;
  if (gIdentifyUntil && (int32_t)(millis() - gIdentifyUntil) < 0) on = (millis() / 60) & 1;  // ~8 Hz strobe
  // one-shots from `evt` over everything else: horn = 12 Hz strobe 0.6 s, flash = two long blinks
  else if ((int32_t)(millis() - gHornUntil) < 0) on = (millis() / 40) & 1;
  else if ((int32_t)(millis() - gFlashUntil) < 0) on = ((gFlashUntil - millis()) / 125) & 1;
  // turn signal from `evt`: left = 1.5 Hz, right = 1.5 Hz with a double flash, hazard = 3 Hz
  else if (gIndicator == 1) on = (millis() % 667) < 333;
  else if (gIndicator == 2) on = (millis() % 667) < 90 || ((millis() % 667) > 180 && (millis() % 667) < 270);
  else if (gIndicator == 3) on = (millis() % 333) < 166;
  else if (!gLinkUp) on = (millis() / 500) & 1;
  else if (plcaOn) on = pst || ((millis() / 125) & 1);
  else on = true;
  digitalWrite(kPinBoardLed, on ? HIGH : LOW);
  delay(1);   // was 5: `w` lines from the PC's wheel feed come at ~1 kHz and their age is measured
}
