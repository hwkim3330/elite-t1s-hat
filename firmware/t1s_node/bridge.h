// Bridge mode: the Elite's own W5500 (100BASE-TX) and the HAT's LAN8651 (10BASE-T1S) as the two
// ports of a learning Ethernet bridge, with the ESP32-S3 forwarding frames between them.
//
// Why this is the easy mode, not the hard one: both drivers already hand lwIP raw Ethernet
// frames (the IDF W5500 driver runs the chip in MACRAW; the LAN865x is a MAC-PHY), so bridging is
// only re-pointing each driver's input path from lwIP to "transmit on the other port". Neither
// side needs an IP address, and this board has none in bridge mode: it is managed over USB serial.
// lwIP's own bridge (ESP_NETIF_BRIDGE) is not compiled into the Arduino core, and a two-port
// learning bridge is small enough that writing it beats rebuilding the core.
//
// What it is NOT: a media converter at line rate. Every frame crosses two SPI buses and the
// ESP32's RAM, and the T1S side is 10 Mbit/s shared, half duplex. The learning table matters
// for exactly that reason: unicast between two hosts on the 100 Mbit side is never copied onto
// the 10 Mbit bus. Broadcast and multicast are (ARP needs it), so a chatty 100 Mbit segment
// still costs the T1S bus its broadcast load -- put the bridge on an edge port, not a trunk.
//
// Sniff mode (start(t1s, true)) is the same plumbing run one way: every frame seen on the T1S
// bus is copied out of the W5500, nothing from the W5500 is ever put on the bus, and the
// caller keeps the LAN8651 in CSMA with no IP, so this node never transmits on T1S. The bus
// under test sees no extra traffic and no PLCA slot is taken. Wire the W5500 port straight to
// the capture PC: a switch would learn the T1S stations on this port.
//
// One bridge per T1S segment. Two bridges joining the same bus to the same switch fabric make
// an L2 loop, and this bridge runs no spanning tree.
#pragma once
#include <Arduino.h>
#include <esp_eth.h>
#include <esp_eth_mac_spi.h>
#include <esp_mac.h>
#include <driver/spi_master.h>
#include <esp_netif.h>
#include "w5500_spi.h"
#include "net_console.h"

// The Elite's W5500, as measured on the W5500 bench (LilyGo's schematic agrees on these four).
// Polled, not interrupt-driven: that project found no working INT line on its boards, and a
// 1 ms poll is what it measured best with.
constexpr int kPinEthSclk = 48, kPinEthMiso = 47, kPinEthMosi = 21, kPinEthCs = 45;
constexpr int kEthSpiMhz = 40;  // measured ceiling on the W5500 bench: 80 fails, 40 == 60

namespace bridge {

enum Side : uint8_t { kTx = 0, kT1s = 1 };

struct Entry {
  uint8_t mac[6];
  uint8_t side;
  uint32_t seenMs;
};
constexpr int kTable = 64;
constexpr uint32_t kAgeMs = 300000;  // 802.1D default ageing time

static Entry gTable[kTable];
static portMUX_TYPE gLock = portMUX_INITIALIZER_UNLOCKED;
static esp_eth_handle_t gPort[2] = {nullptr, nullptr};

struct Stats {
  volatile uint32_t in[2], fwd[2], local[2], noMem[2], err[2];
};
static Stats gStats = {};
static bool gSniff = false;
static inline void bump(volatile uint32_t &v) { v = v + 1; }  // one writer per counter

static inline bool isGroup(const uint8_t *mac) { return mac[0] & 1; }

// Learn src on `side`; return the side dst was last seen on, or -1.
static int learnAndLookup(const uint8_t *dst, const uint8_t *src, uint8_t side) {
  const uint32_t now = millis();
  int found = -1, freeSlot = -1, oldest = 0;
  portENTER_CRITICAL(&gLock);
  bool learned = false;
  for (int i = 0; i < kTable; i++) {
    Entry &e = gTable[i];
    const bool live = e.seenMs && now - e.seenMs < kAgeMs;
    if (!live) {
      if (freeSlot < 0) freeSlot = i;
      continue;
    }
    if (!learned && memcmp(e.mac, src, 6) == 0) {
      e.side = side;  // a station that moved sides is re-learned at once
      e.seenMs = now;
      learned = true;
    }
    if (found < 0 && memcmp(e.mac, dst, 6) == 0) found = e.side;
    if (gTable[i].seenMs < gTable[oldest].seenMs) oldest = i;
  }
  if (!learned && !isGroup(src)) {
    Entry &e = gTable[freeSlot >= 0 ? freeSlot : oldest];
    memcpy(e.mac, src, 6);
    e.side = side;
    e.seenMs = now ? now : 1;
  }
  portEXIT_CRITICAL(&gLock);
  return found;
}

// Runs in the receiving driver's own task. The buffer is ours and must be freed on every path.
static esp_err_t input(esp_eth_handle_t, uint8_t *buf, uint32_t len, void *priv) {
  const uint8_t side = (uint8_t)(uintptr_t)priv, other = side ^ 1;
  bump(gStats.in[side]);
  if (gSniff && side == kTx) {
    // receive-only analyser: the T1S bus is never written
  } else if (gSniff) {
    const esp_err_t e = esp_eth_transmit(gPort[kTx], buf, len);
    if (e == ESP_OK) bump(gStats.fwd[side]);
    else if (e == ESP_ERR_NO_MEM) bump(gStats.noMem[side]);
    else bump(gStats.err[side]);
  } else if (len >= 14) {
    const int dstSide = learnAndLookup(buf, buf + 6, side);
    if (!isGroup(buf) && dstSide == side) {
      bump(gStats.local[side]);  // both ends on this side: the other bus never sees it
    } else {
      const esp_err_t e = esp_eth_transmit(gPort[other], buf, len);
      if (e == ESP_OK) bump(gStats.fwd[side]);
      else if (e == ESP_ERR_NO_MEM) bump(gStats.noMem[side]);  // LAN8651 out of TX credits: 10 Mbit/s full
      else bump(gStats.err[side]);
    }
  }
  free(buf);
  return ESP_OK;
}

// Install the W5500 driver on SPI2 (not started). Returns nullptr and prints why on failure.
inline esp_eth_handle_t w5500Install() {
  spi_bus_config_t bus = {};
  bus.mosi_io_num = kPinEthMosi;
  bus.miso_io_num = kPinEthMiso;
  bus.sclk_io_num = kPinEthSclk;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = 20000;  // see w5500_spi.h: split reads at the RX buffer wrap
  esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
  Con.printf("bridge: w5500 spi bus: %s\n", esp_err_to_name(err));
  if (err != ESP_OK) return nullptr;

  static spi_device_interface_config_t dev = {};
  dev.mode = 0;
  dev.clock_speed_hz = kEthSpiMhz * 1000 * 1000;
  dev.input_delay_ns = 20;
  dev.spics_io_num = kPinEthCs;
  dev.queue_size = 20;

  // static: the custom SPI driver keeps a pointer to this config
  static eth_w5500_config_t w = ETH_W5500_DEFAULT_CONFIG(SPI2_HOST, &dev);
  w.int_gpio_num = -1;
  w.poll_period_ms = 1;
  w.custom_spi_driver.config = &w;
  w.custom_spi_driver.init = w5500SpiInit;
  w.custom_spi_driver.deinit = w5500SpiDeinit;
  w.custom_spi_driver.read = w5500SpiRead;
  w.custom_spi_driver.write = w5500SpiWrite;
  eth_mac_config_t macCfg = ETH_MAC_DEFAULT_CONFIG();
  eth_phy_config_t phyCfg = ETH_PHY_DEFAULT_CONFIG();
  phyCfg.phy_addr = 1;
  phyCfg.reset_gpio_num = -1;
  esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w, &macCfg);
  esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phyCfg);
  if (!mac || !phy) { Con.println("bridge: w5500 driver alloc failed"); return nullptr; }
  esp_eth_config_t cfg = ETH_DEFAULT_CONFIG(mac, phy);
  esp_eth_handle_t tx = nullptr;
  err = esp_eth_driver_install(&cfg, &tx);
  Con.printf("bridge: w5500 install: %s\n", esp_err_to_name(err));
  if (err != ESP_OK) return nullptr;
  uint8_t m[6];
  esp_read_mac(m, ESP_MAC_ETH);
  m[5] ^= 0x01;  // distinct from the LAN8651's, which took the efuse Ethernet MAC
  esp_eth_ioctl(tx, ETH_CMD_S_MAC_ADDR, m);
  return tx;
}

// W5500 as an ordinary lwIP interface: the 100BASE-TX end of a bench where a second board
// stands in for the PC (`mode tx`). No LAN8651 involved.
static esp_eth_handle_t gNodeTx = nullptr;   // the W5500 in `mode tx`

// Frames a driver handed up, counted on the way to lwIP: the plain answer to "does anything
// arrive at all", with no chip register map to trust. Index 0 = W5500, 1 = LAN8651.
struct RxCount { volatile uint32_t frames, bcast; volatile uint16_t lastType; };
static RxCount gRx[2] = {};
// the last few frames' headers per interface: who sent them, to whom, what type
struct RxHdr { uint32_t ms; uint8_t dst[6], src[6]; uint16_t type, len; };
static RxHdr gRxLog[2][6];
static volatile uint8_t gRxLogN[2] = {0, 0};
static esp_err_t countingInput(esp_eth_handle_t h, uint8_t *buf, uint32_t len, void *netif) {
  RxCount &c = gRx[h == gNodeTx ? 0 : 1];
  c.frames = c.frames + 1;
  const int i = h == gNodeTx ? 0 : 1;
  if (len >= 14) {
    if (buf[0] == 0xFF) c.bcast = c.bcast + 1;
    c.lastType = (buf[12] << 8) | buf[13];
    RxHdr &r = gRxLog[i][gRxLogN[i] % 6];
    r.ms = millis();
    memcpy(r.dst, buf, 6);
    memcpy(r.src, buf + 6, 6);
    r.type = c.lastType;
    r.len = len;
    gRxLogN[i] = gRxLogN[i] + 1;
  }
  return esp_netif_receive((esp_netif_t *)netif, buf, len, nullptr);
}
inline void countInput(esp_eth_handle_t h, esp_netif_t *nif) { esp_eth_update_input_path(h, countingInput, nif); }
inline void printRx(int i, const char *name) {
  Con.printf("rx %s: %lu frames (%lu broadcast), last ethertype 0x%04x\n", name, (unsigned long)gRx[i].frames,
             (unsigned long)gRx[i].bcast, gRx[i].lastType);
}
inline void printRxLog(int i, const char *name) {
  const uint8_t n = gRxLogN[i];
  for (int k = n > 6 ? n - 6 : 0; k < n; k++) {
    const RxHdr &r = gRxLog[i][k % 6];
    Con.printf("rxlog %s: %lu ms  %02x:%02x:%02x:%02x:%02x:%02x -> %02x:%02x:%02x:%02x:%02x:%02x  type 0x%04x  %u B\n",
               name, (unsigned long)r.ms, r.src[0], r.src[1], r.src[2], r.src[3], r.src[4], r.src[5], r.dst[0],
               r.dst[1], r.dst[2], r.dst[3], r.dst[4], r.dst[5], r.type, r.len);
  }
}

// W5500 PHYCFGR as the chip reports it -- the link LED's truth, independent of driver events.
// bit0 LNK, bit1 SPD (1 = 100), bit2 DPX (1 = full). The IDF driver maps every PHY register
// register address is the W5500 map form, offset << 16 (common block = 0).
inline void printPhy() {
  if (!gNodeTx) return;
  uint32_t v = 0;
  esp_eth_phy_reg_rw_data_t rw = {.reg_addr = 0x002E << 16, .reg_value_p = &v};  // W5500_MAKE_MAP(PHYCFGR, common block)
  const esp_err_t e = esp_eth_ioctl(gNodeTx, ETH_CMD_READ_PHY_REG, &rw);
  Con.printf("w5500: PHYCFGR 0x%02lx (%s) -> cable %s, %s, %s duplex\n", (unsigned long)(v & 0xFF),
             esp_err_to_name(e), (v & 1) ? "LINKED" : "NO LINK", (v & 2) ? "100M" : "10M", (v & 4) ? "full" : "half");
}

// Toggle the W5500 PHY's reset bit (PHYCFGR bit 7, active low) with "negotiate everything"
// loaded, as the W5500 LiDAR firmware does: the mode bits only take effect across a reset.
inline void phyReset() {
  if (!gNodeTx) return;
  uint32_t v = 0x78;  // OPMD 1, OPMDC 111, RST asserted
  esp_eth_phy_reg_rw_data_t rw = {.reg_addr = 0x002E << 16, .reg_value_p = &v};
  esp_eth_ioctl(gNodeTx, ETH_CMD_WRITE_PHY_REG, &rw);
  delay(10);
  v = 0xF8;
  esp_eth_ioctl(gNodeTx, ETH_CMD_WRITE_PHY_REG, &rw);
  delay(50);
  printPhy();
}

// One byte of any W5500 register, through our own SPI layer (w5500_spi.h): cmd = register
// offset, addr = block << 3 (read). The driver's PHY-register path only takes PHYCFGR.
inline int w5500Byte(uint16_t offset, uint8_t block) {
  if (!gW5500Spi) return -1;
  uint8_t v = 0;
  return w5500SpiRead(gW5500Spi, offset, (uint32_t)block << 3, &v, 1) == ESP_OK ? v : -1;
}
// Socket 0 runs MACRAW: MR (0x00) should be 0x04 (+0x80 MAC filter), SR (0x03) 0x42 = MACRAW open,
// RX_RSR (0x26) = bytes waiting in the chip, RX_RD (0x28) = how far the driver has read.
inline void printW5500Rx() {
  const int mr = w5500Byte(0x00, 1), sr = w5500Byte(0x03, 1), ir = w5500Byte(0x02, 1);
  const int rsr = (w5500Byte(0x26, 1) << 8) | w5500Byte(0x27, 1);
  const int rd = (w5500Byte(0x28, 1) << 8) | w5500Byte(0x29, 1);
  const int wr = (w5500Byte(0x2A, 1) << 8) | w5500Byte(0x2B, 1);
  Con.printf("w5500 s0: MR 0x%02x SR 0x%02x IR 0x%02x  RX_RSR %d  RX_RD 0x%04x RX_WR 0x%04x\n", mr, sr, ir, rsr, rd, wr);
  const W5500Spi *sp = gW5500Spi;
  if (sp)
    Con.printf("w5500 spi: split %lu failures %lu retry-saved %lu fold-saved %lu lost %lu bounced %lu\n",
               (unsigned long)sp->splitReads, (unsigned long)sp->failures, (unsigned long)sp->savedByRetry,
               (unsigned long)sp->savedByFold, (unsigned long)sp->lost, (unsigned long)sp->bounced);
}

inline esp_netif_t *startNode(uint32_t ip, uint32_t mask) {
  esp_eth_handle_t tx = w5500Install();
  gNodeTx = tx;
  if (!tx) return nullptr;
  esp_netif_config_t nifCfg = ESP_NETIF_DEFAULT_ETH();
  esp_netif_t *nif = esp_netif_new(&nifCfg);
  esp_netif_dhcpc_stop(nif);
  esp_netif_ip_info_t info = {};
  info.ip.addr = ip;
  info.netmask.addr = mask;
  esp_netif_set_ip_info(nif, &info);
  esp_netif_attach(nif, esp_eth_new_netif_glue(tx));
  countInput(tx, nif);
  const esp_err_t err = esp_eth_start(tx);
  Con.printf("w5500: start: %s, ip " IPSTR "\n", esp_err_to_name(err), IP2STR(&info.ip));
  return err == ESP_OK ? nif : nullptr;
}

// Bring the W5500 up with no netif, in promiscuous mode, and join it to an already-running
// LAN865x handle. Returns false and prints why on any failure.
inline bool start(esp_eth_handle_t t1s, bool sniff = false) {
  gSniff = sniff;
  esp_eth_handle_t tx = w5500Install();
  if (!tx) return false;
  esp_err_t err;

  gPort[kTx] = tx;
  gPort[kT1s] = t1s;
  bool on = true;
  for (int s = 0; s < 2; s++) {
    err = esp_eth_ioctl(gPort[s], ETH_CMD_S_PROMISCUOUS, &on);
    Con.printf("bridge: %s promiscuous: %s\n", s == kTx ? "w5500" : "lan8651", esp_err_to_name(err));
    esp_eth_update_input_path(gPort[s], input, (void *)(uintptr_t)s);
  }
  err = esp_eth_start(tx);
  Con.printf("bridge: w5500 start: %s -- %s\n", esp_err_to_name(err),
                sniff ? "sniffing: T1S frames copied out of the W5500, nothing sent on T1S"
                      : "bridging 100BASE-TX <-> 10BASE-T1S");
  return err == ESP_OK;
}

inline void printStats() {
  int learned[2] = {0, 0};
  const uint32_t now = millis();
  portENTER_CRITICAL(&gLock);
  for (auto &e : gTable)
    if (e.seenMs && now - e.seenMs < kAgeMs) learned[e.side]++;
  portEXIT_CRITICAL(&gLock);
  if (gSniff) {
    Con.printf("sniff: T1S frames seen %lu, copied out %lu, dropped(W5500 busy) %lu err %lu; "
                  "100BASE-TX frames ignored %lu\n",
                  (unsigned long)gStats.in[kT1s], (unsigned long)gStats.fwd[kT1s],
                  (unsigned long)gStats.noMem[kT1s], (unsigned long)gStats.err[kT1s],
                  (unsigned long)gStats.in[kTx]);
    return;
  }
  const char *name[2] = {"100BASE-TX -> T1S", "T1S -> 100BASE-TX"};
  for (int s = 0; s < 2; s++)
    Con.printf("bridge %s: in %lu fwd %lu kept-local %lu dropped(no TX room) %lu err %lu\n", name[s],
                  (unsigned long)gStats.in[s], (unsigned long)gStats.fwd[s], (unsigned long)gStats.local[s],
                  (unsigned long)gStats.noMem[s], (unsigned long)gStats.err[s]);
  Con.printf("bridge table: %d stations on 100BASE-TX, %d on T1S\n", learned[kTx], learned[kT1s]);
}

}  // namespace bridge
