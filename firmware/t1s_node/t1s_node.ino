// T1S HAT bring-up and node firmware for the LilyGO T-ETH-Elite.
//
// Brings the HAT's LAN8651 up as an ordinary esp_netif Ethernet interface, so lwIP -- ping,
// UDP, anything else -- runs over 10BASE-T1S unchanged. The MAC-PHY driver is Espressif's
// own (src/lan865x, vendored, see VENDORED.md); this file only does what is specific to this
// board: the pin map (pins.h), the reset line, the IRQ line's pull, PLCA, and a serial
// console for the bench.
//
// Nothing here has run on hardware yet. Every place a first power-up could go wrong prints
// what it saw rather than failing silently -- the chip ID, the SPI error, PLCA as read back.
#include <Arduino.h>
#include <Preferences.h>
#include <esp_eth.h>
#include <esp_event.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <lwip/sockets.h>
#include <ping/ping_sock.h>
#include "pins.h"
#include "src/lan865x/esp_eth_mac_lan865x.h"
#include "src/lan865x/esp_eth_phy_lan865x.h"
#include "bridge.h"

// w5500_spi.h (shared with the W5500 bench firmware) declares this extern; one definition per image.
W5500Spi *gW5500Spi = nullptr;

// The LAN8651 is rated to 25 MHz SCLK. The riser stack puts ~20 mm of header between the
// ESP32 and the HAT, so bring-up starts below that; `spi <mhz>` + `save` + `reboot` raises it.
// The driver checks the parity of every control reply, so a clock the wiring cannot carry
// shows up as "footer parity mismatch" in the log, not as silent corruption.
constexpr uint8_t kDefaultSpiMhz = 12;

// PLCA node ID 255 means "PLCA off, plain CSMA/CD". 0 is the coordinator, which sends the
// BEACON and whose node count sets the cycle; every bus with PLCA needs exactly one.
constexpr uint8_t kPlcaOff = 255;

// node:   this board is a T1S endpoint with its own IP (lwIP on the LAN8651)
// bridge: W5500 100BASE-TX <-> LAN8651 10BASE-T1S learning bridge, no IP (bridge.h)
enum Mode : uint8_t { kModeNode = 0, kModeBridge = 1 };

struct Config {
  uint8_t mode = kModeNode;
  uint8_t plcaId = kPlcaOff;
  uint8_t plcaCount = 8;
  uint8_t spiMhz = kDefaultSpiMhz;
  uint32_t ip = 0;     // 0 = derive from the node ID, see defaultIp()
  uint32_t mask = 0;   // 0 = 255.255.255.0
};

static Config gCfg;
static Preferences gPrefs;
static esp_eth_handle_t gEth = nullptr;
static esp_eth_mac_t *gMac = nullptr;
static esp_netif_t *gNetif = nullptr;
static volatile bool gLinkUp = false;
static volatile uint32_t gEchoCount = 0;
// port 9 (discard) sink: what a peer's `blast` actually delivered here
static volatile uint32_t gSinkPackets = 0;
static volatile uint64_t gSinkBytes = 0;
static volatile uint32_t gSinkT0 = 0, gSinkLast = 0;

// 192.168.50.(10 + id), or .9 with PLCA off. A separate /24 from the W5500 bench
// (192.168.1.x) so the two interfaces can be up together later without a routing question.
static uint32_t defaultIp() {
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
  gPrefs.end();
}

static void onEthEvent(void *, esp_event_base_t, int32_t id, void *) {
  if (id == ETHERNET_EVENT_CONNECTED) { gLinkUp = true; Serial.println("t1s: link up"); }
  if (id == ETHERNET_EVENT_DISCONNECTED) { gLinkUp = false; Serial.println("t1s: link down"); }
}

// ---------------------------------------------------------------- PLCA

static esp_err_t applyPlca() {
  bool enable = gCfg.plcaId != kPlcaOff;
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
  if (en) Serial.printf("plca: on, id %u of %u%s\n", id, cnt, id == 0 ? " (coordinator)" : "");
  else Serial.println("plca: off (CSMA/CD)");
}

// ---------------------------------------------------------------- bring-up

// Read the LAN8651's DEVID (MMS 10, 0x94) with one raw OPEN Alliance TC6 control read, before
// any driver exists, to find out which way round MOSI/MISO really are. LilyGo's own schematic
// and example code agree on MOSI=IO11 / MISO=IO9, but their printed pinout image had IO9/IO11
// swapped (LilyGO-T-ETH-Series issue #100) -- so the firmware checks instead of trusting either.
// Frame: 4-byte header + 4 data + 4 dummy out; 4 dummy + echoed header + register back.
static uint32_t tc6RawReadDevid(int mosi, int miso) {
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
  dev.spics_io_num = kPinT1sCs;
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

  int mosi = kPinT1sMosi, miso = kPinT1sMiso;
  uint32_t probed = tc6RawReadDevid(mosi, miso);
  if (!isLan865x(probed)) {
    const uint32_t swapped = tc6RawReadDevid(miso, mosi);
    Serial.printf("t1s: probe MOSI=IO%d MISO=IO%d -> 0x%08lx, swapped -> 0x%08lx\n", mosi, miso,
                  (unsigned long)probed, (unsigned long)swapped);
    if (isLan865x(swapped)) {
      mosi = kPinT1sMiso;
      miso = kPinT1sMosi;
      Serial.println("t1s: MOSI/MISO are the other way round on this board -- using the swapped mapping");
    }
  } else {
    Serial.printf("t1s: probe OK, DEVID 0x%08lx (MOSI=IO%d MISO=IO%d)\n", (unsigned long)probed, mosi, miso);
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
  Serial.printf("t1s: spi bus: %s\n", esp_err_to_name(err));
  if (err != ESP_OK) return false;
  // Softer edges instead of series damping resistors: ~20 mm of riser stack between the ESP32
  // and the HAT, and the ESP32-S3's default drive (~20 mA) rings on it for no benefit at
  // <= 25 MHz. CAP_1 is ~10 mA. Raise it back with `spi` tests if edges ever look too slow.
  for (int pin : {kPinT1sSclk, mosi, kPinT1sCs}) gpio_set_drive_capability((gpio_num_t)pin, GPIO_DRIVE_CAP_1);

  spi_device_interface_config_t dev = {};
  dev.mode = 0;
  dev.clock_speed_hz = gCfg.spiMhz * 1000 * 1000;
  dev.spics_io_num = kPinT1sCs;
  dev.queue_size = 20;

  // Field by field rather than ETH_LAN865X_DEFAULT_CONFIG: that macro lists its designators
  // out of declaration order, which C accepts and C++ (this file) rejects.
  eth_lan865x_config_t lanCfg = {};
  lanCfg.spi_host_id = SPI3_HOST;
  lanCfg.spi_devcfg = &dev;
  lanCfg.int_gpio_num = kPinT1sIrq;
  lanCfg.poll_period_ms = 0;  // interrupt-driven; the driver rejects both or neither
  lanCfg.custom_spi_driver = ETH_DEFAULT_SPI;
  eth_mac_config_t macCfg = ETH_MAC_DEFAULT_CONFIG();
  eth_phy_config_t phyCfg = ETH_PHY_DEFAULT_CONFIG();
  phyCfg.reset_gpio_num = -1;  // done above

  gMac = esp_eth_mac_new_lan865x(&lanCfg, &macCfg);
  esp_eth_phy_t *phy = esp_eth_phy_new_lan865x(&phyCfg);
  if (!gMac || !phy) { Serial.println("t1s: driver alloc failed"); return false; }

  esp_eth_config_t ethCfg = ETH_DEFAULT_CONFIG(gMac, phy);
  err = esp_eth_driver_install(&ethCfg, &gEth);
  // This is the line that says whether the board works at all: install runs the chip reset,
  // reads DEVID and refuses anything but 0x8650/0x8651. (The driver logs its own "Chip ID
  // verified" through ESP_LOG, which Arduino's default core debug level hides -- so a
  // failure is reported here and a success by the devid line below.) ESP_ERR_TIMEOUT or
  // ESP_ERR_INVALID_CRC means SPI never reached the chip: check the riser, CS on IO0, and
  // try a lower `spi` clock.
  Serial.printf("t1s: driver install: %s\n", esp_err_to_name(err));
  if (err != ESP_OK) return false;
  uint32_t devid = 0;
  if (esp_eth_mac_lan865x_read_reg(gMac, 10, 0x94, &devid) == ESP_OK)
    Serial.printf("t1s: chip LAN%04lx rev %lu\n", (unsigned long)((devid >> 4) & 0xFFFF),
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
  Serial.printf("t1s: mac %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  err = applyPlca();
  Serial.printf("t1s: plca config: %s\n", esp_err_to_name(err));
  printPlca();

  if (!withNetif) {  // bridge mode: bridge.h takes the input path over, nothing for lwIP
    err = esp_eth_start(gEth);
    Serial.printf("t1s: start (no IP, bridge port): %s\n", esp_err_to_name(err));
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
  esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, onEthEvent, nullptr);

  err = esp_eth_start(gEth);
  Serial.printf("t1s: start: %s, ip " IPSTR "\n", esp_err_to_name(err), IP2STR(&ip.ip));
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
    const uint32_t now = millis();
    if (gSinkPackets == 0) gSinkT0 = now;
    gSinkLast = now;
    gSinkPackets = gSinkPackets + 1;
    gSinkBytes = gSinkBytes + n;
  }
}

static void cmdSink(bool reset) {
  const uint32_t pk = gSinkPackets;
  const uint64_t by = gSinkBytes;
  const float secs = pk > 1 ? (gSinkLast - gSinkT0) / 1000.0f : 0.0f;
  // same accounting as blast: payload + 42 B of Ethernet/IP/UDP headers per datagram
  const float mbit = secs > 0 ? (by + 42.0f * pk) * 8 / secs / 1e6f : 0.0f;
  Serial.printf("sink: %lu packets, %llu B in %.1f s = %.2f Mbit/s on the wire\n",
                (unsigned long)pk, (unsigned long long)by, secs, mbit);
  if (reset) { gSinkPackets = 0; gSinkBytes = 0; Serial.println("sink: counters reset"); }
}

// ---------------------------------------------------------------- console commands

static void cmdPing(const char *host, int count) {
  ip_addr_t target = {};
  if (!ipaddr_aton(host, &target)) { Serial.println("ping: bad address"); return; }
  esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
  cfg.target_addr = target;
  cfg.count = count;
  cfg.interval_ms = 200;
  esp_ping_callbacks_t cb = {};
  cb.on_ping_success = [](esp_ping_handle_t h, void *) {
    uint32_t t, seq;
    esp_ping_get_profile(h, ESP_PING_PROF_TIMEGAP, &t, sizeof(t));
    esp_ping_get_profile(h, ESP_PING_PROF_SEQNO, &seq, sizeof(seq));
    Serial.printf("ping: seq %lu %lu ms\n", (unsigned long)seq, (unsigned long)t);
  };
  cb.on_ping_timeout = [](esp_ping_handle_t h, void *) {
    uint32_t seq;
    esp_ping_get_profile(h, ESP_PING_PROF_SEQNO, &seq, sizeof(seq));
    Serial.printf("ping: seq %lu timeout\n", (unsigned long)seq);
  };
  cb.on_ping_end = [](esp_ping_handle_t h, void *) {
    uint32_t tx, rx;
    esp_ping_get_profile(h, ESP_PING_PROF_REQUEST, &tx, sizeof(tx));
    esp_ping_get_profile(h, ESP_PING_PROF_REPLY, &rx, sizeof(rx));
    Serial.printf("ping: %lu sent, %lu replied\n", (unsigned long)tx, (unsigned long)rx);
    esp_ping_delete_session(h);
  };
  esp_ping_handle_t h;
  if (esp_ping_new_session(&cfg, &cb, &h) == ESP_OK) esp_ping_start(h);
}

// Push UDP at a peer's discard port for a few seconds and report what left. That is the
// offered load the MAC-PHY accepted, not what arrived -- `sink` on the far node says that.
static void cmdBlast(const char *host, int seconds, int size) {
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in to = {};
  to.sin_family = AF_INET;
  to.sin_port = htons(9);
  if (!inet_aton(host, &to.sin_addr)) { Serial.println("blast: bad address"); close(s); return; }
  size = constrain(size, 18, 1472);
  static uint8_t buf[1472];
  memset(buf, 0xA5, sizeof(buf));
  uint32_t sent = 0, failed = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < (uint32_t)seconds * 1000) {
    if (sendto(s, buf, size, 0, (sockaddr *)&to, sizeof(to)) == size) sent++;
    else { failed++; delay(1); }  // ENOMEM when TX credits run out: back off, not spin
  }
  close(s);
  const float secs = (millis() - t0) / 1000.0f;
  // 42 = Ethernet + IP + UDP headers, what the bus actually carries per datagram (plus
  // preamble and FCS, which are not counted here).
  Serial.printf("blast: %lu x %d B in %.1f s = %.2f Mbit/s on the wire, %lu refused\n",
                (unsigned long)sent, size, secs, sent * (size + 42) * 8 / secs / 1e6f,
                (unsigned long)failed);
}

static void cmdReg(bool write, const char *args) {
  unsigned mms = 0, addr = 0, val = 0;
  int n = sscanf(args, "%u %x %x", &mms, &addr, &val);
  if (n < 2 || (write && n < 3)) { Serial.println("reg r <mms> <hexaddr> | reg w <mms> <hexaddr> <hexval>"); return; }
  uint32_t v = val;
  esp_err_t err = write ? esp_eth_mac_lan865x_write_reg(gMac, mms, addr, v)
                        : esp_eth_mac_lan865x_read_reg(gMac, mms, addr, &v);
  Serial.printf("reg %s mms %u 0x%04x = 0x%08lx (%s)\n", write ? "w" : "r", mms, addr,
                (unsigned long)v, esp_err_to_name(err));
}

static void cmdStatus() {
  esp_netif_ip_info_t ip = {};
  if (gNetif) esp_netif_get_ip_info(gNetif, &ip);
  Serial.printf("mode: %s\n", gCfg.mode == kModeBridge ? "bridge (W5500 <-> T1S, no IP)" : "node");
  Serial.printf("link: %s  ip " IPSTR "/" IPSTR "  spi %u MHz  echo %lu\n", gLinkUp ? "up" : "down",
                IP2STR(&ip.ip), IP2STR(&ip.netmask), gCfg.spiMhz, (unsigned long)gEchoCount);
  if (gCfg.mode == kModeBridge) bridge::printStats();
  else cmdSink(false);
  if (gEth) printPlca();
  // DEVID (misc 0x94) and PADCTRL (misc 0x88): the chip's identity, and how its DIOA pads --
  // the two LEDs on this board -- are currently muxed. PADCTRL is printed, not written: which
  // select value means "LED" is a data sheet lookup that has not been done against a real part.
  if (gMac) {
    uint32_t v;
    if (esp_eth_mac_lan865x_read_reg(gMac, 10, 0x94, &v) == ESP_OK)
      Serial.printf("devid: LAN%04lx rev %lu\n", (unsigned long)((v >> 4) & 0xFFFF), (unsigned long)(v & 0xF));
    if (esp_eth_mac_lan865x_read_reg(gMac, 10, 0x88, &v) == ESP_OK)
      Serial.printf("padctrl: 0x%08lx (DIOA0 sel %lu, DIOA1 sel %lu)\n", (unsigned long)v,
                    (unsigned long)(v & 3), (unsigned long)((v >> 2) & 3));
  }
}

static void help() {
  Serial.println(
      "status                     mode, link, ip, PLCA as read back, chip id, bridge counters\n"
      "mode node|bridge           T1S endpoint with an IP, or W5500<->T1S bridge (save + reboot)\n"
      "plca <id> [count]          PLCA node id (0 = coordinator) and node count\n"
      "csma                       PLCA off, plain CSMA/CD\n"
      "ip <a.b.c.d> [mask]        static address (default 192.168.50.10+id)\n"
      "spi <mhz>                  SPI clock, 1..25 (applied on reboot)\n"
      "ping <ip> [n]              ICMP over the T1S bus\n"
      "blast <ip> [sec] [bytes]   UDP to port 9, reports offered rate\n"
      "sink [reset]               what arrived on port 9 here (the far end of blast)\n"
      "reg r|w <mms> <addr> [val] raw LAN8651 register (hex addr/val)\n"
      "save / reboot              write config to flash / restart\n"
      "(UDP echo on port 7 and the discard sink on port 9 always run)");
}

static void handleLine(char *line) {
  char *cmd = strtok(line, " ");
  if (!cmd) return;
  char *rest = strtok(nullptr, "");
  if (!rest) rest = (char *)"";
  char a[32] = {}, b[32] = {};
  int n = sscanf(rest, "%31s %31s", a, b);

  if (!strcmp(cmd, "status")) cmdStatus();
  else if (!strcmp(cmd, "sink")) cmdSink(n >= 1 && !strcmp(a, "reset"));
  else if (!strcmp(cmd, "plca") && n >= 1) {
    gCfg.plcaId = atoi(a);
    if (n >= 2) gCfg.plcaCount = atoi(b);
    Serial.printf("plca: %s\n", esp_err_to_name(applyPlca()));
    printPlca();
    Serial.println("(`save` to keep it; the default ip follows the id only after reboot)");
  } else if (!strcmp(cmd, "csma")) {
    gCfg.plcaId = kPlcaOff;
    Serial.printf("plca: %s\n", esp_err_to_name(applyPlca()));
    printPlca();
  } else if (!strcmp(cmd, "ip") && n >= 1) {
    IPAddress ip, mask(255, 255, 255, 0);
    if (!ip.fromString(a) || (n >= 2 && !mask.fromString(b))) { Serial.println("ip: bad address"); return; }
    gCfg.ip = (uint32_t)ip;
    gCfg.mask = (uint32_t)mask;
    esp_netif_ip_info_t info = {};
    info.ip.addr = gCfg.ip;
    info.netmask.addr = gCfg.mask;
    esp_netif_set_ip_info(gNetif, &info);
    cmdStatus();
  } else if (!strcmp(cmd, "spi") && n >= 1) {
    gCfg.spiMhz = constrain(atoi(a), 1, 25);
    Serial.printf("spi: %u MHz after save + reboot\n", gCfg.spiMhz);
  } else if (!strcmp(cmd, "ping") && n >= 1) cmdPing(a, n >= 2 ? atoi(b) : 5);
  else if (!strcmp(cmd, "blast") && n >= 1) {
    int sec = 5, size = 1472;
    sscanf(rest, "%*s %d %d", &sec, &size);
    cmdBlast(a, sec, size);
  } else if (!strcmp(cmd, "reg") && n >= 1 && (a[0] == 'r' || a[0] == 'w')) cmdReg(a[0] == 'w', rest + strlen(a));
  else if (!strcmp(cmd, "mode") && n >= 1) {
    if (!strcmp(a, "bridge")) gCfg.mode = kModeBridge;
    else if (!strcmp(a, "node")) gCfg.mode = kModeNode;
    else { Serial.println("mode node|bridge"); return; }
    Serial.printf("mode: %s after save + reboot\n", a);
  }
  else if (!strcmp(cmd, "save")) { saveConfig(); Serial.println("saved"); }
  else if (!strcmp(cmd, "reboot")) ESP.restart();
  else help();
}

// ---------------------------------------------------------------- Arduino

void setup() {
  Serial.begin(115200);
  delay(1500);  // let USB CDC enumerate so the bring-up log is not lost
  Serial.println("\n== T1S HAT node (LAN8651 on T-ETH-Elite) ==");
  pinMode(kPinBoardLed, OUTPUT);
  loadConfig();
  const bool isBridge = gCfg.mode == kModeBridge;
  if (!t1sStart(!isBridge)) {
    Serial.println("t1s: bring-up FAILED -- console still runs; `spi 4`, `save`, `reboot` to retry slower");
  } else if (isBridge) {
    if (!bridge::start(gEth)) Serial.println("bridge: W5500 side FAILED -- T1S side is up, nothing forwarded");
  } else {
    xTaskCreate(echoTask, "udp_echo", 4096, nullptr, 5, nullptr);
    xTaskCreate(sinkTask, "udp_sink", 4096, nullptr, 5, nullptr);
  }
  help();
}

void loop() {
  static char line[128];
  static size_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { line[len] = 0; handleLine(line); len = 0; }
    else if (len < sizeof(line) - 1) line[len++] = c;
  }
  // Board LED: solid with link, slow blink without -- visible from across the bench.
  digitalWrite(kPinBoardLed, gLinkUp ? HIGH : (millis() / 500) & 1);
  delay(5);
}
