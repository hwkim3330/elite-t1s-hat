// Where the T1S HAT's six host signals land on the T-ETH-Elite's ESP32-S3.
//
// Read off LilyGo's own schematic (schematic/T-ETH-ELite.pdf, sheet 2, J1), not the Pi
// pinout: the HAT was laid out to TSN Lab's Raspberry Pi device-tree overlay, so each
// signal sits on the header pin a Pi would use, and this table is what the Elite has
// wired to those pins.
//
//   HAT signal      header pin   Pi name          Elite
//   SDI  (MOSI)     19           GPIO10 MOSI      IO11  SPI_MOSI  (shared with the TF card)
//   SDO  (MISO)     21           GPIO9  MISO      IO9   SPI_MISO  (shared with the TF card)
//   SCLK            23           GPIO11 SCLK      IO10  SPI_SCLK  (shared with the TF card)
//   CS_N            24           GPIO8  CE0       IO0   BOOT      <-- strapping pin, see below
//   CS_N (Rev D)    27           GPIO0  ID_SD     IO8             moved off the strap
//   1PPS (Rev D)    13           GPIO27           IO40            LAN8651 DIOA4, also test pad TP1
//   IRQ_N           16           GPIO23           IO39
//   RESET_N         15           GPIO22           IO42
//
// CS_N on IO0. That is where the Pi convention puts CE0, and on the Elite pin 24 is the
// BOOT line. It works: the strap wants IO0 high at reset, the Elite pulls it up (R1 10k)
// and so does the HAT (R5 10k), and the LAN8651's CS_N is an input, so nothing on the HAT
// can pull it low. Two consequences that are real, though:
//   - pressing the Elite's BOOT button while running asserts CS_N and corrupts whatever
//     transaction is on the wire. The driver sees a parity error and retries; do not
//     press it on a live bus.
//   - holding BOOT at power-up still enters the ROM downloader as usual. Flashing is
//     unaffected.
//
// Rev D of this HAT moved CS_N to header 27 (IO8) and brings the LAN8651's dedicated 1PPS
// (DIOA4) to header 13 (IO40). The firmware reads DEVID on each CS candidate at boot, so
// one image runs on either board, and it knows from that which board it is on.
//
// The three bus pins are the Elite's general SPI, which also runs the TF card slot
// (CS IO12). The card is deselected at boot (kPinSdCs driven high) so it cannot answer
// on the LAN8651's transactions; using the card at the same time would need a shared
// spi bus handle, which this firmware does not do.
//
// The Elite's own W5500 is on a separate bus (IO47/21/48/45) and is untouched here.
#pragma once

constexpr int kPinT1sMosi = 11;
constexpr int kPinT1sMiso = 9;
constexpr int kPinT1sSclk = 10;
constexpr int kPinT1sCs = 0;      // Rev C / TSN Lab HAT
constexpr int kPinT1sCsRevD = 8;  // Rev D
constexpr int kPinT1sPps = 40;    // Rev D: DIOA4 1PPS in
constexpr int kPinT1sIrq = 39;
constexpr int kPinT1sReset = 42;

constexpr int kPinSdCs = 12;    // TF card chip select on the same bus -- held high
constexpr int kPinBoardLed = 38; // Elite's green LED (header pin 18)
