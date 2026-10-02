// WiFi side of the node: a TCP console (port 23) and ArduinoOTA, so a bench of several
// boards can be run and reflashed without a USB cable to each.
//
// Everything the firmware prints goes through `Con`, which writes to USB serial at once and
// queues a copy for the TCP client (the network task drains it, so any task may print).
// Bytes typed on the TCP console come back through netConsoleRead().
#pragma once
#include <Arduino.h>

class ConsoleTee : public Print {
 public:
  size_t write(uint8_t c) override;
  size_t write(const uint8_t *buf, size_t n) override;
  void begin();  // call first in setup(): creates the line lock
};
extern ConsoleTee Con;

// Read the WiFi/OTA settings from NVS and start the network task. Call LAST in setup(): on this
// part, tasks created after WiFi starts can fail for lack of internal RAM.
// `tag` names the board on the network: hostname "t1s-<tag>", AP "t1s-<tag>".
void netConsoleBegin(const char *tag);
// Next byte typed on the TCP console, or -1.
int netConsoleRead();
// `wifi` / `ota` console commands. Return true if the command was theirs.
bool netConsoleCommand(const char *cmd, const char *a, const char *b, int n);
void netConsolePrintStatus();
