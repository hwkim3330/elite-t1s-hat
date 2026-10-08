#pragma once
#include <stddef.h>
#include <stdint.h>
// Zenoh-pico over the T1S netif (see zenoh_t1s.cpp). Stubs when zenoh-pico is not built in.
void zenohT1sLoop(bool netUp, int plcaId, int plcaCount);
void zenohT1sPrintStatus();
bool zenohT1sAvailable();
// Run the loop in its own FreeRTOS task, so a blocked send (e.g. no transmit opportunity on the
// bus) can never stall the serial console. The pointers are read on every pass.
// name: fixed node name for the keys, or nullptr for "t1s-hat-<PLCA id>".
void zenohT1sStartTask(volatile bool *netUp, const uint8_t *plcaId, const uint8_t *plcaCount, uint8_t plcaOff,
                       const char *name = nullptr);
// `zenoh ...` console subcommands: status | ping <hz> | rtts [reset] | blast <sec> <bytes> | sink [reset]
void zenohT1sCommand(const char *args);
// Remote configuration: a command received on t1s/<node>/config, taken by the console task.
bool zenohT1sTakeConfig(char *out, size_t n);
// The console task's answer, published on t1s/<node>/config/ack by the zenoh task.
void zenohT1sAck(const char *text);
// `tele`'s Zenoh part: up, paused, sent, pongs, last rtt us, peers, name:pings:ago_ms,...
void zenohT1sTele(char *out, size_t n);
// Generic keys for other modules (ZoneLink: zonelink.h). Register before zenohT1sStartTask; they are declared when
// the session opens. prio = Zenoh priority 1 RealTime .. 7 Background. Puts before the session is up return false.
typedef void (*ZenohT1sCb)(const char *key, size_t keyLen, const uint8_t *payload, size_t n);
int zenohT1sAddPub(const char *key, int prio);
bool zenohT1sPut(int pub, const void *payload, size_t n);
void zenohT1sAddSub(const char *keyexpr, ZenohT1sCb cb);
// `zenoh router <locator>|none` (saved): client of a zenohd (e.g. udp/192.168.100.70:7447) instead of the multicast
// peer; the test traffic (signal, ping, hello, stats) is then off, only registered keys flow.
