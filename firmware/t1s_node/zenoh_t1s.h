#pragma once
#include <stdint.h>
// Zenoh-pico over the T1S netif (see zenoh_t1s.cpp). Stubs when zenoh-pico is not built in.
void zenohT1sLoop(bool netUp, int plcaId, int plcaCount);
void zenohT1sPrintStatus();
bool zenohT1sAvailable();
// Run the loop in its own FreeRTOS task, so a blocked send (e.g. no transmit opportunity on the
// bus) can never stall the serial console. The pointers are read on every pass.
void zenohT1sStartTask(volatile bool *netUp, const uint8_t *plcaId, const uint8_t *plcaCount, uint8_t plcaOff);
