#pragma once
// Zenoh-pico over the T1S netif (see zenoh_t1s.cpp). Stubs when zenoh-pico is not built in.
void zenohT1sLoop(bool netUp, int plcaId, int plcaCount);
void zenohT1sPrintStatus();
bool zenohT1sAvailable();
