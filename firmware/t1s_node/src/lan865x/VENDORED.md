Vendored from https://github.com/espressif/esp-eth-drivers at
b3be86142c813a81343467bc9ae2e0a49d10020f (2026-09-18): `lan865x/` 0.2.0 and
`lan86xx_common/`, flattened into one folder so Arduino compiles them as part of
the sketch. Apache-2.0, see LICENSE.

One local change, marked `LOCAL PATCH (t1s_hat)` in `esp_eth_mac_lan865x.c/.h`:
`esp_eth_mac_lan865x_read_reg` / `_write_reg`, raw register access under the
driver's own SPI lock. Used by the firmware's `reg` command and LED setup.
