"""T1S SDV signal API: COVESA VSS signals over SOME/IP on 10BASE-T1S, with an ESP32-S3 + LAN8651 as the gateway.
  wire   formats (SOME/IP, SOME/IP-SD, the host channel), the dictionary helpers
  node   a SOME/IP node on an IP interface (Linux on the T1S bus, or any LAN)
  host   a host behind the gateway: USB serial (macOS, Linux, Windows: no T1S driver needed) or UDP"""
from .vss_dict import HASH, SIGNALS  # noqa: F401
