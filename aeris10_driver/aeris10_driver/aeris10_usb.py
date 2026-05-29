"""
AERIS-10 USB hardware interface.

Binary frame protocol (little-endian):
  Header — 16 bytes:
    magic[2]       : 0xAE 0x10
    frame_id[4]    : uint32
    timestamp_us[8]: uint64
    n_returns[2]   : uint16

  Per-return — 16 bytes × n_returns:
    range_m[4]   : float32  — slant range
    az_deg[4]    : float32  — azimuth, 0=+X (drone forward), CCW positive
    el_deg[4]    : float32  — elevation, positive=up
    power_dBm[4] : float32  — received power

Adapt FRAME_MAGIC / VID / PID to match actual AERIS-10 firmware.
"""

import struct
import time
import threading
from dataclasses import dataclass, field
from typing import Optional

try:
    import usb.core
    import usb.util
    _USB_AVAILABLE = True
except ImportError:
    _USB_AVAILABLE = False

AERIS10_VID   = 0x0483   # STMicroelectronics (typical for STM32 USB CDC)
AERIS10_PID   = 0xAE10   # AERIS-10 product ID — update from device descriptor
FRAME_MAGIC   = b'\xAE\x10'
HEADER_FMT    = '<2sIQH'   # magic(2) frame_id(4) timestamp_us(8) n_returns(2)
HEADER_SIZE   = struct.calcsize(HEADER_FMT)   # 16
RETURN_FMT    = '<ffff'    # range az el power (4×4 = 16)
RETURN_SIZE   = struct.calcsize(RETURN_FMT)   # 16
USB_TIMEOUT_MS = 1000
USB_EP_IN      = 0x81     # bulk IN endpoint — verify with lsusb -v


@dataclass
class RadarReturn:
    range_m:   float
    az_deg:    float
    el_deg:    float
    power_dBm: float


@dataclass
class RadarFrame:
    frame_id:     int
    timestamp_us: int
    returns:      list = field(default_factory=list)


class Aeris10USB:
    """
    Blocking USB reader for AERIS-10.  Call read_frame() in a thread loop.
    Raises IOError on connection loss; caller should reconnect.
    """

    def __init__(self, vid: int = AERIS10_VID, pid: int = AERIS10_PID):
        self._vid   = vid
        self._pid   = pid
        self._dev   = None
        self._ep_in = None
        self._buf   = b''
        self._lock  = threading.Lock()

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        if not _USB_AVAILABLE:
            return False
        dev = usb.core.find(idVendor=self._vid, idProduct=self._pid)
        if dev is None:
            return False
        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
            dev.set_configuration()
            cfg = dev.get_active_configuration()
            intf = cfg[(0, 0)]
            ep = usb.util.find_descriptor(
                intf,
                custom_match=lambda e: (
                    usb.util.endpoint_direction(e.bEndpointAddress) ==
                    usb.util.ENDPOINT_IN
                ),
            )
            if ep is None:
                return False
            self._dev   = dev
            self._ep_in = ep
            self._buf   = b''
            return True
        except usb.core.USBError:
            return False

    def disconnect(self):
        if self._dev is not None:
            try:
                usb.util.dispose_resources(self._dev)
            except Exception:
                pass
            self._dev   = None
            self._ep_in = None

    @property
    def connected(self) -> bool:
        return self._dev is not None

    # ── Reading ───────────────────────────────────────────────────────────────

    def _read_raw(self, n: int) -> bytes:
        """Read exactly n bytes from USB, accumulating across bulk packets."""
        while len(self._buf) < n:
            try:
                chunk = self._ep_in.read(
                    self._ep_in.wMaxPacketSize * 4,
                    timeout=USB_TIMEOUT_MS,
                )
                self._buf += bytes(chunk)
            except usb.core.USBTimeoutError:
                raise IOError('USB read timeout')
            except usb.core.USBError as e:
                raise IOError(f'USB error: {e}')
        data, self._buf = self._buf[:n], self._buf[n:]
        return data

    def _sync_to_magic(self):
        """Discard bytes until FRAME_MAGIC found (re-sync after partial frame)."""
        while True:
            b = self._read_raw(1)
            if b == FRAME_MAGIC[0:1]:
                b2 = self._read_raw(1)
                if b2 == FRAME_MAGIC[1:2]:
                    self._buf = FRAME_MAGIC + self._buf
                    return

    def read_frame(self) -> RadarFrame:
        """Block until one complete frame is available. Thread-safe."""
        with self._lock:
            # Sync to magic bytes
            hdr_raw = self._read_raw(HEADER_SIZE)
            if hdr_raw[:2] != FRAME_MAGIC:
                # Lost sync — find next magic
                self._buf = hdr_raw[2:] + self._buf
                self._sync_to_magic()
                hdr_raw = self._read_raw(HEADER_SIZE)

            magic, frame_id, ts_us, n_ret = struct.unpack(HEADER_FMT, hdr_raw)
            if magic != FRAME_MAGIC:
                raise IOError('Frame magic mismatch after sync')

            frame = RadarFrame(frame_id=frame_id, timestamp_us=ts_us)

            payload = self._read_raw(n_ret * RETURN_SIZE)
            for i in range(n_ret):
                rng, az, el, pwr = struct.unpack_from(
                    RETURN_FMT, payload, i * RETURN_SIZE)
                frame.returns.append(RadarReturn(
                    range_m=rng, az_deg=az, el_deg=el, power_dBm=pwr))

            return frame
