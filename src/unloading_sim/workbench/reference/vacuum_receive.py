"""Receive-only selections from reference vacuum.py at f0353d720374066528f258de6f4de24502408535.
0x71=positive, 0x72=negative, as hardware-corrected upstream.
No serial writer, threshold setter or alarm motion callback is imported.
"""
from __future__ import annotations
import math
import re
import struct
from dataclasses import dataclass
HEADER = b"\xEE\x16"


DEVICE = 0x03


BAUD = 115200


MAX_PARAMETERS = 5


COMMANDS = {
    0xF1: "电机故障", 0xF2: "压力表超压",
    # 0x11 is 电动执行器开始(测距模块) in the document.
    0x11: "测距", 0x12: "抓取货物", 0x13: "释放货物", 0x14: "设置传送带1转速", 0x15: "设置传送带2转速",
    0x21: "电动执行器准备好", 0x22: "压力表异常状态", 0x23: "电机一般报警信息",
    0x31: "读取压力表压力", 0x32: "读取传送带速率",
    0x41: "读取程序版本信息", 0x42: "读取本次完成时间", 0x43: "读取初始化时间",
    0x51: "空压机阀门开", 0x52: "空压机阀门关", 0x53: "负压吸具阀门开", 0x54: "负压吸具阀门关",
    0x55: "真空泵阀门开", 0x56: "真空泵阀门关",
    0x61: "吸具压力", 0x62: "负压储气罐压力", 0x63: "真空泵压力", 0x64: "压缩机压力",
    # The document (§5 ④ 五) has these two swapped; on the board 71 sets POS_PRESS, 72 the negative one.
    0x71: "配置正压阈值", 0x72: "配置负压阈值",
}


POSITIVE_THRESHOLD, NEGATIVE_THRESHOLD = 0x71, 0x72


ALARMS = {0xF1, 0xF2, 0x22, 0x23}


@dataclass(frozen=True)
class Gauge:
    number: int
    name: str
    low_kpa: float
    high_kpa: float
    expected_kpa: float  # Idle reading before start (§4 注意事项 5).

    @property
    def heartbeat(self) -> int:
        """The heartbeat command carrying this gauge: 0x61-0x64."""
        return 0x60 + self.number


GAUGES = (Gauge(1, "吸具", -100., 0., 0.), Gauge(2, "储气罐", -100., 0., -60.),
          Gauge(3, "真空泵", -100., 0., -85.), Gauge(4, "压缩机", 0., 600., 300.))


HEARTBEATS = {gauge.heartbeat: gauge.number for gauge in GAUGES}


PLAUSIBLE_KPA = 2000.  # Beyond any gauge here; a corrupted value is dropped rather than shown.


HEARTBEAT_FORMAT = "<f"  # Little-endian float32, kPa (confirmed against the board's vendor tool).


class FrameParser:
    """Splits a byte stream into (command, parameters), skipping noise and bad checksums.

    A complete frame whose device code or checksum is wrong is never parsed, but is kept
    in `bad` as (raw bytes, reason) so the GUI can show it: a firmware checksum that
    differs from the document would otherwise look like silence.
    """

    def __init__(self):
        self._buffer = bytearray()
        self.rejected = 0
        self.bad = []  # The caller takes and clears these after each feed().

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self._buffer += data
        frames = []
        while True:
            start = self._buffer.find(HEADER)
            if start < 0:
                del self._buffer[:max(0, len(self._buffer) - 1)]  # Keep a possible split header.
                return frames
            del self._buffer[:start]
            if len(self._buffer) < 3:
                return frames
            length = self._buffer[2]
            if not 2 <= length <= 2 + MAX_PARAMETERS:
                self.rejected += 1
                del self._buffer[:1]
                continue
            if len(self._buffer) < 3 + length + 1:
                return frames
            body, checksum = bytes(self._buffer[3:3 + length]), self._buffer[3 + length]
            if body[0] != DEVICE or sum(body) & 0xFF != checksum:
                self.rejected += 1
                reason = (f"设备码 {body[0]:02X} 不是 {DEVICE:02X}" if body[0] != DEVICE
                          else f"校验失败（应为 {sum(body) & 0xFF:02X}）")
                self.bad.append((bytes(self._buffer[:3 + length + 1]), reason))
                del self._buffer[:1]  # Resynchronise inside it: it may hide a real frame.
                continue
            del self._buffer[:3 + length + 1]
            frames.append((body[1], body[2:]))


class TextReportParser:
    """Pressures from the board's periodic ASCII debug report (no framing or checksum).

        ***jiffis = 5449000***
        AI:
         1[ 2949]   2[ 1414]   3[ 1403]   4[ 1959]     raw ADC
         1[-1.98]   2[-79.02]  3[-79.57]  4[241.72]    kPa
        AO: ...
        DI: ...

    Only the AI block's decimal line is used; binary frames in the same stream are ignored.
    Any other text the board prints (e.g. POS_PRESS after a 71 threshold answer) is
    collected in `messages`, which the caller takes and clears after each feed().
    """

    VALUE = re.compile(rb"(\d+)\[\s*(-?\d+\.\d+)\s*\]")
    REPORT_ITEM = re.compile(rb"\d+\[\s*-?\d+(?:\.\d+)?\s*\]")  # Any AI/AO/DI entry, integer or decimal.
    TEXT = re.compile(rb"[\x20-\x7e\t]{4,}")  # Printable runs; a frame's few printable bytes stay below this.

    def __init__(self):
        self._line = bytearray()
        self._pending = b""  # An incomplete binary frame, held until it can be removed whole.
        self._in_ai = False
        self.messages = []

    def feed(self, data: bytes) -> list[tuple[int, float]]:
        readings = []
        # Frames out first: their bytes may hold 0x0A and would otherwise split and pollute lines.
        text, self._pending = _remove_frames(self._pending + data)
        self._line += text
        *lines, rest = self._line.split(b"\n")
        self._line = bytearray(rest[-256:])  # A line without newline is not text; bound it.
        for line in lines:
            if b"AI:" in line:
                self._in_ai = True
            elif b"AO:" in line or b"DI:" in line or b"jiffis" in line:
                self._in_ai = False
            elif self._in_ai:
                values = self.VALUE.findall(line)
                if values:
                    readings += [(int(channel), float(value)) for channel, value in values]
                    self._in_ai = False
            elif not self.REPORT_ITEM.search(line):  # Not part of the periodic report: a message.
                text = " ".join(run.decode("ascii").strip() for run in self.TEXT.findall(line)).strip()
                if any(char.isalnum() for char in text):  # Skips filler such as "......".
                    self.messages.append(text)
        return readings


def _remove_frames(data: bytes) -> tuple[bytes, bytes]:
    """(data without complete EE 16 frames, an unfinished frame to prepend to the next data)."""
    out, index = bytearray(), 0
    while (start := data.find(HEADER, index)) >= 0:
        out += data[index:start]
        if start + 3 > len(data):
            return bytes(out), bytes(data[start:])
        length = data[start + 2]
        if not 2 <= length <= 2 + MAX_PARAMETERS:  # Not a frame after all: keep the bytes as text.
            out += HEADER
            index = start + 2
            continue
        if start + 4 + length > len(data):
            return bytes(out), bytes(data[start:])
        index = start + 4 + length
    out += data[index:]
    if out.endswith(HEADER[:1]):  # Possibly the first byte of the next header.
        return bytes(out[:-1]), HEADER[:1]
    return bytes(out), b""


def parse_heartbeat(command: int, parameters: bytes):
    """(gauge number, kPa) from a 0x61-0x64 heartbeat, or None if it is not one.

    The board sends a little-endian float32 in kPa: 80 50 DD BF = -1.73 (吸具),
    D2 D4 9D C2 = -78.92 (储气罐), as its vendor tool shows. Never guess the byte
    order: near zero, a little-endian value's bytes often also read as a plausible
    big-endian one (40 53 DD BF is -1.73 little-endian but 3.31 big-endian).
    """
    if command not in HEARTBEATS or len(parameters) != 4:
        return None
    value = struct.unpack(HEARTBEAT_FORMAT, parameters)[0]
    if not math.isfinite(value) or abs(value) > PLAUSIBLE_KPA:
        return None
    return HEARTBEATS[command], float(value)
