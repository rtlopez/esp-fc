#!/usr/bin/env python3
# Requires pyserial: install with `python3 -m pip install pyserial`
# On Ubuntu you can also install it with `sudo apt install python3-serial`
import argparse
from dataclasses import dataclass
import sys
import time

import serial

"""
MSP debug script
"""

MSP_BUF_SIZE = 192
TIMEOUT_SECONDS = 3.0

MSP_STATE_IDLE = 0
MSP_STATE_HEADER_START = 1
MSP_STATE_HEADER_M = 2
MSP_STATE_HEADER_V1 = 3
MSP_STATE_PAYLOAD_V1 = 4
MSP_STATE_CHECKSUM_V1 = 5
MSP_STATE_HEADER_X = 6
MSP_STATE_HEADER_V2 = 7
MSP_STATE_PAYLOAD_V2 = 8
MSP_STATE_CHECKSUM_V2 = 9
MSP_STATE_RECEIVED = 10

MSP_TYPE_CMD = 0
MSP_TYPE_REPLY = 1

MSP_V1 = 0
MSP_V2 = 1

@dataclass
class ParsedFrame:
    version: int
    direction: int
    frame_type: int
    flags: int
    cmd: int
    payload: bytes
    checksum: int
    raw: bytes

    @property
    def header_length(self) -> int:
        return 5 if self.version == MSP_V1 else 8

    @property
    def header(self) -> bytes:
        return self.raw[: self.header_length]


class MspParser:
    def __init__(self) -> None:
        self.reset(full=True)

    def reset(self, full: bool = False) -> None:
        self.state = MSP_STATE_IDLE
        self.version = MSP_V1
        self.direction = MSP_TYPE_CMD
        self.frame_type = 0
        self.flags = 0
        self.cmd = 0
        self.expected = 0
        self.received = 0
        self.checksum = 0
        self.checksum2 = 0
        self.buffer = bytearray()
        self.raw = bytearray() if full else self.raw[:0]

    def feed(self, byte: int):
        c = byte & 0xFF

        if self.state == MSP_STATE_IDLE:
            if c == ord('$'):
                self.raw = bytearray((c,))
                self.state = MSP_STATE_HEADER_START
            return None

        if self.state == MSP_STATE_HEADER_START:
            self.received = 0
            self.checksum = 0
            self.checksum2 = 0
            self.buffer = bytearray()
            self.raw.append(c)
            if c == ord('M'):
                self.version = MSP_V1
                self.state = MSP_STATE_HEADER_M
            elif c == ord('X'):
                self.version = MSP_V2
                self.state = MSP_STATE_HEADER_X
            else:
                self.reset()
            return None

        if self.state == MSP_STATE_HEADER_M:
            self.raw.append(c)
            if c == ord('>'):
                self.direction = MSP_TYPE_REPLY
                self.frame_type = c
                self.state = MSP_STATE_HEADER_V1
            elif c == ord('<'):
                self.direction = MSP_TYPE_CMD
                self.frame_type = c
                self.state = MSP_STATE_HEADER_V1
            elif c == ord('!'):
                self.direction = MSP_TYPE_REPLY
                self.frame_type = c
                self.state = MSP_STATE_HEADER_V1
            else:
                self.reset()
            return None

        if self.state == MSP_STATE_HEADER_X:
            self.raw.append(c)
            if c == ord('>'):
                self.direction = MSP_TYPE_REPLY
                self.frame_type = c
                self.state = MSP_STATE_HEADER_V2
            elif c == ord('<'):
                self.direction = MSP_TYPE_CMD
                self.frame_type = c
                self.state = MSP_STATE_HEADER_V2
            elif c == ord('!'):
                self.direction = MSP_TYPE_REPLY
                self.frame_type = c
                self.state = MSP_STATE_HEADER_V2
            else:
                self.reset()
            return None

        if self.state == MSP_STATE_HEADER_V1:
            self.buffer.append(c)
            self.raw.append(c)
            self.received += 1
            self.checksum ^= c
            if self.received == 2:
                size = self.buffer[0]
                if size > MSP_BUF_SIZE:
                    self.reset()
                else:
                    self.expected = size
                    self.cmd = self.buffer[1]
                    self.received = 0
                    self.buffer = bytearray()
                    self.state = MSP_STATE_PAYLOAD_V1 if self.expected > 0 else MSP_STATE_CHECKSUM_V1
            return None

        if self.state == MSP_STATE_PAYLOAD_V1:
            self.buffer.append(c)
            self.raw.append(c)
            self.received += 1
            self.checksum ^= c
            if self.received == self.expected:
                self.state = MSP_STATE_CHECKSUM_V1
            return None

        if self.state == MSP_STATE_CHECKSUM_V1:
            self.raw.append(c)
            if self.checksum != c:
                self.reset()
                return None
            frame = ParsedFrame(
                version=self.version,
                direction=self.direction,
                frame_type=self.frame_type,
                flags=0,
                cmd=self.cmd,
                payload=bytes(self.buffer),
                checksum=c,
                raw=bytes(self.raw),
            )
            self.state = MSP_STATE_RECEIVED
            self.reset()
            return frame

        if self.state == MSP_STATE_HEADER_V2:
            self.buffer.append(c)
            self.raw.append(c)
            self.received += 1
            self.checksum2 = crc8_dvb_s2(self.checksum2, c)
            if self.received == 5:
                flags = self.buffer[0]
                cmd = self.buffer[1] | (self.buffer[2] << 8)
                size = self.buffer[3] | (self.buffer[4] << 8)
                if size > MSP_BUF_SIZE:
                    self.reset()
                else:
                    self.flags = flags
                    self.cmd = cmd
                    self.expected = size
                    self.received = 0
                    self.buffer = bytearray()
                    self.state = MSP_STATE_PAYLOAD_V2 if self.expected > 0 else MSP_STATE_CHECKSUM_V2
            return None

        if self.state == MSP_STATE_PAYLOAD_V2:
            self.buffer.append(c)
            self.raw.append(c)
            self.received += 1
            self.checksum2 = crc8_dvb_s2(self.checksum2, c)
            if self.received == self.expected:
                self.state = MSP_STATE_CHECKSUM_V2
            return None

        if self.state == MSP_STATE_CHECKSUM_V2:
            self.raw.append(c)
            if self.checksum2 != c:
                self.reset()
                return None
            frame = ParsedFrame(
                version=self.version,
                direction=self.direction,
                frame_type=self.frame_type,
                flags=self.flags,
                cmd=self.cmd,
                payload=bytes(self.buffer),
                checksum=c,
                raw=bytes(self.raw),
            )
            self.state = MSP_STATE_RECEIVED
            self.reset()
            return frame

        self.reset()
        return None


def crc8_dvb_s2(crc: int, value: int) -> int:
    crc ^= value & 0xFF
    for _ in range(8):
        if crc & 0x80:
            crc = ((crc << 1) ^ 0xD5) & 0xFF
        else:
            crc = (crc << 1) & 0xFF
    return crc


def parse_message_id(value: str) -> int:
    try:
        cmd = int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid message id: {value}") from exc
    if not 0 <= cmd <= 0xFFFF:
        raise argparse.ArgumentTypeError("message id must be in range 0..65535")
    return cmd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("message_id", type=parse_message_id)
    parser.add_argument("--baud", type=int, default=115200)
    return parser.parse_args()


@dataclass
class RequestFrame:
    version: int
    header: bytes
    payload: bytes
    checksum: int

    @property
    def raw(self) -> bytes:
        return self.header + self.payload + bytes((self.checksum,))


def build_request(cmd: int) -> RequestFrame:
    if cmd <= 0xFF:
        header = bytes((ord('$'), ord('M'), ord('<'), 0, cmd))
        checksum = 0 ^ header[3] ^ header[4]
        return RequestFrame(MSP_V1, header, b"", checksum)

    header = bytes((ord('$'), ord('X'), ord('<'), 0, cmd & 0xFF, (cmd >> 8) & 0xFF, 0, 0))
    checksum = 0
    for value in header[3:8]:
        checksum = crc8_dvb_s2(checksum, value)
    return RequestFrame(MSP_V2, header, b"", checksum)


def read_response(ser: serial.Serial, expected_cmd: int, timeout: float) -> ParsedFrame:
    parser = MspParser()
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"timeout waiting for response to message {expected_cmd}")
        ser.timeout = max(0.0, min(remaining, 0.2))
        chunk = ser.read(256)
        if not chunk:
            continue
        for byte in chunk:
            frame = parser.feed(byte)
            if frame and frame.direction == MSP_TYPE_REPLY and frame.cmd == expected_cmd:
                return frame


def open_serial(port: str, baud: int) -> serial.Serial:
    try:
        return serial.Serial(
            port=port,
            baudrate=baud,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.2,
            write_timeout=1.0,
        )
    except (serial.SerialException, ValueError) as exc:
        raise OSError(f"failed to open serial port {port}: {exc}") from exc


def format_bytes(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def response_marker(response: ParsedFrame) -> str:
    return "!" if response.frame_type == ord("!") else ">"


def format_header(header: bytes) -> str:
    result = header[0:3].decode("ascii")
    result += " " + " ".join(f"{byte:02X}" for byte in header[0:])
    return result


def print_frame_parts(request: RequestFrame, response: ParsedFrame) -> None:
    marker = response_marker(response)
    print("<", format_header(request.header), "..", format_bytes(bytes((request.checksum,))))
    print("<", format_bytes(request.payload))
    print(marker, format_header(response.header), "..", format_bytes(bytes((response.checksum,))))
    print(marker, format_bytes(response.payload))


def main() -> int:
    args = parse_args()
    request = build_request(args.message_id)
    ser = open_serial(args.port, args.baud)
    try:
        ser.write(request.raw)
        ser.flush()
        response = read_response(ser, args.message_id, TIMEOUT_SECONDS)
    finally:
        ser.close()
    print_frame_parts(request, response)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, TimeoutError, ValueError, serial.SerialException) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1)
