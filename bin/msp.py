#!/usr/bin/env python3
# Requires pyserial: install with `python3 -m pip install pyserial`
# On Ubuntu you can also install it with `sudo apt install python3-serial`
import argparse
import sys
import time
from dataclasses import dataclass

import serial

"""
MSP debug script
"""

TIMEOUT_SECONDS = 3.0
DATAFLASH_TIMEOUT_SECONDS = 0.5
DATAFLASH_RETRIES = 0
MSP_DATAFLASH_SUMMARY = 70
MSP_DATAFLASH_READ = 71
DATAFLASH_READ_SIZE = 256

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
            if c == ord("$"):
                self.raw = bytearray((c,))
                self.state = MSP_STATE_HEADER_START
            return None

        if self.state == MSP_STATE_HEADER_START:
            self.received = 0
            self.checksum = 0
            self.checksum2 = 0
            self.buffer = bytearray()
            self.raw.append(c)
            if c == ord("M"):
                self.version = MSP_V1
                self.state = MSP_STATE_HEADER_M
            elif c == ord("X"):
                self.version = MSP_V2
                self.state = MSP_STATE_HEADER_X
            else:
                self.reset()
            return None

        if self.state == MSP_STATE_HEADER_M:
            self.raw.append(c)
            if c == ord(">"):
                self.direction = MSP_TYPE_REPLY
                self.frame_type = c
                self.state = MSP_STATE_HEADER_V1
            elif c == ord("<"):
                self.direction = MSP_TYPE_CMD
                self.frame_type = c
                self.state = MSP_STATE_HEADER_V1
            elif c == ord("!"):
                self.direction = MSP_TYPE_REPLY
                self.frame_type = c
                self.state = MSP_STATE_HEADER_V1
            else:
                self.reset()
            return None

        if self.state == MSP_STATE_HEADER_X:
            self.raw.append(c)
            if c == ord(">"):
                self.direction = MSP_TYPE_REPLY
                self.frame_type = c
                self.state = MSP_STATE_HEADER_V2
            elif c == ord("<"):
                self.direction = MSP_TYPE_CMD
                self.frame_type = c
                self.state = MSP_STATE_HEADER_V2
            elif c == ord("!"):
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
                self.expected = self.buffer[0]
                self.cmd = self.buffer[1]
                self.received = 0
                self.buffer = bytearray()
                self.state = (
                    MSP_STATE_PAYLOAD_V1
                    if self.expected > 0
                    else MSP_STATE_CHECKSUM_V1
                )
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
                self.flags = self.buffer[0]
                self.cmd = self.buffer[1] | (self.buffer[2] << 8)
                self.expected = self.buffer[3] | (self.buffer[4] << 8)
                self.received = 0
                self.buffer = bytearray()
                self.state = (
                    MSP_STATE_PAYLOAD_V2
                    if self.expected > 0
                    else MSP_STATE_CHECKSUM_V2
                )
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
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    commands = parser.add_subparsers(dest="action", required=True)

    send_parser = commands.add_parser("send")
    send_parser.add_argument("message_id", type=parse_message_id, help="Message ID to read")

    get_log_parser = commands.add_parser("get_log")
    get_log_parser.add_argument("--logfile", required=True, help="Output Blackbox log file")
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


def build_request(cmd: int, payload: bytes = b"") -> RequestFrame:
    if cmd <= 0xFF and len(payload) <= 0xFF:
        header = bytes((ord("$"), ord("M"), ord("<"), len(payload), cmd))
        checksum = 0
        for value in header[3:] + payload:
            checksum ^= value
        return RequestFrame(MSP_V1, header, payload, checksum)

    header = bytes(
        (
            ord("$"),
            ord("X"),
            ord("<"),
            0,
            cmd & 0xFF,
            (cmd >> 8) & 0xFF,
            len(payload) & 0xFF,
            (len(payload) >> 8) & 0xFF,
        )
    )
    checksum = 0
    for value in header[3:] + payload:
        checksum = crc8_dvb_s2(checksum, value)
    return RequestFrame(MSP_V2, header, payload, checksum)


def read_response(ser: serial.Serial, expected_cmd: int, timeout: float) -> ParsedFrame:
    parser = MspParser()
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(
                f"timeout waiting for response to message {expected_cmd}"
            )
        ser.timeout = max(0.0, min(remaining, 0.2))
        chunk = ser.read(max(1, ser.in_waiting))
        if not chunk:
            continue
        for byte in chunk:
            frame = parser.feed(byte)
            if (
                frame
                and frame.direction == MSP_TYPE_REPLY
                and frame.cmd == expected_cmd
            ):
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
    print(
        "<",
        format_header(request.header),
        "..",
        format_bytes(bytes((request.checksum,))),
    )
    print("<", format_bytes(request.payload))
    print(
        marker,
        format_header(response.header),
        "..",
        format_bytes(bytes((response.checksum,))),
    )
    print(marker, format_bytes(response.payload))


def send_request(
    ser: serial.Serial,
    cmd: int,
    payload: bytes = b"",
    timeout: float = TIMEOUT_SECONDS,
    retries: int = 0,
) -> tuple[RequestFrame, ParsedFrame]:
    request = build_request(cmd, payload)
    for attempt in range(retries + 1):
        ser.write(request.raw)
        ser.flush()
        try:
            response = read_response(ser, cmd, timeout)
            break
        except TimeoutError:
            if attempt == retries:
                raise
            ser.reset_input_buffer()
    if response.frame_type == ord("!"):
        raise ValueError(f"MSP command {cmd} failed")
    return request, response


def read_u16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 2], "little")


def read_u32(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset : offset + 4], "little")


def get_log(ser: serial.Serial, logfile: str) -> int:
    _, summary = send_request(ser, MSP_DATAFLASH_SUMMARY)
    if len(summary.payload) < 13:
        raise ValueError("invalid MSP_DATAFLASH_SUMMARY response")

    flags = summary.payload[0]
    flash_size = read_u32(summary.payload, 5)
    log_size = read_u32(summary.payload, 9)
    print(f"Flags: {flags:02X}, Flash size: {flash_size}, Log size: {log_size}")
    if flags & 2 == 0:
        raise ValueError("dataflash is not supported")
    if flags & 1 == 0:
        raise ValueError("dataflash is not ready")
    if log_size > flash_size:
        raise ValueError("invalid dataflash size in summary")

    address = 0
    with open(logfile, "wb") as output:
        while address < log_size:
            requested = min(DATAFLASH_READ_SIZE, log_size - address)
            payload = (
                address.to_bytes(4, "little")
                + requested.to_bytes(2, "little")
                + b"\x00"
            )
            _, response = send_request(
                ser,
                MSP_DATAFLASH_READ,
                payload,
                DATAFLASH_TIMEOUT_SECONDS,
                DATAFLASH_RETRIES,
            )
            if len(response.payload) < 7:
                raise ValueError("invalid MSP_DATAFLASH_READ response")

            response_address = read_u32(response.payload, 0)
            read_length = read_u16(response.payload, 4)
            compression = response.payload[6]
            data = response.payload[7:]
            print(f"addr: {response_address:08X}/{log_size:08X}, length: {read_length}")
            if response_address != address:
                raise ValueError(
                    f"unexpected dataflash address {response_address:08X}, expected {address:08X}"
                )
            if compression != 0:
                raise ValueError(f"unsupported dataflash compression method {compression}")
            if read_length != len(data) or read_length > requested:
                raise ValueError("invalid data length in MSP_DATAFLASH_READ response")
            if read_length == 0:
                raise ValueError(f"dataflash read stopped at address {address}")

            output.write(data)
            address += read_length

    print(f"Saved {address} bytes to {logfile}")
    return 0


def main() -> int:
    args = parse_args()
    ser = open_serial(args.port, args.baud)
    try:
        if args.action == "get_log":
            return get_log(ser, args.logfile)
        request, response = send_request(ser, args.message_id)
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
