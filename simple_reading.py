#!/usr/bin/env python3
"""
Temtop PMD 331 particle counter logger (Modbus RTU over RS232).

Polls the "Read Detected Data" register block (function 0x04, start
address 0x0003, 14 registers = 7 particle-size channels x 32-bit count)
every SAMPLE_INTERVAL_S seconds and appends the results to a CSV file.

Request frame  : FE 04 00 03 00 0E <CRC lo> <CRC hi>          (8 bytes)
Response frame : FE 04 1C <28 data bytes> <CRC lo> <CRC hi>    (33 bytes)
Each channel is a 32-bit big-endian unsigned count (Hi register
then Lo register, i.e. standard Modbus word order).

Requires: pip install pyserial
"""

import csv
import os
import struct
import sys
import time
from datetime import datetime

import serial

# ----------------------------------------------------------------------
# Configuration - adjust to match your setup
# ----------------------------------------------------------------------
SERIAL_PORT = '/dev/ttyUSB0'  # e.g. "COM3" on Windows, "/dev/ttyUSB0" on Linux/Mac
BAUDRATE = 115200             # device default per manual; unit also supports
                            # 19200 / 115200 -> check Menu > System Setting > COM
                            # Setting on the device and match it here.
SERIAL_TIMEOUT = 2.0        # seconds to wait for a reply

SLAVE_ADDRESS = 0xFE
FUNCTION_READ = 0x04
START_ADDRESS = 0x0003
REGISTER_COUNT = 0x000E     # 14 registers = 7 channels x 2 registers (32-bit) each

SAMPLE_INTERVAL_S = 60      # seconds between readings

CSV_PATH = "pmd331_log.csv"

CHANNEL_LABELS = ["0.3um", "0.5um", "0.7um", "1.0um", "2.5um", "5.0um", "10.0um"]

# Expected response length: addr(1) + func(1) + bytecount(1) + data(28) + crc(2)
RESPONSE_LEN = 3 + (REGISTER_COUNT * 2) + 2


def crc16_modbus(data: bytes) -> bytes:
    """Return the 2-byte Modbus RTU CRC16 for `data`, low byte first."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])  # low byte, then high byte


def build_read_request() -> bytes:
    """Build the Modbus RTU 'read detected data' request frame."""
    frame = bytes([
        SLAVE_ADDRESS,
        FUNCTION_READ,
        (START_ADDRESS >> 8) & 0xFF,
        START_ADDRESS & 0xFF,
        (REGISTER_COUNT >> 8) & 0xFF,
        REGISTER_COUNT & 0xFF,
    ])
    return frame + crc16_modbus(frame)


def parse_response(resp: bytes):
    """Validate length/CRC and extract the 7 particle counts.

    Returns a list[int] of length 7, or None if the frame is invalid.
    """
    if len(resp) != RESPONSE_LEN:
        print(f"[warn] unexpected response length: {len(resp)} bytes")
        return None

    payload, received_crc = resp[:-2], resp[-2:]
    if crc16_modbus(payload) != received_crc:
        print("[warn] CRC mismatch, discarding frame")
        return None

    if resp[0] != SLAVE_ADDRESS or resp[1] != FUNCTION_READ:
        print("[warn] unexpected address/function code in response")
        return None

    byte_count = resp[2]
    data = resp[3:3 + byte_count]
    counts = [struct.unpack(">I", data[i:i + 4])[0] for i in range(0, len(data), 4)]
    return counts


def append_to_csv(timestamp: str, counts: list):
    new_file = not os.path.exists(CSV_PATH)
    with open(CSV_PATH, "a", newline="") as f:
        writer = csv.writer(f)
        if new_file:
            writer.writerow(["timestamp"] + CHANNEL_LABELS)
        writer.writerow([timestamp] + counts)


def main():
    print(f"Opening {SERIAL_PORT} @ {BAUDRATE} baud (8N1) ...")
    with serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=SERIAL_TIMEOUT,
    ) as ser:
        print(f"Logging every {SAMPLE_INTERVAL_S}s to {CSV_PATH}. Press Ctrl+C to stop.")
        request = build_read_request()

        while True:
            cycle_start = time.monotonic()
            timestamp = datetime.now().isoformat(timespec="seconds")

            try:
                ser.reset_input_buffer()
                ser.write(request)
                response = ser.read(RESPONSE_LEN)
                counts = parse_response(response)

                if counts is not None:
                    append_to_csv(timestamp, counts)
                    readable = ", ".join(f"{lbl}={c}" for lbl, c in zip(CHANNEL_LABELS, counts))
                    print(f"{timestamp}  {readable}")
                else:
                    print(f"{timestamp}  no valid reading (got {len(response)} bytes)")

            except serial.SerialException as e:
                print(f"[error] serial communication failed: {e}")

            elapsed = time.monotonic() - cycle_start
            time.sleep(max(0.0, SAMPLE_INTERVAL_S - elapsed))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user.")
        sys.exit(0)