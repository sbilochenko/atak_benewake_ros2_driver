"""Parser for the Benewake TFmini-S standard 9-byte UART frame."""

from dataclasses import dataclass


@dataclass(frozen=True)
class TFminiSMeasurement:
    distance_cm: int
    strength: int
    temperature_raw: int


class TFminiSParser:
    """Find and validate TFmini-S frames in an arbitrary byte stream."""

    HEADER = b"\x59\x59"
    FRAME_LENGTH = 9

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[TFminiSMeasurement]:
        self._buffer.extend(data)
        measurements = []

        while len(self._buffer) >= self.FRAME_LENGTH:
            # Find the frame header, even if bytes arrive misaligned.
            header_index = self._buffer.find(self.HEADER)

            if header_index < 0:
                # Keep a trailing 0x59 in case the next read completes header.
                keep_last = self._buffer[-1:] == b"\x59"
                self._buffer[:] = self._buffer[-1:] if keep_last else b""
                break

            if header_index:
                del self._buffer[:header_index]

            if len(self._buffer) < self.FRAME_LENGTH:
                break

            frame = self._buffer[:self.FRAME_LENGTH]
            checksum = sum(frame[:8]) & 0xFF

            if checksum != frame[8]:
                # Shift one byte and search again for a valid frame.
                del self._buffer[0]
                continue

            distance_cm = frame[2] | (frame[3] << 8)
            strength = frame[4] | (frame[5] << 8)
            temperature_raw = frame[6] | (frame[7] << 8)

            measurements.append(
                TFminiSMeasurement(
                    distance_cm=distance_cm,
                    strength=strength,
                    temperature_raw=temperature_raw,
                )
            )
            del self._buffer[:self.FRAME_LENGTH]

        return measurements
