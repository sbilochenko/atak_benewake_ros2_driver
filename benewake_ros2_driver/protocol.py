
"""Parser for the Benewake TFmini-S standard 9-byte UART frame."""

from benewake_ros2_driver.models import LidarMeasurement


# Backward-compatible alias for code using the previous class name.
TFminiSMeasurement = LidarMeasurement


class TFminiSParser:
    """Find and validate TFmini-S frames in an arbitrary byte stream."""

    HEADER = b'\x59\x59'
    FRAME_LENGTH = 9

    def __init__(self):
        """Initialize the buffer for incoming UART data."""
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[LidarMeasurement]:
        """Parse incoming bytes and return valid measurements."""
        self._buffer.extend(data)
        measurements = []

        while len(self._buffer) >= self.FRAME_LENGTH:
            header_index = self._buffer.find(self.HEADER)

            if header_index < 0:
                keep_last = self._buffer[-1:] == b'\x59'
                self._buffer[:] = (
                    self._buffer[-1:] if keep_last else b''
                )
                break

            if header_index:
                del self._buffer[:header_index]

            if len(self._buffer) < self.FRAME_LENGTH:
                break

            frame = self._buffer[:self.FRAME_LENGTH]
            checksum = sum(frame[:8]) & 0xFF

            if checksum != frame[8]:
                del self._buffer[0]
                continue

            distance_cm = frame[2] | (frame[3] << 8)
            strength = frame[4] | (frame[5] << 8)
            temperature_raw = frame[6] | (frame[7] << 8)

            measurements.append(
                LidarMeasurement(
                    distance_m=distance_cm / 100.0,
                    strength=strength,
                    temperature_raw=temperature_raw,
                )
            )

            del self._buffer[:self.FRAME_LENGTH]

        return measurements
