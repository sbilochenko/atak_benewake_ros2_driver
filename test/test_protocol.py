
"""Tests for the Benewake TFmini-S UART protocol."""

from benewake_ros2_driver.protocol import TFminiSParser


def test_valid_frame():
    """Accept a valid frame and decode its measurement."""
    # Distance = 123 cm, strength = 50, temperature raw = 1000.
    frame = bytearray([
        0x59, 0x59,
        123, 0,
        50, 0,
        232, 3,
        0,
    ])
    frame[8] = sum(frame[:8]) & 0xFF

    parser = TFminiSParser()
    measurements = parser.feed(bytes(frame))

    assert len(measurements) == 1
    assert measurements[0].distance_cm == 123
    assert measurements[0].strength == 50
    assert measurements[0].temperature_raw == 1000


def test_invalid_checksum_is_rejected():
    """Reject a frame with an invalid checksum."""
    frame = bytearray([0x59, 0x59, 123, 0, 50, 0, 232, 3, 0])
    frame[8] = 1

    parser = TFminiSParser()
    assert parser.feed(bytes(frame)) == []
