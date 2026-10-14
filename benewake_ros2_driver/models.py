"""Common measurement model for Benewake lidar sensors."""

from dataclasses import dataclass


@dataclass(frozen=True)
class LidarMeasurement:
    """Normalized measurement shared by supported sensor models."""

    distance_m: float
    strength: int
    temperature_raw: int | None = None

    @property
    def distance_cm(self) -> int:
        """Return distance in centimeters for backward compatibility."""
        return round(self.distance_m * 100)
