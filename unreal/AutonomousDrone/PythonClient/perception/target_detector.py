"""Extension point for future object-detection models."""

from __future__ import annotations


class TargetDetector:
    """Placeholder interface so mission code does not depend on a specific model."""

    def detect(self, image_bytes: bytes) -> list[dict]:
        del image_bytes
        return []
