from dataclasses import dataclass
from typing import Any, Dict, Optional


@dataclass(frozen=True)
class Position:
    frame_id: str
    x: float
    y: float
    z: float
    unit: str = "m"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "frame_id": self.frame_id,
            "x": float(self.x),
            "y": float(self.y),
            "z": float(self.z),
            "unit": self.unit,
        }


@dataclass(frozen=True)
class TargetEvent:
    seq: int
    timestamp: float
    drone_id: str
    message_type: str
    target_id: str
    target_type: str
    result: Dict[str, Any]
    position: Position
    confidence: Optional[float] = None
    image: Optional[Dict[str, Any]] = None
    version: int = 1

    def to_dict(self) -> Dict[str, Any]:
        event = {
            "version": self.version,
            "seq": self.seq,
            "timestamp": self.timestamp,
            "drone_id": self.drone_id,
            "message_type": self.message_type,
            "target_id": self.target_id,
            "target_type": self.target_type,
            "result": dict(self.result),
            "position": self.position.to_dict(),
        }
        if self.confidence is not None:
            event["confidence"] = float(self.confidence)
        if self.image is not None:
            event["image"] = dict(self.image)
        return event

