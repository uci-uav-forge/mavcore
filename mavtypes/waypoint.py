from dataclasses import dataclass


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    radius: float

    def __hash__(self):
        # hash floats in a stable way
        return hash(
            (
                round(self.x, 10),
                round(self.y, 10),
                round(self.z, 10),
                round(self.radius, 10),
            )
        )
