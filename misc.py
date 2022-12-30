from dataclasses import dataclass
from math import sqrt
from dataclass_wizard import JSONWizard

# import numba


@dataclass
class Position2D(JSONWizard):
    x: float
    y: float

    def dist(self, to: "Position2D") -> float:
        """Euclidean distance to other coordinates"""
        return sqrt((self.x - to.x) ** 2 + (self.y - to.y) ** 2)

    def __eq__(self, other: "Position2D") -> bool:
        return (self.x, self.y) == (other.x, other.y)

    def __hash__(self):
        return hash((self.x, self.y))

    def in_circle(self, circle: "Circle") -> bool:
        return self.dist(circle.center) < circle.radius

    def __add__(self, other: "Position2D") -> "Position2D":
        return Position2D(self.x + other.x, self.y + other.y)

    def __mul__(self, other: int) -> "Position2D":
        return Position2D(self.x * other, self.y * other)

    def __sub__(self, other: "Position2D") -> "Position2D":
        return self + (other * -1)

    def vector_dot(self, other: "Position2D") -> int:
        return self.x * other.x + self.y * other.y

    @classmethod
    def from_str(cls, s: str) -> "Position2D":
        return cls(*map(int, s.split()))

    def to_str(self):
        return f"{self.x} {self.y}"


# @numba.njit
def in_circle(x, y, cx, cy, r):
    return (x - cx) ** 2 + (y - cy) ** 2 < r**2


# @numba.njit
def distance_in_circle(p1x, p1y, p2x, p2y, cx, cy, r):
    p1_in, p2_in = in_circle(p1x, p1y, cx, cy, r), in_circle(p2x, p2y, cx, cy, r)
    if p1_in and p2_in:
        return ((p1x - p2x) ** 2 + (p1y - p2y) ** 2) ** 0.5

    (x1, y1), (x2, y2) = (p1x - cx, p1y - cy), (p2x - cx, p2y - cy)
    dx, dy = (x2 - x1), (y2 - y1)
    dr = (dx**2 + dy**2) ** 0.5
    big_d = x1 * y2 - x2 * y1
    discriminant = r**2 * dr**2 - big_d**2

    if discriminant <= 0:
        return 0

    intersections = [
        (
            cx
            + (big_d * dy + sign * (-1 if dy < 0 else 1) * dx * discriminant**0.5)
            / dr**2,
            cy + (-big_d * dx + sign * abs(dy) * discriminant**0.5) / dr**2,
        )
        for sign in ((1, -1) if dy < 0 else (-1, 1))
    ]  # This makes sure the order along the segment is correct
    fraction_along_segment = [
        (xi - p1x) / dx if abs(dx) > abs(dy) else (yi - p1y) / dy
        for xi, yi in intersections
    ]
    intersections = [
        pt for pt, frac in zip(intersections, fraction_along_segment) if 0 <= frac <= 1
    ]

    if p1_in:
        intersections.append((p1x, p1y))
    if p2_in:
        intersections.append((p2x, p2y))

    if len(intersections) < 2:
        return 0

    (x1, y1), (x2, y2) = intersections
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
