from dataclasses import dataclass
import tkinter as tk

from . import Drawable, DrawablesFactory
from dataclass_wizard import JSONWizard
from misc import Position2D, distance_in_circle


@dataclass
class Obstacle(Drawable):
    permeability: float  # 1 - full permeability, 0 - no permeability

    def penalty(self, f: Position2D, t: Position2D) -> float:
        raise NotImplementedError()


@dataclass
class Circle(Obstacle, JSONWizard):
    center: Position2D
    radius: float

    def intersects(self, other: "Drawable") -> bool:
        if isinstance(other, Circle):
            return self.center.dist(other.center) < self.radius + other.radius

    def draw(self, canvas: tk.Canvas):
        assert self.permeability != 1
        # check != 1 because it's pointless to draw anything that will not affect field anyhow
        x0 = self.center.x - self.radius
        y0 = self.center.y - self.radius
        x1 = self.center.x + self.radius
        y1 = self.center.y + self.radius
        if self.permeability < 0:
            rgb = tuple(int(e * (1 / (1 - self.permeability))) for e in (186, 85, 211))
        elif self.permeability < 1:
            rgb = tuple(int(e * (1 - self.permeability)) for e in (65, 105, 225))
        else:
            rgb = tuple(int(e * (1 - (1 / self.permeability))) for e in (0, 205, 0))
        canvas.create_oval(x0, y0, x1, y1, fill="#%02x%02x%02x" % rgb)

    def penalty(self, f: Position2D, t: Position2D) -> tuple[float, float]:
        d = distance_in_circle(
            f.x, f.y, t.x, t.y, self.center.x, self.center.y, self.radius
        )
        # returns distance and penalty
        return d, d / self.permeability


def register_obstacles(df: DrawablesFactory):
    df.registry["circle"] = Circle.from_dict
