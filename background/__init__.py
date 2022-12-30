"""2d map"""

from dataclasses import dataclass
import tkinter as tk
from typing import Any
from util import read_json


@dataclass
class Drawable:
    """An object on the map."""

    permeability: float

    def intersects(self, other: "Drawable") -> bool:
        raise Exception("Not implemented")

    def draw(self, canvas: tk.Canvas):
        raise Exception("Not implemented")


class DrawablesFactory:
    def __init__(self):
        self.registry = {}

    def from_json(self, json_data: dict) -> Drawable:
        t = json_data["type"]
        del json_data["type"]
        return self.registry[t](json_data)


# background is a set of drawables
class BackgroundFactory:
    def __init__(self, drawables_factory: DrawablesFactory):
        self.drawables_factory = drawables_factory

    def read(self, file: str) -> list[Any]:
        res = []
        for item in read_json(file):
            drawable = self.drawables_factory.from_json(item)
            assert not any(drawable.intersects(other) for other in res)
            res.append(drawable)
        return res
