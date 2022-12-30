import argparse
from copy import deepcopy
from dataclasses import dataclass
from random import gauss, randint, uniform
from time import sleep
import tkinter as tk
from annelaer import (
    OptimalPathFinder,
    OptimalPathFromBaseFinder,
    Path,
    WidePathMutator,
    absolute_rand_path,
)
from background import BackgroundFactory, DrawablesFactory
from background.obstacles import register_obstacles
from misc import Position2D
from util import pairs, triples


@dataclass
class PathFromBaseMutator:
    x_var: int
    y_var: int

    def mutate(self, path: list[Position2D], cos_a, sin_a, l) -> list[Position2D]:
        # context aware
        mutant = [0] * len(path)
        # knowing the constext of [base, *path, f]
        rpath = [Position2D(1, 0)]
        rpath.extend(retranslate(pos, cos_a, sin_a) for pos in path)
        rpath.append(Position2D(l - 1, 0))
        for i, p in enumerate(rpath[1:-1]):
            x_max = rpath[i + 2].x - 1
            x_min = rpath[i].x + 1
            if x_max > x_min:
                p.x = gauss(p.x, self.x_var)
                p.x = max(x_min, min(x_max, p.x))
            p.y = gauss(p.y, self.y_var)
            mutant[i] = translate(p, cos_a, sin_a)
        return mutant


# performs better with violet zones
@dataclass
class AccuratePathFromBaseMutator:
    x_var: int
    y_var: int

    def mutate(self, path: list[Position2D], cos_a, sin_a, l) -> list[Position2D]:
        # context aware
        mutant = [0] * len(path)
        # knowing the constext of [base, *path, f]
        rpath = [Position2D(1, 0)]
        rpath.extend(retranslate(pos, cos_a, sin_a) for pos in path)
        rpath.append(Position2D(l - 1, 0))
        s = randint(1, len(rpath) - 2)
        for i, p in enumerate(rpath[1:-1]):
            if i == s:
                x_max = rpath[i + 2].x - 1
                x_min = rpath[i].x + 1
                if x_max > x_min:
                    p.x = gauss(p.x, self.x_var)
                    p.x = max(x_min, min(x_max, p.x))
                p.y = gauss(p.y, self.y_var)
                mutant[i] = translate(p, cos_a, sin_a)
            else:
                mutant[i] = translate(p, cos_a, sin_a)
        return mutant


def translate(pos: Position2D, cos_a, sin_a) -> Position2D:
    # represent back from the rotated coordinate system
    return Position2D(
        pos.x * cos_a - pos.y * sin_a,
        pos.x * sin_a + pos.y * cos_a,
    )


def retranslate(pos: Position2D, cos_a, sin_a) -> Position2D:
    # represent in the rotated coordinate system
    return Position2D(
        pos.x * cos_a + pos.y * sin_a,
        pos.y * cos_a - pos.x * sin_a,
    )


def main(args):
    args.f = Position2D(0, 0)

    df = DrawablesFactory()
    register_obstacles(df)
    bf = BackgroundFactory(df)
    back = bf.read(args.map_file)

    window = tk.Tk()

    canvas = tk.Canvas(window, width=600, height=600, bg="black")

    def draw_background():
        for item in back:
            item.draw(canvas)
        canvas.pack()

    draw_background()

    def penalty(f: Position2D, t: Position2D):
        covered = 0
        total_p = 0
        for item in back:
            if item.penalty is None:
                continue
            cover, p = item.penalty(f, t)
            covered += cover
            total_p += p
        return f.dist(t) - covered + total_p

    def objective(path: list[Position2D]):
        res = 0
        prev = path[0]
        for pos in path[1:]:
            res += penalty(pos, prev)
            prev = pos
        return res

    def draw_path(path: list[Position2D]):
        canvas.delete("all")
        draw_background()
        prev = path[0]
        for pos in path[1:]:
            canvas.create_line(prev.x, prev.y, pos.x, pos.y, fill="red")
            prev = pos
        window.update()

    # def mutate(*a, **kwargs):
    #     m = WidePathMutator(1, 10, 10).mutate(*a, **kwargs)
    #     p = [args.f] + m + [args.t]
    #     draw_path(p)
    #     sleep(0.1)
    #     return m

    def mutate(path: list[Position2D], cos_a, sin_a, l, t) -> list[Position2D]:
        # TODO:
        # (optimize)
        # for a AccuratePathFromBaseMutator
        # mutation does not change the whole path,
        # only a one point, so the new length
        # can be recalculated using the previous length
        m = AccuratePathFromBaseMutator(t * 10, t * 10).mutate(path, cos_a, sin_a, l)
        draw_path([args.f] + m + [args.t])
        # sleep(0.1)
        return m

    # def rand_path(segmentation, *_):
    #     r = [None] * segmentation
    #     for i in range(segmentation):
    #         r[i] = Position2D(uniform(args.f.x, args.t.x), uniform(args.f.y, args.t.y))
    #     p = [args.f] + r + [args.t]
    #     draw_path(p)
    #     return r

    def rand_path_from_base(segmentation, cos_a, sin_a, l):
        res = [None] * segmentation
        for i in range(segmentation):
            x = l * (i + 1) / (segmentation + 1)
            y_max = min(x * cos_a / sin_a, (100 - x * sin_a) / cos_a)
            y_min = max(-x * sin_a / cos_a, (-100 + x * cos_a) / sin_a)
            y = uniform(y_min, y_max)
            res[i] = translate(Position2D(x, y), cos_a, sin_a)
        draw_path([args.f] + res + [args.t])
        return res

    def straight_path_from_base(segmentation, cos_a, sin_a, l):
        res = [None] * segmentation
        for i in range(segmentation):
            x = l * (i + 1) / (segmentation + 1)
            y = 0
            res[i] = translate(Position2D(x, y), cos_a, sin_a)
        draw_path([args.f] + res + [args.t])
        return res

    input()
    steps = 1000
    segmentation = 1
    base = Position2D(0, 0)
    l = args.f.dist(args.t)
    cos_a = (args.t.x) / l
    sin_a = (args.t.y) / l
    p = [base] + rand_path_from_base(segmentation, cos_a, sin_a, l) + [args.f]
    p = Path(p, objective(p))
    for i in range(6):
        p = OptimalPathFromBaseFinder(
            len(p.path) - 2,
            mutate,
            objective,
            p,
            schedule={
                "tmax": 4 / (i + 1),
                "tmin": 0.01,
                "steps": steps,
                "updates": steps,
            },
        ).optimal_path(args.t)

        # insert intermediate points
        tmp = [p.path[0]]
        for prev, curr in pairs(p.path):
            tmp.append(Position2D((prev.x + curr.x) / 2, (prev.y + curr.y) / 2))
            tmp.append(curr)
        p.path = tmp

        draw_path(p.path)

    window.mainloop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("map_file", type=str)
    parser.add_argument("f", type=Position2D.from_str)
    parser.add_argument("t", type=Position2D.from_str)
    main(parser.parse_args())
