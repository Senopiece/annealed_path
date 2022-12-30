from dataclasses import dataclass
from random import gauss, uniform
from misc import Position2D
from simanneal import Annealer


@dataclass
class Path:
    path: list[Position2D]
    length: float


# TODO: mutators that use coordinate system rotation (they converge very fast)


@dataclass
class CrazyPathMutator:
    threshold: float

    def mutate(self, path: list[Position2D], *_) -> list[Position2D]:
        return [
            Position2D(uniform(0, 10000), uniform(0, 10000))
            if uniform(0, 1) < self.threshold
            else c
            for c in path
        ]


@dataclass
class WidePathMutator:
    threshold: float
    x_var: float
    y_var: float

    def mutate(self, path: list[Position2D], *_) -> list[Position2D]:
        return [
            Position2D(
                min(max(gauss(c.x, self.x_var), 0), 10000),
                min(max(gauss(c.y, self.y_var), 0), 10000),
            )
            if uniform(0, 1) < self.threshold
            else c
            for c in path
        ]


# TODO: coordinate system aware rand_path_generator (makes convergence faster)


def absolute_rand_path(segmentation, *_):
    res = [None] * segmentation
    for i in range(segmentation):
        res[i] = Position2D(uniform(0, 10000), uniform(0, 10000))
    return res


class OptimalPathFinder:
    segmentation: int
    mutate: callable  # context aware
    objective: callable  # context unaware
    rand_path_generator: callable  # context aware
    schedule: dict = {"tmax": 100.0, "tmin": 1, "steps": 340, "updates": 100}

    def __init__(
        self,
        segmentation,
        mutate,
        objective,
        rand_path_generator=None,
        schedule=None,
    ):
        self.segmentation = segmentation
        self.mutate = mutate
        self.objective = objective
        if rand_path_generator is None:
            self.rand_path_generator = absolute_rand_path
        else:
            self.rand_path_generator = rand_path_generator
        if schedule is not None:
            self.schedule = schedule

    def optimal_path(self, f: Position2D, t: Position2D) -> Path:
        mutate = self.mutate
        objective = self.objective

        l = f.dist(t)
        if l < 200:
            return Path([f, t], objective([f, t]))

        cos_a = f.x / l
        sin_a = f.y / l

        class PathAnnealer(Annealer):
            def move(self):
                path = [f] + mutate(self.state.path[1:-1], cos_a, sin_a, l) + [t]
                length = objective(path)
                self.state = Path(path, length)

            def energy(self):
                return self.state.length

        init = [f] + self.rand_path_generator(self.segmentation, cos_a, sin_a, l) + [t]
        annealer = PathAnnealer(Path(init, objective(init)))
        annealer.set_schedule(self.schedule)
        best, cost = annealer.anneal()
        return best


base = Position2D(0, 0)


class OptimalPathFromBaseFinder:
    segmentation: int
    mutate: callable  # context aware
    objective: callable  # context unaware
    init: Path
    schedule: dict = {"tmax": 100.0, "tmin": 1, "steps": 340, "updates": 100}

    def __init__(
        self,
        segmentation,
        mutate,
        objective,
        init,
        schedule=None,
    ):
        self.segmentation = segmentation
        self.mutate = mutate
        self.objective = objective
        self.init = init
        if schedule is not None:
            self.schedule = schedule

    def optimal_path(self, f: Position2D) -> Path:
        mutate = self.mutate
        objective = self.objective

        l = f.dist(base)
        if l < 200:
            return Path([base, f], objective([base, f]))

        cos_a = f.x / l
        sin_a = f.y / l

        class PathAnnealer(Annealer):
            def move(self):
                path = (
                    [base]
                    + mutate(self.state.path[1:-1], cos_a, sin_a, l, self.t)
                    + [f]
                )
                length = objective(path)
                self.state = Path(path, length)

            def energy(self):
                return self.state.length

            def update(self, *args, **kwargs):
                self.t = args[1]
                self.default_update(*args, **kwargs)

        annealer = PathAnnealer(self.init)
        annealer.set_schedule(self.schedule)
        best, cost = annealer.anneal()
        return best
