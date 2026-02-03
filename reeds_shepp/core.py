"""
Implementation of the optimal path formulas given in the following paper:

OPTIMAL PATHS FOR A CAR THAT GOES BOTH FORWARDS AND BACKWARDS
J. A. REEDS AND L. A. SHEPP

notes: there are some typos in the formulas given in the paper;
some formulas have been adapted (cf http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c)
"""

import math
import itertools
import heapq
from enum import Enum
from dataclasses import dataclass, replace, field
from typing import List, Optional, Tuple
from .utils import M, R, change_of_basis, deg2rad


class Steering(Enum):
    LEFT = -1
    RIGHT = 1
    STRAIGHT = 0


class Gear(Enum):
    FORWARD = 1
    BACKWARD = -1


@dataclass(eq=True)
class PathElement:
    param: float
    steering: Steering
    gear: Gear

    @classmethod
    def create(cls, param: float, steering: Steering, gear: Gear):
        if param >= 0:
            return cls(param, steering, gear)
        else:
            return cls(-param, steering, gear).reverse_gear()

    def __repr__(self):
        s = "{ Steering: " + self.steering.name + "\tGear: " + self.gear.name \
            + "\tdistance: " + str(round(self.param, 2)) + " }"
        return s

    def reverse_steering(self):
        steering = Steering(-self.steering.value)
        return replace(self, steering=steering)

    def reverse_gear(self):
        gear = Gear(-self.gear.value)
        return replace(self, gear=gear)


@dataclass
class ReedsSheppCurveWaypoint:
    x: float
    y: float
    theta: float  # in degrees


@dataclass
class ReedsSheppCurvePath:
    elements: List[PathElement]
    waypoints: List[ReedsSheppCurveWaypoint]
    _total_length: Optional[float] = field(default=None, init=False, repr=False, compare=False)

    @property
    def total_length(self) -> float:
        if self._total_length is None:
            self._total_length = sum(e.param for e in self.elements)
        return self._total_length


class ReedsSheppCurve:
    def plan(
        self,
        waypoints: List[ReedsSheppCurveWaypoint],
        *,
        max_paths: Optional[int] = 10,
        max_paths_per_segment: Optional[int] = None,
    ) -> List[ReedsSheppCurvePath]:
        """
        Return all possible paths between the waypoints, sorted from shortest to longest.
        Each path visits all waypoints in the provided order.
        By default, returns up to 10 paths for better scaling. Use max_paths=None to return all.
        Optional parameters can further limit the number of paths returned.
        """
        if not waypoints:
            return []
        if len(waypoints) == 1:
            return [ReedsSheppCurvePath([], waypoints)]
        if max_paths is not None and max_paths <= 0:
            return []
        if max_paths_per_segment is not None and max_paths_per_segment <= 0:
            return []

        # Get all path variants for each segment
        segments_variants: List[List[Tuple[List[PathElement], float]]] = []
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i+1]
            start_tuple = (start.x, start.y, start.theta)
            end_tuple = (end.x, end.y, end.theta)
            variants = get_all_paths(start_tuple, end_tuple)
            if not variants:
                return []
            variants_with_len = [(v, path_length(v)) for v in variants]
            variants_with_len.sort(key=lambda t: t[1])
            if max_paths_per_segment is not None:
                variants_with_len = variants_with_len[:max_paths_per_segment]
            segments_variants.append(variants_with_len)

        if max_paths == 1:
            full_elements = []
            total_length = 0.0
            for variants in segments_variants:
                best_elements, best_length = variants[0]
                full_elements.extend(best_elements)
                total_length += best_length
            path = ReedsSheppCurvePath(full_elements, waypoints)
            path._total_length = total_length
            return [path]

        if max_paths is None:
            # Generate all combinations of segments
            full_paths_with_len: List[Tuple[float, ReedsSheppCurvePath]] = []
            # combination will be a tuple of lists of PathElements and their lengths
            for combination in itertools.product(*segments_variants):
                total_len = sum(seg_len for _, seg_len in combination)
                full_elements = [e for segment, _ in combination for e in segment]
                path = ReedsSheppCurvePath(full_elements, waypoints)
                path._total_length = total_len
                full_paths_with_len.append((total_len, path))

            # Sort by length
            full_paths_with_len.sort(key=lambda t: t[0])
            return [path for _, path in full_paths_with_len]

        # Compute the k shortest combinations without expanding all products.
        k = max_paths
        partials = segments_variants[0]
        for segment in segments_variants[1:]:
            partials = _combine_k_shortest(partials, segment, k)
            if not partials:
                return []
        paths = []
        for elements, total_len in partials:
            path = ReedsSheppCurvePath(elements, waypoints)
            path._total_length = total_len
            paths.append(path)
        return paths


def _combine_k_shortest(
    left: List[Tuple[List[PathElement], float]],
    right: List[Tuple[List[PathElement], float]],
    k: int,
) -> List[Tuple[List[PathElement], float]]:
    """
    Combine two sorted (by length) lists of path variants and return the k shortest.
    Path lengths are non-negative, so keeping only the k shortest partials is safe.
    """
    if not left or not right:
        return []
    max_k = min(k, len(left) * len(right))
    heap: List[Tuple[float, int, int]] = []
    visited = set()

    heapq.heappush(heap, (left[0][1] + right[0][1], 0, 0))
    visited.add((0, 0))

    result: List[Tuple[List[PathElement], float]] = []
    while heap and len(result) < max_k:
        total, i, j = heapq.heappop(heap)
        result.append((left[i][0] + right[j][0], total))

        if i + 1 < len(left) and (i + 1, j) not in visited:
            heapq.heappush(heap, (left[i + 1][1] + right[j][1], i + 1, j))
            visited.add((i + 1, j))
        if j + 1 < len(right) and (i, j + 1) not in visited:
            heapq.heappush(heap, (left[i][1] + right[j + 1][1], i, j + 1))
            visited.add((i, j + 1))

    return result


def path_length(path):
    """
    this one's obvious
    """
    return sum(e.param for e in path)


def get_optimal_path(start, end):
    """
    Return the shortest path from start to end among those that exist
    """
    paths = get_all_paths(start, end)
    return min(paths, key=path_length)


def get_all_paths(start, end):
    """
    Return a list of all the paths from start to end generated by the
    12 functions and their variants
    """
    paths = []

    # get coordinates of end in the set of axis where start is (0,0,0)
    x, y, theta = change_of_basis(start, end)

    for get_path in PATH_FUNCTIONS:
        # get the four variants for each path type, cf article
        for path in (
            get_path(x, y, theta),
            timeflip(get_path(-x, y, -theta)),
            reflect(get_path(x, -y, -theta)),
            reflect(timeflip(get_path(-x, -y, theta))),
        ):
            if not path:
                continue
            if any(e.param == 0 for e in path):
                path = [e for e in path if e.param != 0]
                if not path:
                    continue
            paths.append(path)

    return paths


def timeflip(path):
    """
    timeflip transform described around the end of the article
    """
    new_path = [e.reverse_gear() for e in path]
    return new_path


def reflect(path):
    """
    reflect transform described around the end of the article
    """
    new_path = [e.reverse_steering() for e in path]
    return new_path


def path1(x, y, phi):
    """
    Formula 8.1: CSC (same turns)
    """
    phi = deg2rad(phi)
    path = []

    u, t = R(x - math.sin(phi), y - 1 + math.cos(phi))
    v = M(phi - t)

    path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
    path.append(PathElement.create(u, Steering.STRAIGHT, Gear.FORWARD))
    path.append(PathElement.create(v, Steering.LEFT, Gear.FORWARD))

    return path


def path2(x, y, phi):
    """
    Formula 8.2: CSC (opposite turns)
    """
    phi = M(deg2rad(phi))
    path = []

    rho, t1 = R(x + math.sin(phi), y - 1 - math.cos(phi))

    if rho * rho >= 4:
        u = math.sqrt(rho * rho - 4)
        t = M(t1 + math.atan2(2, u))
        v = M(t - phi)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(u, Steering.STRAIGHT, Gear.FORWARD))
        path.append(PathElement.create(v, Steering.RIGHT, Gear.FORWARD))

    return path


def path3(x, y, phi):
    """
    Formula 8.3: C|C|C
    """
    phi = deg2rad(phi)
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        A = math.acos(rho / 4)
        t = M(theta + math.pi/2 + A)
        u = M(math.pi - 2*A)
        v = M(phi - t - u)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(u, Steering.RIGHT, Gear.BACKWARD))
        path.append(PathElement.create(v, Steering.LEFT, Gear.FORWARD))

    return path


def path4(x, y, phi):
    """
    Formula 8.4 (1): C|CC
    """
    phi = deg2rad(phi)
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        A = math.acos(rho / 4)
        t = M(theta + math.pi/2 + A)
        u = M(math.pi - 2*A)
        v = M(t + u - phi)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(u, Steering.RIGHT, Gear.BACKWARD))
        path.append(PathElement.create(v, Steering.LEFT, Gear.BACKWARD))

    return path


def path5(x, y, phi):
    """
    Formula 8.4 (2): CC|C
    """
    phi = deg2rad(phi)
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        u = math.acos(1 - rho*rho/8)
        A = math.asin(2 * math.sin(u) / rho)
        t = M(theta + math.pi/2 - A)
        v = M(t - u - phi)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(u, Steering.RIGHT, Gear.FORWARD))
        path.append(PathElement.create(v, Steering.LEFT, Gear.BACKWARD))

    return path


def path6(x, y, phi):
    """
    Formula 8.7: CCu|CuC
    """
    phi = deg2rad(phi)
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        if rho <= 2:
            A = math.acos((rho + 2) / 4)
            t = M(theta + math.pi/2 + A)
            u = M(A)
            v = M(phi - t + 2*u)
        else:
            A = math.acos((rho - 2) / 4)
            t = M(theta + math.pi/2 - A)
            u = M(math.pi - A)
            v = M(phi - t + 2*u)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(u, Steering.RIGHT, Gear.FORWARD))
        path.append(PathElement.create(u, Steering.LEFT, Gear.BACKWARD))
        path.append(PathElement.create(v, Steering.RIGHT, Gear.BACKWARD))

    return path


def path7(x, y, phi):
    """
    Formula 8.8: C|CuCu|C
    """
    phi = deg2rad(phi)
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)
    u1 = (20 - rho*rho) / 16

    if rho <= 6 and 0 <= u1 <= 1:
        u = math.acos(u1)
        A = math.asin(2 * math.sin(u) / rho)
        t = M(theta + math.pi/2 + A)
        v = M(t - phi)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(u, Steering.RIGHT, Gear.BACKWARD))
        path.append(PathElement.create(u, Steering.LEFT, Gear.BACKWARD))
        path.append(PathElement.create(v, Steering.RIGHT, Gear.FORWARD))

    return path


def path8(x, y, phi):
    """
    Formula 8.9 (1): C|C[pi/2]SC
    """
    phi = deg2rad(phi)
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        u = math.sqrt(rho*rho - 4) - 2
        A = math.atan2(2, u+2)
        t = M(theta + math.pi/2 + A)
        v = M(t - phi + math.pi/2)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(math.pi/2, Steering.RIGHT, Gear.BACKWARD))
        path.append(PathElement.create(u, Steering.STRAIGHT, Gear.BACKWARD))
        path.append(PathElement.create(v, Steering.LEFT, Gear.BACKWARD))

    return path


def path9(x, y, phi):
    """
    Formula 8.9 (2): CSC[pi/2]|C
    """
    phi = deg2rad(phi)
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        u = math.sqrt(rho*rho - 4) - 2
        A = math.atan2(u+2, 2)
        t = M(theta + math.pi/2 - A)
        v = M(t - phi - math.pi/2)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(u, Steering.STRAIGHT, Gear.FORWARD))
        path.append(PathElement.create(math.pi/2, Steering.RIGHT, Gear.FORWARD))
        path.append(PathElement.create(v, Steering.LEFT, Gear.BACKWARD))

    return path


def path10(x, y, phi):
    """
    Formula 8.10 (1): C|C[pi/2]SC
    """
    phi = deg2rad(phi)
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        t = M(theta + math.pi/2)
        u = rho - 2
        v = M(phi - t - math.pi/2)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(math.pi/2, Steering.RIGHT, Gear.BACKWARD))
        path.append(PathElement.create(u, Steering.STRAIGHT, Gear.BACKWARD))
        path.append(PathElement.create(v, Steering.RIGHT, Gear.BACKWARD))

    return path


def path11(x, y, phi):
    """
    Formula 8.10 (2): CSC[pi/2]|C
    """
    phi = deg2rad(phi)
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        t = M(theta)
        u = rho - 2
        v = M(phi - t - math.pi/2)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(u, Steering.STRAIGHT, Gear.FORWARD))
        path.append(PathElement.create(math.pi/2, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(v, Steering.RIGHT, Gear.BACKWARD))

    return path


def path12(x, y, phi):
    """
    Formula 8.11: C|C[pi/2]SC[pi/2]|C
    """
    phi = deg2rad(phi)
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 4:
        u = math.sqrt(rho*rho - 4) - 4
        A = math.atan2(2, u+4)
        t = M(theta + math.pi/2 + A)
        v = M(t - phi)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(math.pi/2, Steering.RIGHT, Gear.BACKWARD))
        path.append(PathElement.create(u, Steering.STRAIGHT, Gear.BACKWARD))
        path.append(PathElement.create(math.pi/2, Steering.LEFT, Gear.BACKWARD))
        path.append(PathElement.create(v, Steering.RIGHT, Gear.FORWARD))

    return path


# Static tuple to avoid re-allocating on every call to get_all_paths.
PATH_FUNCTIONS = (
    path1,
    path2,
    path3,
    path4,
    path5,
    path6,
    path7,
    path8,
    path9,
    path10,
    path11,
    path12,
)
