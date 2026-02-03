# Reeds-Shepp Curves

A Python implementation of the Reeds-Shepp curves for path planning. This package provides an interface to calculate the optimal paths for a car that can go both forwards and backwards.

Based on the paper:
Reeds, J. A.; Shepp, L. A. Optimal paths for a car that goes both forwards and backwards. Pacific J. Math. 145 (1990), no. 2, 367-393.
[https://projecteuclid.org/euclid.pjm/1102645450](https://projecteuclid.org/euclid.pjm/1102645450)

## Installation

You can install the package locally for development:

```bash
pip install -e .
```

## Usage

The package provides a high-level class-based interface.

### Interface Classes

- `ReedsSheppCurveWaypoint`: Represents a point $(x, y)$ with an orientation $	heta$ (in degrees).
- `ReedsSheppCurvePath`: Represents a path between two waypoints, containing a sequence of `PathElement`s.
- `ReedsSheppCurve`: The main planner class.

### Example

```python
from reeds_shepp import ReedsSheppCurve, ReedsSheppCurveWaypoint

# Define waypoints
waypoints = [
    ReedsSheppCurveWaypoint(0, 0, 0),
    ReedsSheppCurveWaypoint(5, 5, 90),
    ReedsSheppCurveWaypoint(10, 0, 180)
]

# Initialize the planner
planner = ReedsSheppCurve()

# Generate paths between consecutive waypoints
paths = planner.plan(waypoints)

# Iterate through paths
for i, path in enumerate(paths):
    print(f"Path {i} length: {path.total_length:.2f}")
    for element in path.elements:
        print(f"  {element.steering.name} {element.gear.name} {element.param:.2f}")
```

## Examples

A visual demonstration using the `turtle` module is provided in the `examples` folder:

```bash
python examples/visual_demo.py
```

![Reeds-Shepp curves implementation example](demo.gif)

## Testing

The package uses `pytest` for testing:

```bash
pytest
```

## Author

- **sezer-muhammed** - [sezer@imsezer.com](mailto:sezer@imsezer.com)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.