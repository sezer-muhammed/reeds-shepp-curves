import turtle
import math
import random as rd
from reeds_shepp import (
    ReedsSheppCurve,
    ReedsSheppCurveWaypoint,
    Steering,
    Gear
)

# drawing n units (eg turtle.forward(n)) will draw n * SCALE pixels
SCALE = 40

def scale(x):
    if type(x) is tuple or type(x) is list:
        return [p * SCALE for p in x]
    return x * SCALE

def rad2deg(rad):
    return 180 * rad / math.pi

def vec(bob):
    bob.down()
    bob.pensize(3)
    bob.forward(scale(1.2))
    bob.right(25)
    bob.backward(scale(.4))
    bob.forward(scale(.4))
    bob.left(50)
    bob.backward(scale(.4))
    bob.forward(scale(.4))
    bob.right(25)
    bob.pensize(1)
    bob.up()

def goto(bob, waypoint: ReedsSheppCurveWaypoint):
    bob.up()
    bob.setpos(scale(waypoint.x), scale(waypoint.y))
    bob.setheading(waypoint.theta)
    bob.down()

def draw_path(bob, elements):
    for e in elements:
        gear = 1 if e.gear == Gear.FORWARD else -1
        if e.steering == Steering.LEFT:
            bob.circle(scale(1), gear * rad2deg(e.param))
        elif e.steering == Steering.RIGHT:
            bob.circle(- scale(1), gear * rad2deg(e.param))
        elif e.steering == Steering.STRAIGHT:
            bob.forward(gear * scale(e.param))

def set_random_pencolor(bob):
    r, g, b = 1, 1, 1
    while r + g + b > 2.5:
        r, g, b = rd.uniform(0, 1), rd.uniform(0, 1), rd.uniform(0, 1)
    bob.pencolor(r, g, b)

def main():
    # points to be followed
    pts = [(-6,-7), (-6,0), (-4, 6), (0, 5), (0,-2), (-2, -6), (3, -5), (3, 6), (6, 4)]

    waypoints = []
    for i in range(len(pts) - 1):
        dx = pts[i+1][0] - pts[i][0]
        dy = pts[i+1][1] - pts[i][1]
        theta = math.atan2(dy, dx)
        waypoints.append(ReedsSheppCurveWaypoint(pts[i][0], pts[i][1], rad2deg(theta)))
    waypoints.append(ReedsSheppCurveWaypoint(pts[-1][0], pts[-1][1], 0))

    # init turtle
    tesla = turtle.Turtle()
    tesla.speed(0)
    tesla.shape('arrow')
    tesla.resizemode('user')
    tesla.shapesize(1, 1)

    # draw vectors representing points in waypoints
    for pt in waypoints:
        goto(tesla, pt)
        vec(tesla)

    # Use the new API
    planner = ReedsSheppCurve()
    all_full_paths = planner.plan(waypoints)
    
    if not all_full_paths:
        print("No paths found.")
        return

    # Draw shortest route
    shortest_path = all_full_paths[0]
    tesla.pencolor(1, 0, 0)
    tesla.pensize(3)
    tesla.speed(10)
    
    goto(tesla, shortest_path.waypoints[0])
    draw_path(tesla, shortest_path.elements)

    print("Shortest path length: {} px.".format(int(scale(shortest_path.total_length))))
    print(f"Total alternatives found: {len(all_full_paths)}")

    turtle.done()

if __name__ == '__main__':
    main()