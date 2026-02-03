"""Benchmark Reed-Shepp planning as number of waypoints grows.

Runs trials for waypoint counts from 2..20. For each trial it spawns a
separate process to call the planner with a timeout to avoid combinatorial
explosion for large waypoint counts. Results are printed and saved to
`examples/benchmark_results.csv`.
"""
import csv
import random
import time
import multiprocessing as mp
from typing import List

from reeds_shepp import ReedsSheppCurve, ReedsSheppCurveWaypoint


def generate_waypoints(n: int, span: float = 10.0, seed: int | None = None) -> List[ReedsSheppCurveWaypoint]:
    """Generate n random waypoints inside [-span, span] with random headings."""
    if seed is not None:
        random.seed(seed)
    pts = []
    for _ in range(n):
        x = random.uniform(-span, span)
        y = random.uniform(-span, span)
        theta = random.uniform(-180.0, 180.0)
        pts.append(ReedsSheppCurveWaypoint(x, y, theta))
    return pts


def planner_worker(conn, waypoints):
    """Child process target: run planner.plan and send back stats."""
    start = time.perf_counter()
    planner = ReedsSheppCurve()
    paths = planner.plan(waypoints)
    elapsed = time.perf_counter() - start
    # send back (success, elapsed, num_alternatives, shortest_length)
    if paths:
        shortest = paths[0].total_length
        num = len(paths)
    else:
        shortest = 0.0
        num = 0
    conn.send((True, elapsed, num, shortest))
    conn.close()


def run_with_timeout(waypoints, timeout_s=10.0):
    parent_conn, child_conn = mp.Pipe()
    p = mp.Process(target=planner_worker, args=(child_conn, waypoints))
    p.start()
    p.join(timeout_s)
    if p.is_alive():
        p.terminate()
        p.join()
        return (False, None, None, None)
    if parent_conn.poll():
        return parent_conn.recv()
    return (False, None, None, None)


def main():
    random.seed(0)
    results = []
    trials_per_n = 3
    timeout_s = 5.0

    for n in range(2, 21):
        print(f"Running n={n} waypoints (" + str(trials_per_n) + " trials) ...")
        for t in range(trials_per_n):
            waypoints = generate_waypoints(n)
            ok, elapsed, num_paths, shortest = run_with_timeout(waypoints, timeout_s=timeout_s)
            if ok:
                print(f" n={n:2d} trial={t+1} time={elapsed:.4f}s alternatives={num_paths} shortest_len={shortest:.4f}")
                results.append((n, t+1, True, round(elapsed, 6), num_paths, round(shortest, 6)))
            else:
                print(f" n={n:2d} trial={t+1} TIMEOUT after {timeout_s}s")
                results.append((n, t+1, False, None, None, None))

    # Save results
    out_csv = "examples/benchmark_results.csv"
    with open(out_csv, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["n_waypoints", "trial", "ok", "elapsed_s", "num_alternatives", "shortest_length"])
        for row in results:
            writer.writerow(row)

    print(f"Benchmark finished. Results written to {out_csv}")


if __name__ == '__main__':
    # Multiprocessing on Windows requires the __main__ guard.
    main()
