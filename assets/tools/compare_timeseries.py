import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple


@dataclass
class CompareStats:
    max_abs_error: float
    mean_abs_error: float
    rmse: float


def read_two_column_csv(path: Path, time_col: int = 0, value_col: int = 1) -> Tuple[List[float], List[float]]:
    t: List[float] = []
    y: List[float] = []
    with path.open("r", encoding="utf-8", errors="ignore", newline="") as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if len(row) <= max(time_col, value_col):
                continue
            try:
                t.append(float(row[time_col]))
                y.append(float(row[value_col]))
            except ValueError:
                continue
    return t, y


def linear_interp(x: List[float], y: List[float], xq: float) -> float:
    if not x:
        raise ValueError("empty series")
    if xq <= x[0]:
        return y[0]
    if xq >= x[-1]:
        return y[-1]
    lo = 0
    hi = len(x) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if x[mid] <= xq:
            lo = mid
        else:
            hi = mid
    x0, x1 = x[lo], x[hi]
    y0, y1 = y[lo], y[hi]
    a = (xq - x0) / max(x1 - x0, 1e-15)
    return y0 * (1.0 - a) + y1 * a


def compare_series(
    baseline_t: List[float],
    baseline_y: List[float],
    sim_t: List[float],
    sim_y: List[float],
) -> CompareStats:
    import math

    errors = []
    for t, yb in zip(baseline_t, baseline_y):
        ys = linear_interp(sim_t, sim_y, t)
        errors.append(abs(ys - yb))

    max_abs = max(errors) if errors else 0.0
    mean_abs = sum(errors) / len(errors) if errors else 0.0
    rmse = math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else 0.0
    return CompareStats(max_abs, mean_abs, rmse)


def main():
    parser = argparse.ArgumentParser(description="Compare scalar time-series CSV with interpolation.")
    parser.add_argument("--baseline", required=True, help="Baseline CSV path")
    parser.add_argument("--sim", required=True, help="Simulation CSV path")
    parser.add_argument("--baseline-time-col", type=int, default=0)
    parser.add_argument("--baseline-value-col", type=int, default=1)
    parser.add_argument("--sim-time-col", type=int, default=0)
    parser.add_argument("--sim-value-col", type=int, default=1)
    parser.add_argument("--max-abs-threshold", type=float, default=None)
    args = parser.parse_args()

    bt, by = read_two_column_csv(Path(args.baseline), args.baseline_time_col, args.baseline_value_col)
    st, sy = read_two_column_csv(Path(args.sim), args.sim_time_col, args.sim_value_col)

    stats = compare_series(bt, by, st, sy)
    print(f"max_abs_error={stats.max_abs_error:.10g}")
    print(f"mean_abs_error={stats.mean_abs_error:.10g}")
    print(f"rmse={stats.rmse:.10g}")

    if args.max_abs_threshold is not None and stats.max_abs_error > args.max_abs_threshold:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
