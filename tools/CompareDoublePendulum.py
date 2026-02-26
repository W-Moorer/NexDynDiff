import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def read_vector_csv(path: Path):
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return data[:, 0], data[:, 1:4]


def interpolate_reference(reference_t, reference_v, target_t):
    out = np.zeros((target_t.size, 3), dtype=float)
    for axis in range(3):
        out[:, axis] = np.interp(target_t, reference_t, reference_v[:, axis])
    return out


def compute_metrics(reference_v, simulation_v):
    error = simulation_v - reference_v
    rmse = np.sqrt(np.mean(error * error, axis=0))
    mae = np.mean(np.abs(error), axis=0)
    max_abs = np.max(np.abs(error), axis=0)
    return rmse, mae, max_abs


def main():
    parser = argparse.ArgumentParser(description="Compare NexDynDiff double pendulum results with RecurDyn CSVs.")
    parser.add_argument(
        "--reference-dir",
        default="assets/models/double_pendulum",
        help="Directory containing RecurDyn CSV files.",
    )
    parser.add_argument(
        "--simulation-dir",
        default="output/double_pendulum",
        help="Directory containing NexDynDiff CSV files.",
    )
    parser.add_argument(
        "--figure-dir",
        default="output/figures/double_pendulum",
        help="Directory for output figures.",
    )
    args = parser.parse_args()

    reference_dir = Path(args.reference_dir)
    simulation_dir = Path(args.simulation_dir)
    figure_dir = Path(args.figure_dir)
    figure_dir.mkdir(parents=True, exist_ok=True)

    plt.rcParams.update(
        {
            "font.family": "Times New Roman",
            "font.size": 11,
        }
    )

    quantities = ["Pos", "Vel", "Acc"]
    axis_labels = ["X", "Y", "Z"]
    metrics_rows = []

    for quantity in quantities:
        fig, axes = plt.subplots(2, 3, figsize=(16, 8), constrained_layout=True, sharex=True)
        for link_idx in [1, 2]:
            reference_path = reference_dir / f"Link{link_idx}_{quantity}.csv"
            simulation_path = simulation_dir / f"Link{link_idx}_{quantity}_NexDynDiff.csv"

            simulation_t, simulation_v = read_vector_csv(simulation_path)
            reference_t, reference_v = read_vector_csv(reference_path)
            reference_interp = interpolate_reference(reference_t, reference_v, simulation_t)

            rmse, mae, max_abs = compute_metrics(reference_interp, simulation_v)
            for axis in range(3):
                metrics_rows.append(
                    {
                        "dataset": f"Link{link_idx}_{quantity}",
                        "axis": axis_labels[axis],
                        "rmse": rmse[axis],
                        "mae": mae[axis],
                        "max_abs": max_abs[axis],
                    }
                )

            for axis in range(3):
                ax = axes[link_idx - 1, axis]
                ax.plot(simulation_t, reference_interp[:, axis], color="black", linewidth=1.6, label="RecurDyn")
                ax.plot(simulation_t, simulation_v[:, axis], color="#C0504D", linewidth=1.3, linestyle="--", label="NexDynDiff")
                ax.set_title(f"Link{link_idx} {quantity} {axis_labels[axis]}")
                ax.set_xlabel("Time (s)")
                ax.set_ylabel("Value")
                ax.grid(True, alpha=0.3)
                if link_idx == 1 and axis == 0:
                    ax.legend(loc="best")

        figure_path = figure_dir / f"{quantity}_Comparison.png"
        fig.savefig(figure_path, dpi=160)
        plt.close(fig)

    metrics_path = simulation_dir / "DoublePendulumComparisonMetrics.csv"
    with open(metrics_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["dataset", "axis", "rmse", "mae", "max_abs"])
        writer.writeheader()
        writer.writerows(metrics_rows)

    summary_path = simulation_dir / "DoublePendulumComparisonSummary.txt"
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write("Double Pendulum Comparison Summary\n")
        f.write("Reference: RecurDyn CSV\n")
        f.write("Simulation: NexDynDiff scene JSON\n\n")
        for quantity in quantities:
            f.write(f"[{quantity}]\n")
            quantity_rows = [row for row in metrics_rows if row["dataset"].endswith(quantity)]
            avg_rmse = np.mean([row["rmse"] for row in quantity_rows])
            avg_mae = np.mean([row["mae"] for row in quantity_rows])
            max_abs = np.max([row["max_abs"] for row in quantity_rows])
            f.write(f"  average_rmse: {avg_rmse:.8e}\n")
            f.write(f"  average_mae : {avg_mae:.8e}\n")
            f.write(f"  max_abs     : {max_abs:.8e}\n")
            f.write("\n")

    print(f"Figures written to: {figure_dir}")
    print(f"Metrics written to: {metrics_path}")
    print(f"Summary written to: {summary_path}")


if __name__ == "__main__":
    main()

