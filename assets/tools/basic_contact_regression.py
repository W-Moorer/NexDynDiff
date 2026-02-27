import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

from compare_timeseries import read_two_column_csv, compare_series
from rmd_to_scene import convert


def _save_svg_comparison(
    baseline_t,
    baseline_y,
    sim_t,
    sim_y,
    out_svg: Path,
    title: str,
):
    if not baseline_t or not baseline_y or not sim_t or not sim_y:
        return

    x_min = min(min(baseline_t), min(sim_t))
    x_max = max(max(baseline_t), max(sim_t))
    y_min = min(min(baseline_y), min(sim_y))
    y_max = max(max(baseline_y), max(sim_y))

    if abs(x_max - x_min) < 1e-15:
        x_max = x_min + 1.0
    if abs(y_max - y_min) < 1e-15:
        y_max = y_min + 1.0

    width, height = 1200, 760
    left, right, top, bottom = 110, 40, 70, 95
    plot_w = width - left - right
    plot_h = height - top - bottom
    font_family = "Times New Roman, Times, serif"

    def map_x(x):
        return left + (x - x_min) / (x_max - x_min) * plot_w

    def map_y(y):
        return top + (y_max - y) / (y_max - y_min) * plot_h

    def polyline_points(tx, yy):
        return " ".join(f"{map_x(x):.2f},{map_y(y):.2f}" for x, y in zip(tx, yy))

    def tick_values(vmin, vmax, n):
        if n <= 1:
            return [vmin]
        step = (vmax - vmin) / (n - 1)
        return [vmin + i * step for i in range(n)]

    x_ticks = tick_values(x_min, x_max, n=6)

    y_tick_step = 0.01
    y_tick_start = math.floor(y_min / y_tick_step) * y_tick_step
    y_tick_end = math.ceil(y_max / y_tick_step) * y_tick_step
    y_ticks = []
    yv = y_tick_start
    while yv <= y_tick_end + 1e-12:
        y_ticks.append(yv)
        yv += y_tick_step
    if len(y_ticks) < 2:
        y_ticks = tick_values(y_min, y_max, n=5)

    def fmt_tick(v):
        a = abs(v)
        if a >= 1e3 or (a > 0 and a < 1e-3):
            return f"{v:.1e}"
        return f"{v:.4g}"

    baseline_points = polyline_points(baseline_t, baseline_y)
    sim_points = polyline_points(sim_t, sim_y)

    x_tick_svg = []
    for xv in x_ticks:
        px = map_x(xv)
        x_tick_svg.append(
            f'<line x1="{px:.2f}" y1="{top+plot_h:.2f}" x2="{px:.2f}" y2="{top+plot_h+7:.2f}" stroke="#111" stroke-width="1.4"/>'
        )
        x_tick_svg.append(
            f'<text x="{px:.2f}" y="{top+plot_h+28:.2f}" text-anchor="middle" font-size="20" font-family="{font_family}">{fmt_tick(xv)}</text>'
        )

    y_tick_svg = []
    for yv in y_ticks:
        py = map_y(yv)
        y_tick_svg.append(
            f'<line x1="{left:.2f}" y1="{py:.2f}" x2="{left+plot_w:.2f}" y2="{py:.2f}" stroke="#c8c8c8" stroke-width="1.0"/>'
        )
        y_tick_svg.append(
            f'<line x1="{left-7:.2f}" y1="{py:.2f}" x2="{left:.2f}" y2="{py:.2f}" stroke="#111" stroke-width="1.4"/>'
        )
        y_tick_svg.append(
            f'<text x="{left-12:.2f}" y="{py+6:.2f}" text-anchor="end" font-size="20" font-family="{font_family}">{fmt_tick(yv)}</text>'
        )

    legend_x = left + plot_w - 295
    legend_y = top + 10

    svg = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect x="0" y="0" width="{width}" height="{height}" fill="white"/>
  <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="#111" stroke-width="2.0"/>
  <text x="{width/2:.1f}" y="36" text-anchor="middle" font-size="24" font-family="{font_family}">{title}</text>
  <text x="{width/2:.1f}" y="{height-28}" text-anchor="middle" font-size="24" font-family="{font_family}">Time (s)</text>
  <text x="28" y="{height/2:.1f}" transform="rotate(-90,28,{height/2:.1f})" text-anchor="middle" font-size="24" font-family="{font_family}">Body2 Y (m)</text>

  {' '.join(x_tick_svg)}
  {' '.join(y_tick_svg)}

  <polyline fill="none" stroke="#1f77b4" stroke-width="2" points="{baseline_points}"/>
  <polyline fill="none" stroke="#d62728" stroke-width="2" points="{sim_points}"/>

  <line x1="{legend_x+14}" y1="{legend_y+20}" x2="{legend_x+48}" y2="{legend_y+20}" stroke="#1f77b4" stroke-width="3"/>
  <text x="{legend_x+56}" y="{legend_y+26}" font-size="20" font-family="{font_family}">RecurDyn baseline</text>
  <line x1="{legend_x+14}" y1="{legend_y+44}" x2="{legend_x+48}" y2="{legend_y+44}" stroke="#d62728" stroke-width="3"/>
  <text x="{legend_x+56}" y="{legend_y+50}" font-size="20" font-family="{font_family}">NexDynDiff</text>
</svg>
'''

    out_svg.parent.mkdir(parents=True, exist_ok=True)
    out_svg.write_text(svg, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Run basic_contact regression against Body2_Pos.csv baseline.")
    parser.add_argument("--repo-root", default=None, help="Repo root path")
    parser.add_argument("--exe", required=True, help="Path to NexDynDiff executable")
    parser.add_argument("--rmd", default=None, help="Path to .rmd scene file (default: assets/models/basic_contact/basic_contact.rmd)")
    parser.add_argument("--csv-fps", type=int, default=None, help="CSV sampling FPS for NexDynDiff (default: infer from baseline)")
    parser.add_argument("--ipc-barrier-type", choices=["cubic", "log"], default=None, help="Inject IPC barrier type into scene contact global params")
    parser.add_argument("--ipc-friction-type", choices=["c0", "c1"], default=None, help="Inject IPC friction type into scene contact global params")
    parser.add_argument("--strict-feasibility", action="store_true", help="Enable strict IPC feasibility check (distance threshold) in contact model")
    parser.add_argument("--ccd-eta", type=float, default=None, help="Safety factor for max step and strict feasibility threshold (0~1)")
    parser.add_argument("--max-abs-threshold", type=float, default=2e-2)
    parser.add_argument("--enforce-threshold", action="store_true", help="Exit non-zero when max_abs_error exceeds threshold")
    args = parser.parse_args()

    repo_root = Path(args.repo_root).resolve() if args.repo_root else Path(__file__).resolve().parents[2]
    rmd = Path(args.rmd).resolve() if args.rmd else (repo_root / "assets" / "models" / "basic_contact" / "basic_contact.rmd")
    baseline_csv = rmd.parent / "Body2_Pos.csv"
    rmd_name = rmd.stem

    bt, by = read_two_column_csv(baseline_csv, time_col=0, value_col=2)
    inferred_csv_fps = 1000
    if len(bt) >= 2:
        dt_candidates = [bt[i] - bt[i - 1] for i in range(1, len(bt)) if bt[i] - bt[i - 1] > 1e-12]
        if dt_candidates:
            inferred_csv_fps = max(1, int(round(1.0 / min(dt_candidates))))
    csv_fps = args.csv_fps if args.csv_fps is not None else inferred_csv_fps

    scene_json, _, _ = convert(rmd, repo_root, "json", apply_rm_offset=True)

    with scene_json.open("r", encoding="utf-8") as f:
        scene_data = json.load(f)

    output_cfg = scene_data.setdefault("settings", {}).setdefault("output", {})
    output_cfg["output_directory"] = f"./output/{rmd_name}"

    if (
        args.ipc_barrier_type is not None
        or args.ipc_friction_type is not None
        or args.strict_feasibility
        or args.ccd_eta is not None
    ):
        gp = scene_data.setdefault("interactions", {}).setdefault("contact", {}).setdefault("global_params", {})
        if args.ipc_barrier_type is not None:
            gp["ipc_barrier_type"] = args.ipc_barrier_type
            output_cfg["codegen_directory"] = f"./codegen/rmd_generated_{args.ipc_barrier_type}"
        if args.ipc_friction_type is not None:
            gp["ipc_friction_type"] = args.ipc_friction_type
        if args.strict_feasibility:
            gp["strict_feasibility"] = True
        if args.ccd_eta is not None:
            gp["ccd_eta"] = float(args.ccd_eta)

    with scene_json.open("w", encoding="utf-8") as f:
        json.dump(scene_data, f, indent=2, ensure_ascii=False)

    cmd = [
        str(Path(args.exe).resolve()),
        str(scene_json),
        "--csv-fps",
        str(csv_fps),
        "--quiet",
    ]
    print("Run:", " ".join(cmd))
    subprocess.run(cmd, check=True, cwd=str(repo_root))

    sim_csv = repo_root / "output" / rmd_name / "Body2_Pos.csv"
    if not sim_csv.exists():
        sim_csv = scene_json.parent / "output" / rmd_name / "Body2_Pos.csv"

    st, sy = read_two_column_csv(sim_csv, time_col=0, value_col=2)
    stats = compare_series(bt, by, st, sy)

    print(f"max_abs_error={stats.max_abs_error:.10g}")
    print(f"mean_abs_error={stats.mean_abs_error:.10g}")
    print(f"rmse={stats.rmse:.10g}")
    print(f"csv_fps={csv_fps}")

    fig_dir = repo_root / "output" / "figures" / rmd_name
    fig_path = fig_dir / "Body2_y_compare.svg"
    _save_svg_comparison(bt, by, st, sy, fig_path, title=f"{rmd_name}: Body2 y baseline vs simulation")

    print(f"sim_csv={sim_csv}")
    print(f"figure={fig_path}")

    if stats.max_abs_error > args.max_abs_threshold:
        msg = f"max_abs_error exceeds threshold: {stats.max_abs_error:.10g} > {args.max_abs_threshold:.10g}"
        if args.enforce_threshold:
            print(f"Regression failed: {msg}", file=sys.stderr)
            raise SystemExit(2)
        print(f"Warning: {msg}")


if __name__ == "__main__":
    main()
