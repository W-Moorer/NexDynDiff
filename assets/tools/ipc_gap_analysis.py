import argparse
import csv
import json
import math
from pathlib import Path


def read_time_pos_y(csv_path: Path):
    t, y = [], []
    with csv_path.open("r", encoding="utf-8", errors="ignore") as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if len(row) < 3:
                continue
            t.append(float(row[0]))
            y.append(float(row[2]))
    return t, y


def read_solver_hard_guarantee_csv(csv_path: Path):
    t, min_d, threshold = [], [], []
    with csv_path.open("r", encoding="utf-8", errors="ignore") as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if len(row) < 3:
                continue
            time_val = float(row[0])
            min_dist = float(row[1])
            thresh = float(row[2])
            if min_dist >= 1e29:
                continue
            t.append(time_val)
            min_d.append(min_dist)
            threshold.append(thresh)
    return t, min_d, threshold


def bbox_from_obj(obj_path: Path):
    xs, ys, zs = [], [], []
    with obj_path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if line.startswith("v "):
                _, x, y, z = line.split()[:4]
                xs.append(float(x))
                ys.append(float(y))
                zs.append(float(z))
    if not xs:
        raise RuntimeError(f"No vertices in {obj_path}")
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


def extract_body_info(scene_json: Path, body_name: str):
    scene = json.loads(scene_json.read_text(encoding="utf-8"))
    for rb in scene.get("objects", {}).get("rigidbodies", []):
        if rb.get("id") == body_name:
            tr = rb.get("transform", {}).get("translation", [0.0, 0.0, 0.0])
            geom = rb.get("geometry", {})
            return {
                "translation": tuple(float(v) for v in tr),
                "obj_rel": geom.get("path", ""),
            }
    raise KeyError(f"Rigid body '{body_name}' not found in scene")


def extract_contact_threshold(scene_json: Path, body_plane: str, body_point: str, d_hat_override, eta_override):
    scene = json.loads(scene_json.read_text(encoding="utf-8"))
    gp = (
        scene.get("interactions", {})
        .get("contact", {})
        .get("global_params", {})
    )

    rigid_map = {
        rb.get("id"): rb
        for rb in scene.get("objects", {}).get("rigidbodies", [])
    }

    default_thickness = gp.get("default_contact_thickness", None)
    eta_scene = gp.get("ccd_eta", 1.0)

    def body_thickness(body_id):
        rb = rigid_map.get(body_id, {})
        contact = rb.get("contact", {})
        if "thickness" in contact:
            return float(contact["thickness"])
        if default_thickness is not None:
            return float(default_thickness)
        return None

    th_a = body_thickness(body_plane)
    th_b = body_thickness(body_point)

    if d_hat_override is not None:
        pair_dhat = float(d_hat_override)
    elif th_a is not None and th_b is not None:
        pair_dhat = th_a + th_b
    else:
        pair_dhat = 1e-5

    eta = float(eta_override) if eta_override is not None else float(eta_scene)
    return pair_dhat, eta


def generate_svg(time, gap, d_hat, out_svg: Path, title: str, threshold_series=None):
    width, height = 1200, 760
    left, right, top, bottom = 110, 40, 70, 95
    plot_w = width - left - right
    plot_h = height - top - bottom
    font = "Times New Roman, Times, serif"

    x_min, x_max = min(time), max(time)
    y_min_orig = min(min(gap), min(threshold_series))
    y_max_orig = max(max(gap), max(threshold_series))

    if threshold_series is None:
        threshold_series = [d_hat for _ in time]

    if abs(x_max - x_min) < 1e-15:
        x_max = x_min + 1.0

    if abs(y_max_orig - y_min_orig) < 1e-15:
        y_max_orig = y_min_orig + 1.0

    x_ticks = [x_min + i * (x_max - x_min) / 4 for i in range(5)]
    y_ticks = []
    y_range = y_max_orig - y_min_orig
    if y_range <= 0:
        y_range = 1.0
    
    data_magnitude = 10 ** math.floor(math.log10(y_range))
    normalized_range = y_range / data_magnitude
    
    if normalized_range <= 0.2:
        y_step = 0.05 * data_magnitude
    elif normalized_range <= 0.5:
        y_step = 0.1 * data_magnitude
    elif normalized_range <= 1.0:
        y_step = 0.2 * data_magnitude
    elif normalized_range <= 2.0:
        y_step = 0.5 * data_magnitude
    else:
        y_step = data_magnitude
    
    y_start = math.floor(y_min_orig / y_step) * y_step
    y_end = math.ceil(y_max_orig / y_step) * y_step
    v = y_start
    while v <= y_end + 1e-12:
        y_ticks.append(v)
        v += y_step
    
    if not y_ticks:
        y_ticks = [y_min_orig, y_max_orig]
    
    y_min = y_start - y_step * 0.1
    y_max = y_end + y_step * 0.1

    def mx(x):
        return left + (x - x_min) / (x_max - x_min) * plot_w

    def my(y):
        return top + (y_max - y) / (y_max - y_min) * plot_h

    curve_pts = " ".join(f"{mx(x):.2f},{my(y):.2f}" for x, y in zip(time, gap))
    dline_pts = " ".join(f"{mx(x):.2f},{my(y):.2f}" for x, y in zip(time, threshold_series))

    def fmt(v):
        a = abs(v)
        if a >= 1e3 or (a > 0 and a < 1e-3):
            return f"{v:.2e}"
        return f"{v:.3g}"

    x_tick_svg = []
    for xv in x_ticks:
        px = mx(xv)
        x_tick_svg.append(f'<line x1="{px:.2f}" y1="{top+plot_h:.2f}" x2="{px:.2f}" y2="{top+plot_h+7:.2f}" stroke="#111" stroke-width="1.4"/>')
        x_tick_svg.append(f'<text x="{px:.2f}" y="{top+plot_h+28:.2f}" text-anchor="middle" font-size="20" font-family="{font}">{fmt(xv)}</text>')

    y_tick_svg = []
    for yv in y_ticks:
        py = my(yv)
        y_tick_svg.append(f'<line x1="{left:.2f}" y1="{py:.2f}" x2="{left+plot_w:.2f}" y2="{py:.2f}" stroke="#d0d0d0" stroke-width="1.0"/>')
        y_tick_svg.append(f'<line x1="{left-7:.2f}" y1="{py:.2f}" x2="{left:.2f}" y2="{py:.2f}" stroke="#111" stroke-width="1.4"/>')
        y_tick_svg.append(f'<text x="{left-12:.2f}" y="{py+6:.2f}" text-anchor="end" font-size="20" font-family="{font}">{fmt(yv)}</text>')

    legend_x = left + plot_w - 330
    legend_y = top + 10
    svg = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect x="0" y="0" width="{width}" height="{height}" fill="white"/>
  <rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="none" stroke="#111" stroke-width="2.0"/>
  <text x="{width/2:.1f}" y="36" text-anchor="middle" font-size="24" font-family="{font}">{title}</text>
  <text x="{width/2:.1f}" y="{height-28}" text-anchor="middle" font-size="24" font-family="{font}">Time (s)</text>
  <text x="28" y="{height/2:.1f}" transform="rotate(-90,28,{height/2:.1f})" text-anchor="middle" font-size="24" font-family="{font}">Minimum Gap (m)</text>

  {' '.join(x_tick_svg)}
  {' '.join(y_tick_svg)}

    <polyline fill="none" stroke="#ff7f0e" stroke-width="2.5" stroke-dasharray="9 6" points="{dline_pts}"/>
  <polyline fill="none" stroke="#2ca02c" stroke-width="3" points="{curve_pts}"/>


  <line x1="{legend_x+14}" y1="{legend_y+20}" x2="{legend_x+48}" y2="{legend_y+20}" stroke="#2ca02c" stroke-width="3"/>
  <text x="{legend_x+56}" y="{legend_y+26}" font-size="20" font-family="{font}">minimum gap</text>
  <line x1="{legend_x+14}" y1="{legend_y+44}" x2="{legend_x+48}" y2="{legend_y+44}" stroke="#ff7f0e" stroke-width="2.5" stroke-dasharray="9 6"/>
  <text x="{legend_x+56}" y="{legend_y+50}" font-size="20" font-family="{font}">d_hat threshold</text>
</svg>
'''

    out_svg.parent.mkdir(parents=True, exist_ok=True)
    out_svg.write_text(svg, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Analyze minimum gap vs d_hat from simulation CSV (no solver changes).")
    parser.add_argument("--repo-root", default=None, help="Repo root path")
    parser.add_argument("--scene-json", required=True, help="Path to converted scene json")
    parser.add_argument("--sim-csv", required=True, help="Path to Body2_Pos.csv simulation output")
    parser.add_argument("--solver-hard-csv", default=None, help="Optional solver-exported IPC_hard_guarantee.csv for solver-consistent analysis")
    parser.add_argument("--body-plane", default="Body1", help="Body treated as contact plane body")
    parser.add_argument("--body-point", default="Body2", help="Body treated as moving body")
    parser.add_argument("--d-hat", type=float, default=None, help="Threshold d_hat override. If omitted, uses pair thickness from scene")
    parser.add_argument("--eta", type=float, default=None, help="Safety factor override. If omitted, uses scene contact global_params.ccd_eta")
    args = parser.parse_args()

    repo_root = Path(args.repo_root).resolve() if args.repo_root else Path(__file__).resolve().parents[2]
    scene_json = Path(args.scene_json).resolve()
    sim_csv = Path(args.sim_csv).resolve()

    scene_name = Path(scene_json).stem
    solver_hard_csv = Path(args.solver_hard_csv).resolve() if args.solver_hard_csv else (repo_root / "output" / scene_name / "IPC_hard_guarantee.csv")

    plane_info = extract_body_info(scene_json, args.body_plane)
    point_info = extract_body_info(scene_json, args.body_point)

    plane_obj = (scene_json.parent / plane_info["obj_rel"]).resolve()
    point_obj = (scene_json.parent / point_info["obj_rel"]).resolve()

    (_, _), (py_min_local, py_max_local), (_, _) = bbox_from_obj(plane_obj)
    (_, _), (qy_min_local, qy_max_local), (_, _) = bbox_from_obj(point_obj)

    plane_top_world = plane_info["translation"][1] + py_max_local
    radius_y = max(abs(qy_min_local), abs(qy_max_local))

    pair_dhat, eta = extract_contact_threshold(scene_json, args.body_plane, args.body_point, args.d_hat, args.eta)
    effective_threshold = eta * pair_dhat
    threshold_at_min = effective_threshold

    using_solver_csv = solver_hard_csv.exists()
    if using_solver_csv:
        t, min_gap, threshold_series = read_solver_hard_guarantee_csv(solver_hard_csv)
        threshold_at_min = threshold_series[min(range(len(min_gap)), key=lambda i: min_gap[i])] if min_gap else effective_threshold
        positive_thresholds = [v for v in threshold_series if v > 0.0]
        effective_threshold = min(positive_thresholds) if positive_thresholds else effective_threshold
    else:
        t, y_center = read_time_pos_y(sim_csv)
        min_gap = []
        for yc in y_center:
            body_bottom = yc - radius_y
            gap = body_bottom - plane_top_world
            min_gap.append(gap)

    if using_solver_csv:
        exceed = [g < th for g, th in zip(min_gap, threshold_series)]
    else:
        exceed = [g < effective_threshold for g in min_gap]
    exceed_count = sum(1 for e in exceed if e)
    min_gap_val = min(min_gap)
    min_idx = min(range(len(min_gap)), key=lambda i: min_gap[i])

    fig_dir = repo_root / "output" / "figures" / scene_name
    fig_path = fig_dir / "IPC_min_gap_vs_dhat.svg"
    generate_svg(
        t,
        min_gap,
        effective_threshold,
        fig_path,
        title=f"{Path(scene_json).stem}: minimum gap vs eta*d_hat",
        threshold_series=threshold_series if using_solver_csv else None,
    )

    report_path = fig_dir / "IPC_min_gap_report.txt"
    extra_line = f"threshold_at_min_gap={threshold_at_min:.12g}\n" if using_solver_csv else ""
    report = (
        f"scene={scene_json}\n"
        f"sim_csv={sim_csv}\n"
        f"solver_hard_csv={solver_hard_csv if using_solver_csv else 'N/A'}\n"
        f"using_solver_hard_csv={using_solver_csv}\n"
        f"plane_body={args.body_plane}, point_body={args.body_point}\n"
        f"plane_top_world_y={plane_top_world:.12g}\n"
        f"point_radius_y={radius_y:.12g}\n"
        f"pair_d_hat={pair_dhat:.12g}\n"
        f"eta={eta:.12g}\n"
        f"effective_threshold(eta*d_hat)={effective_threshold:.12g}\n"
        f"{extra_line}"
        f"min_gap={min_gap_val:.12g} at t={t[min_idx]:.12g}\n"
        f"exceed_count(gap<eta*d_hat)={exceed_count}/{len(min_gap)}\n"
    )
    report_path.write_text(report, encoding="utf-8")

    print(report.strip())
    print(f"figure={fig_path}")
    print(f"report={report_path}")


if __name__ == "__main__":
    main()
