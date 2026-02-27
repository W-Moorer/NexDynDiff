import argparse
import csv
import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import xml.etree.ElementTree as ET


def _to_float(text: str) -> float:
    token = text.strip().replace("D", "E").replace("d", "e")
    if token.endswith("E") or token.endswith("e"):
        token = token + "0"
    if token in {"+", "-", ""}:
        raise ValueError(f"Invalid numeric token: '{text}'")
    return float(token)


def _is_entity_start(line: str) -> bool:
    return (not line.startswith(",")) and bool(re.match(r"^[A-Z_]+\s*/", line.strip()))


def _split_csv_payload(line: str) -> List[str]:
    payload = line.split("!")[0].strip()
    if not payload.startswith(","):
        return []
    return [p.strip() for p in payload[1:].split(",")]


def _parse_key_value(line: str, key: str) -> Optional[str]:
    m = re.search(rf"{re.escape(key)}\s*=\s*(.+)", line)
    if not m:
        return None
    return m.group(1).strip()


def _vec3_to_str(v: Tuple[float, float, float]) -> str:
    return ",".join(f"{x:.17g}" for x in v)


def _read_last_time_from_csv(csv_path: Path) -> Optional[float]:
    if not csv_path.exists():
        return None
    last_t: Optional[float] = None
    with csv_path.open("r", encoding="utf-8", errors="ignore") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            cell = row[0].strip()
            if not cell:
                continue
            try:
                last_t = _to_float(cell)
            except ValueError:
                continue
    return last_t


def infer_end_time(rmd_path: Path) -> float:
    candidates = sorted(rmd_path.parent.glob("*_Pos.csv"))
    best: Optional[float] = None
    for c in candidates:
        t = _read_last_time_from_csv(c)
        if t is None:
            continue
        if best is None or t > best:
            best = t
    if best is not None and best > 0.0:
        return best
    return 0.3


@dataclass
class Part:
    part_id: int
    name: str = ""
    mass: float = 1.0
    ip: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    qg: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    reuler: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class Marker:
    marker_id: int
    name: str = ""
    part_id: int = -1
    qp: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    reuler: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class JointFixed:
    i_marker: int
    j_marker: int


@dataclass
class RmdSimulationInfo:
    hmax: Optional[float] = None
    n_threads: Optional[int] = None


@dataclass
class SolidContact:
    icsurface_id: int
    jcsurface_id: int
    bp_pen: Optional[float] = None
    stiffness: Optional[float] = None
    friction: Optional[float] = None


@dataclass
class PatchSurface:
    csurface_id: int
    name: str = ""
    rm_marker: Optional[int] = None
    no_patch: int = 0
    no_node: int = 0
    patchtype: str = ""
    patches_raw: List[List[float]] = field(default_factory=list)
    nodes: List[Tuple[float, float, float]] = field(default_factory=list)


def parse_rmd(rmd_path: Path):
    parts: Dict[int, Part] = {}
    markers: Dict[int, Marker] = {}
    fixed_joints: List[JointFixed] = []
    contacts: List[SolidContact] = []
    gravity = (0.0, -9.80665, 0.0)
    sim_info = RmdSimulationInfo()

    surfaces: Dict[int, PatchSurface] = {}

    current_entity = None
    current_id = None

    in_csurface = False
    csurface_id = None
    reading_patch = False
    reading_node = False

    joint_is_fixed = False
    joint_i: Optional[int] = None
    joint_j: Optional[int] = None

    def flush_joint_if_needed():
        nonlocal joint_is_fixed, joint_i, joint_j
        if joint_is_fixed and joint_i is not None and joint_j is not None:
            fixed_joints.append(JointFixed(i_marker=joint_i, j_marker=joint_j))
        joint_is_fixed = False
        joint_i = None
        joint_j = None

    with rmd_path.open("r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = raw.lstrip("\ufeff")
            stripped = line.strip()

            if _is_entity_start(line):
                if current_entity == "JOINT":
                    flush_joint_if_needed()
                reading_patch = False
                reading_node = False

                m = re.match(r"^([A-Z_]+)\s*/\s*(.*)$", stripped)
                current_entity = m.group(1) if m else None
                tail = m.group(2) if m else ""

                in_csurface = current_entity == "CSURFACE"
                if in_csurface:
                    m2 = re.match(r"([0-9]+)", tail.strip())
                    if m2:
                        csurface_id = int(m2.group(1))
                        surfaces[csurface_id] = PatchSurface(csurface_id=csurface_id)
                else:
                    csurface_id = None

                if current_entity in {"PART", "MARKER", "JOINT", "SOLIDCONTACT"}:
                    m2 = re.match(r"([0-9]+)", tail.strip())
                    current_id = int(m2.group(1)) if m2 else None
                    if current_entity == "PART" and current_id is not None:
                        parts[current_id] = Part(part_id=current_id)
                    elif current_entity == "MARKER" and current_id is not None:
                        markers[current_id] = Marker(marker_id=current_id)
                    elif current_entity == "SOLIDCONTACT" and current_id is not None:
                        contacts.append(SolidContact(icsurface_id=-1, jcsurface_id=-1))
                    elif current_entity == "JOINT":
                        joint_is_fixed = False
                        joint_i = None
                        joint_j = None
                else:
                    current_id = None
                continue

            if stripped.startswith("accgrav"):
                current_entity = "ACCGRAV"
                continue

            if in_csurface and csurface_id is not None:
                surf = surfaces[csurface_id]

                if ", PATCH" in line and "=" in line:
                    reading_patch = True
                    reading_node = False
                    continue
                if ", NODE" in line and "=" in line:
                    reading_node = True
                    reading_patch = False
                    continue

                if reading_patch:
                    vals = _split_csv_payload(line)
                    if vals:
                        row = [_to_float(v) for v in vals]
                        surf.patches_raw.append(row)
                    continue

                if reading_node:
                    vals = _split_csv_payload(line)
                    if len(vals) >= 3:
                        surf.nodes.append((_to_float(vals[0]), _to_float(vals[1]), _to_float(vals[2])))
                    continue

                if "NAME" in line:
                    m = re.search(r"NAME\s*=\s*'([^']+)'", line)
                    if m:
                        surf.name = m.group(1)
                if "RM" in line:
                    v = _parse_key_value(line, "RM")
                    if v:
                        surf.rm_marker = int(v.split(",")[0].strip())
                if "NO_PATCH" in line and "NO_NODE" in line:
                    m = re.search(r"NO_PATCH\s*=\s*([0-9]+).*NO_NODE\s*=\s*([0-9]+)", line)
                    if m:
                        surf.no_patch = int(m.group(1))
                        surf.no_node = int(m.group(2))
                if "PATCHTYPE" in line:
                    v = _parse_key_value(line, "PATCHTYPE")
                    if v:
                        surf.patchtype = v.split(",")[0].strip()
                continue

            if current_entity == "PART" and current_id in parts:
                part = parts[current_id]
                if "NAME" in line:
                    m = re.search(r"NAME\s*=\s*'([^']+)'", line)
                    if m:
                        part.name = m.group(1)
                elif "MASS" in line:
                    v = _parse_key_value(line, "MASS")
                    if v:
                        part.mass = _to_float(v.split(",")[0])
                elif "IP" in line:
                    vals = _split_csv_payload(line)
                    if len(vals) >= 3 and vals[0].strip().startswith("IP") is False:
                        pass
                    if "IP" in line:
                        m = re.search(r"IP\s*=\s*([^,]+),\s*([^,]+),\s*([^,\n]+)", line)
                        if m:
                            part.ip = (_to_float(m.group(1)), _to_float(m.group(2)), _to_float(m.group(3)))
                elif "QG" in line:
                    m = re.search(r"QG\s*=\s*([^,]+),\s*([^,]+),\s*([^,\n]+)", line)
                    if m:
                        part.qg = (_to_float(m.group(1)), _to_float(m.group(2)), _to_float(m.group(3)))
                elif "REULER" in line:
                    m = re.search(r"REULER\s*=\s*([^,]+),\s*([^,]+),\s*([^,\n]+)", line)
                    if m:
                        part.reuler = (_to_float(m.group(1)), _to_float(m.group(2)), _to_float(m.group(3)))

            elif current_entity == "MARKER" and current_id in markers:
                marker = markers[current_id]
                if "NAME" in line:
                    m = re.search(r"NAME\s*=\s*'([^']+)'", line)
                    if m:
                        marker.name = m.group(1)
                elif "PART" in line:
                    v = _parse_key_value(line, "PART")
                    if v:
                        marker.part_id = int(v.split(",")[0].strip())
                elif "QP" in line:
                    m = re.search(r"QP\s*=\s*([^,]+),\s*([^,]+),\s*([^,\n]+)", line)
                    if m:
                        marker.qp = (_to_float(m.group(1)), _to_float(m.group(2)), _to_float(m.group(3)))
                elif "REULER" in line:
                    m = re.search(r"REULER\s*=\s*([^,]+),\s*([^,]+),\s*([^,\n]+)", line)
                    if m:
                        marker.reuler = (_to_float(m.group(1)), _to_float(m.group(2)), _to_float(m.group(3)))

            elif current_entity == "JOINT" and fixed_joints is not None:
                if stripped.startswith(",") and "=" not in stripped:
                    joint_type = stripped[1:].strip().lower()
                    if joint_type.startswith("fixed"):
                        joint_is_fixed = True
                if "I" in line and "=" in line and re.search(r"\bI\s*=", line):
                    v = _parse_key_value(line, "I")
                    if v:
                        joint_i = int(v.split(",")[0].strip())
                if "J" in line and "=" in line and re.search(r"\bJ\s*=", line):
                    v = _parse_key_value(line, "J")
                    if v:
                        joint_j = int(v.split(",")[0].strip())

            elif current_entity == "SOLIDCONTACT" and contacts:
                c = contacts[-1]
                if "ICSURFACEID" in line:
                    v = _parse_key_value(line, "ICSURFACEID")
                    if v:
                        c.icsurface_id = int(v.split(",")[0].strip())
                elif "JCSURFACEID" in line:
                    v = _parse_key_value(line, "JCSURFACEID")
                    if v:
                        c.jcsurface_id = int(v.split(",")[0].strip())
                elif re.search(r"\bBPEN\s*=", line):
                    v = _parse_key_value(line, "BPEN")
                    if v:
                        c.bp_pen = _to_float(v.split(",")[0])
                elif re.search(r"\bK\s*=", line):
                    v = _parse_key_value(line, "K")
                    if v:
                        c.stiffness = _to_float(v.split(",")[0])
                elif re.search(r"\bS_F_C\s*=", line):
                    v = _parse_key_value(line, "S_F_C")
                    if v:
                        c.friction = _to_float(v.split(",")[0])

            elif current_entity == "ACCGRAV":
                if "IGRAV" in line:
                    v = _parse_key_value(line, "IGRAV")
                    if v:
                        gravity = (_to_float(v.split(",")[0]), gravity[1], gravity[2])
                elif "JGRAV" in line:
                    v = _parse_key_value(line, "JGRAV")
                    if v:
                        gravity = (gravity[0], _to_float(v.split(",")[0]), gravity[2])
                elif "KGRAV" in line:
                    v = _parse_key_value(line, "KGRAV")
                    if v:
                        gravity = (gravity[0], gravity[1], _to_float(v.split(",")[0]))

            elif current_entity == "INTPAR":
                if re.search(r"\bHMAX\s*=", line):
                    v = _parse_key_value(line, "HMAX")
                    if v:
                        sim_info.hmax = _to_float(v.split(",")[0])

            elif current_entity == "SOLVEROPTION":
                if re.search(r"\bNUM_THREAD\s*=", line):
                    v = _parse_key_value(line, "NUM_THREAD")
                    if v:
                        token = v.split(",")[0].strip().lower()
                        if token not in {"auto", ""}:
                            try:
                                sim_info.n_threads = int(token)
                            except ValueError:
                                pass

    if current_entity == "JOINT":
        flush_joint_if_needed()

    return parts, markers, surfaces, fixed_joints, contacts, gravity, sim_info


def _rotate_xyz_deg(v: Tuple[float, float, float], rdeg: Tuple[float, float, float]) -> Tuple[float, float, float]:
    x, y, z = v
    rx, ry, rz = math.radians(rdeg[0]), math.radians(rdeg[1]), math.radians(rdeg[2])

    cy, sy = math.cos(rx), math.sin(rx)
    y, z = y * cy - z * sy, y * sy + z * cy

    cy, sy = math.cos(ry), math.sin(ry)
    x, z = x * cy + z * sy, -x * sy + z * cy

    cy, sy = math.cos(rz), math.sin(rz)
    x, y = x * cy - y * sy, x * sy + y * cy

    return x, y, z


def _inverse_rotate_xyz_deg(v: Tuple[float, float, float], rdeg: Tuple[float, float, float]) -> Tuple[float, float, float]:
    x, y, z = v
    rx, ry, rz = math.radians(rdeg[0]), math.radians(rdeg[1]), math.radians(rdeg[2])

    cz, sz = math.cos(-rz), math.sin(-rz)
    x, y = x * cz - y * sz, x * sz + y * cz

    cy, sy = math.cos(-ry), math.sin(-ry)
    x, z = x * cy + z * sy, -x * sy + z * cy

    cx, sx = math.cos(-rx), math.sin(-rx)
    y, z = y * cx - z * sx, y * sx + z * cx

    return x, y, z


def _vadd(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vsub(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def build_normals(surface: PatchSurface):
    nodes = surface.nodes
    patches = surface.patches_raw
    raw_tris = [[int(p[0]) - 1, int(p[1]) - 1, int(p[2]) - 1] for p in patches]

    tris = []
    filtered_patches = []
    if nodes:
        xs = [v[0] for v in nodes]
        ys = [v[1] for v in nodes]
        zs = [v[2] for v in nodes]
        bbox_diag = math.sqrt((max(xs) - min(xs)) ** 2 + (max(ys) - min(ys)) ** 2 + (max(zs) - min(zs)) ** 2)
    else:
        bbox_diag = 1.0
    eps_area2 = max(1e-18, (bbox_diag ** 2) * 1e-16)

    for tri, p in zip(raw_tris, patches):
        i, j, k = tri
        if i < 0 or j < 0 or k < 0 or i >= len(nodes) or j >= len(nodes) or k >= len(nodes):
            continue
        if i == j or j == k or i == k:
            continue
        p0, p1, p2 = nodes[i], nodes[j], nodes[k]
        ux, uy, uz = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]
        vx, vy, vz = p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        area2 = math.sqrt(nx * nx + ny * ny + nz * nz)
        if not math.isfinite(area2) or area2 <= eps_area2:
            continue
        tris.append(tri)
        filtered_patches.append(p)

    patches = filtered_patches

    face_normals = []
    if patches and len(patches[0]) >= 6:
        for p in patches:
            nx, ny, nz = p[3], p[4], p[5]
            n = math.sqrt(nx * nx + ny * ny + nz * nz)
            face_normals.append((nx / n if n > 1e-12 else 0.0, ny / n if n > 1e-12 else 0.0, nz / n if n > 1e-12 else 1.0))
    else:
        for tri in tris:
            p0, p1, p2 = nodes[tri[0]], nodes[tri[1]], nodes[tri[2]]
            ux, uy, uz = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]
            vx, vy, vz = p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]
            nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
            n = math.sqrt(nx * nx + ny * ny + nz * nz)
            face_normals.append((nx / n if n > 1e-12 else 0.0, ny / n if n > 1e-12 else 0.0, nz / n if n > 1e-12 else 1.0))

    vertex_normals = [(0.0, 0.0, 0.0) for _ in nodes]
    counts = [0 for _ in nodes]

    if patches and len(patches[0]) >= 15 and surface.patchtype == "EACHNODENORMAL":
        acc = [[0.0, 0.0, 0.0] for _ in nodes]
        for p in patches:
            ids = [int(p[0]) - 1, int(p[1]) - 1, int(p[2]) - 1]
            ns = [p[6:9], p[9:12], p[12:15]]
            for idx, nvec in zip(ids, ns):
                acc[idx][0] += nvec[0]
                acc[idx][1] += nvec[1]
                acc[idx][2] += nvec[2]
                counts[idx] += 1
        out = []
        for i, a in enumerate(acc):
            nx, ny, nz = a
            n = math.sqrt(nx * nx + ny * ny + nz * nz)
            if n > 1e-12:
                out.append((nx / n, ny / n, nz / n))
            else:
                out.append((0.0, 0.0, 1.0))
        vertex_normals = out
    else:
        acc = [[0.0, 0.0, 0.0] for _ in nodes]
        for tri, fn in zip(tris, face_normals):
            for idx in tri:
                acc[idx][0] += fn[0]
                acc[idx][1] += fn[1]
                acc[idx][2] += fn[2]
                counts[idx] += 1
        out = []
        for i, a in enumerate(acc):
            nx, ny, nz = a
            n = math.sqrt(nx * nx + ny * ny + nz * nz)
            if n > 1e-12:
                out.append((nx / n, ny / n, nz / n))
            else:
                out.append((0.0, 0.0, 1.0))
        vertex_normals = out

    return tris, face_normals, vertex_normals


def write_obj_with_normals(
    surface: PatchSurface,
    out_obj: Path,
    out_meta: Path,
    rm_translation: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    rm_rotation_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    body_translation: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    body_rotation_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    shifted = PatchSurface(
        csurface_id=surface.csurface_id,
        name=surface.name,
        rm_marker=surface.rm_marker,
        no_patch=surface.no_patch,
        no_node=surface.no_node,
        patchtype=surface.patchtype,
        patches_raw=surface.patches_raw,
        nodes=[
            _inverse_rotate_xyz_deg(
                _vsub(
                    _vadd(_rotate_xyz_deg(v, rm_rotation_deg), rm_translation),
                    body_translation,
                ),
                body_rotation_deg,
            )
            for v in surface.nodes
        ],
    )

    tris, face_normals, vertex_normals = build_normals(shifted)

    out_obj.parent.mkdir(parents=True, exist_ok=True)
    with out_obj.open("w", encoding="utf-8") as f:
        f.write(f"# Generated from {surface.name}\n")
        for v in shifted.nodes:
            f.write(f"v {v[0]:.17g} {v[1]:.17g} {v[2]:.17g}\n")
        for vn in vertex_normals:
            f.write(f"vn {vn[0]:.17g} {vn[1]:.17g} {vn[2]:.17g}\n")
        for tri in tris:
            i, j, k = tri[0] + 1, tri[1] + 1, tri[2] + 1
            f.write(f"f {i}//{i} {j}//{j} {k}//{k}\n")

    meta = {
        "name": surface.name,
        "patchtype": surface.patchtype,
        "vertex_count": len(shifted.nodes),
        "triangle_count": len(tris),
        "rm_translation": [rm_translation[0], rm_translation[1], rm_translation[2]],
        "rm_rotation_deg": [rm_rotation_deg[0], rm_rotation_deg[1], rm_rotation_deg[2]],
        "body_translation": [body_translation[0], body_translation[1], body_translation[2]],
        "body_rotation_deg": [body_rotation_deg[0], body_rotation_deg[1], body_rotation_deg[2]],
        "triangles": tris,
        "face_normals": face_normals,
        "vertex_normals": vertex_normals,
    }
    with out_meta.open("w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False)


def _surface_body_name(surface_name: str) -> Optional[str]:
    m = re.search(r"##SOLID##_([^\.]+)\.", surface_name)
    if m:
        return m.group(1)
    return None


def _marker_by_name(markers: Dict[int, Marker], name: str) -> Optional[Marker]:
    for marker in markers.values():
        if marker.name == name:
            return marker
    return None


def build_scene_json(
    rmd_path: Path,
    parts: Dict[int, Part],
    markers: Dict[int, Marker],
    surfaces: Dict[int, PatchSurface],
    fixed_joints: List[JointFixed],
    contacts: List[SolidContact],
    gravity: Tuple[float, float, float],
    sim_info: RmdSimulationInfo,
    end_simulation_time: float,
    obj_root: Path,
    scene_path: Path,
):
    stem = rmd_path.stem

    csurface_to_body: Dict[int, str] = {}
    body_to_surface: Dict[str, int] = {}
    for sid, surf in surfaces.items():
        body = _surface_body_name(surf.name)
        if body:
            csurface_to_body[sid] = body
            if body not in body_to_surface:
                body_to_surface[body] = sid

    rigidbodies = []
    parts_by_name = {p.name: p for p in parts.values()}
    for part in parts.values():
        if part.name.upper() == "GROUND":
            continue
        if part.name not in body_to_surface:
            continue

        cm_marker = _marker_by_name(markers, f"{part.name}.CM")
        translation = cm_marker.qp if cm_marker else part.qg
        rotation = cm_marker.reuler if cm_marker else part.reuler

        sid = body_to_surface[part.name]
        obj_file = obj_root / f"{part.name}.obj"
        rel_obj = Path(
            str(obj_file.relative_to(scene_path.parent))
        ) if obj_file.is_relative_to(scene_path.parent) else Path(__import__("os").path.relpath(obj_file, scene_path.parent))

        rigidbodies.append(
            {
                "id": part.name,
                "label": part.name,
                "mass": part.mass,
                "inertia_diagonal": [part.ip[0], part.ip[1], part.ip[2]],
                "geometry": {
                    "type": "file",
                    "path": rel_obj.as_posix(),
                },
                "transform": {
                    "translation": [translation[0], translation[1], translation[2]],
                    "rotation": [rotation[0], rotation[1], rotation[2]],
                },
            }
        )

    boundary_conditions = []
    for joint in fixed_joints:
        mi = markers.get(joint.i_marker)
        mj = markers.get(joint.j_marker)
        if mi is None or mj is None:
            continue
        if mi.part_id in parts and parts[mi.part_id].name.upper() == "GROUND" and mj.part_id in parts:
            target = parts[mj.part_id].name
        elif mj.part_id in parts and parts[mj.part_id].name.upper() == "GROUND" and mi.part_id in parts:
            target = parts[mi.part_id].name
        else:
            continue
        boundary_conditions.append(
            {
                "type": "rigidbody_constraint",
                "constraint_type": "fix",
                "target": target,
            }
        )

    default_contact_thickness = None
    min_contact_stiffness = None
    contact_pairs = []
    for c in contacts:
        if c.bp_pen is not None and default_contact_thickness is None:
            default_contact_thickness = c.bp_pen
        if c.stiffness is not None and min_contact_stiffness is None:
            min_contact_stiffness = c.stiffness
        b0 = csurface_to_body.get(c.icsurface_id)
        b1 = csurface_to_body.get(c.jcsurface_id)
        if b0 and b1:
            pair = {"object1": b0, "object2": b1, "enabled": True}
            if c.friction is not None:
                pair["friction"] = c.friction
            contact_pairs.append(pair)

    scene = {
        "version": "1.0",
        "metadata": {
            "name": stem,
            "source_rmd": str(rmd_path.as_posix()),
        },
        "settings": {
            "output": {
                "simulation_name": stem,
                "output_directory": f"./output/{stem}",
                "codegen_directory": "./codegen/rmd_generated",
                "enable_output": True,
                "fps": 1000,
            },
            "simulation": {
                "gravity": [gravity[0], gravity[1], gravity[2]],
                "init_frictional_contact": True,
                **({"max_time_step_size": sim_info.hmax} if sim_info.hmax is not None else {}),
            },
            "execution": {
                "end_simulation_time": float(end_simulation_time),
                **({"n_threads": sim_info.n_threads} if sim_info.n_threads is not None else {}),
            },
        },
        "objects": {
            "rigidbodies": rigidbodies,
        },
        "boundary_conditions": boundary_conditions,
    }

    if default_contact_thickness is not None or min_contact_stiffness is not None or contact_pairs:
        scene["interactions"] = {"contact": {}}
        scene["interactions"]["contact"]["global_params"] = {}
        if default_contact_thickness is not None:
            scene["interactions"]["contact"]["global_params"]["default_contact_thickness"] = default_contact_thickness
        if min_contact_stiffness is not None:
            scene["interactions"]["contact"]["global_params"]["min_contact_stiffness"] = min_contact_stiffness
        if contact_pairs:
            scene["interactions"]["contact"]["pairs"] = contact_pairs

    return scene


def write_scene_xml(scene: dict, xml_path: Path):
    root = ET.Element("scene", {"version": str(scene.get("version", "1.0"))})

    settings = ET.SubElement(root, "settings")
    out = scene.get("settings", {}).get("output", {})
    ET.SubElement(settings, "output", {
        "simulation_name": str(out.get("simulation_name", "scene")),
        "output_directory": str(out.get("output_directory", "./output")),
        "codegen_directory": str(out.get("codegen_directory", "./codegen")),
        "enable_output": str(out.get("enable_output", False)).lower(),
        "fps": str(out.get("fps", 60)),
    })

    sim = scene.get("settings", {}).get("simulation", {})
    sim_attrs = {
        "init_frictional_contact": str(sim.get("init_frictional_contact", False)).lower(),
    }
    if "gravity" in sim:
        sim_attrs["gravity"] = _vec3_to_str(tuple(sim["gravity"]))
    ET.SubElement(settings, "simulation", sim_attrs)

    exe = scene.get("settings", {}).get("execution", {})
    ET.SubElement(settings, "execution", {"end_simulation_time": str(exe.get("end_simulation_time", 1.0))})

    objects = ET.SubElement(root, "objects")
    rbs = ET.SubElement(objects, "rigidbodies")
    for rb in scene.get("objects", {}).get("rigidbodies", []):
        rb_node = ET.SubElement(rbs, "rigidbody", {
            "id": str(rb["id"]),
            "label": str(rb.get("label", rb["id"])),
            "mass": str(rb.get("mass", 1.0)),
            "inertia_diagonal": _vec3_to_str(tuple(rb.get("inertia_diagonal", [1.0, 1.0, 1.0]))),
        })
        geom = rb.get("geometry", {})
        ET.SubElement(rb_node, "geometry", {
            "type": str(geom.get("type", "file")),
            "path": str(geom.get("path", "")),
        })
        tr = rb.get("transform", {})
        ET.SubElement(rb_node, "transform", {
            "translation": _vec3_to_str(tuple(tr.get("translation", [0.0, 0.0, 0.0]))),
            "rotation": _vec3_to_str(tuple(tr.get("rotation", [0.0, 0.0, 0.0]))),
        })

    bcs = ET.SubElement(root, "boundary_conditions")
    for bc in scene.get("boundary_conditions", []):
        ET.SubElement(bcs, "rigidbody_constraint", {
            "type": str(bc.get("constraint_type", "fix")),
            "target": str(bc.get("target", "")),
        })

    interactions = scene.get("interactions", {})
    if interactions:
        interactions_node = ET.SubElement(root, "interactions")
        contact = interactions.get("contact", {})
        contact_node = ET.SubElement(interactions_node, "contact")
        gp = contact.get("global_params", {})
        if gp:
            gp_attrs = {}
            if "default_contact_thickness" in gp:
                gp_attrs["default_contact_thickness"] = str(gp["default_contact_thickness"])
            if "min_contact_stiffness" in gp:
                gp_attrs["min_contact_stiffness"] = str(gp["min_contact_stiffness"])
            ET.SubElement(contact_node, "global_params", gp_attrs)
        for pair in contact.get("pairs", []):
            attrs = {
                "object1": str(pair["object1"]),
                "object2": str(pair["object2"]),
                "enabled": str(pair.get("enabled", True)).lower(),
            }
            if "friction" in pair:
                attrs["friction"] = str(pair["friction"])
            ET.SubElement(contact_node, "friction_pair", attrs)

    ET.indent(root, space="  ")
    xml_path.write_text('<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(root, encoding="unicode") + '\n', encoding="utf-8")


def convert(
    rmd_path: Path,
    out_dir: Path,
    scene_format: str,
    apply_rm_offset: bool = False,
    end_time: Optional[float] = None,
):
    parts, markers, surfaces, fixed_joints, contacts, gravity, sim_info = parse_rmd(rmd_path)

    mesh_out_dir = out_dir / "models" / "obj" / rmd_path.stem
    scene_out_dir = rmd_path.parent

    parts_by_name = {p.name: p for p in parts.values()}
    for sid, surf in surfaces.items():
        body_name = _surface_body_name(surf.name)
        if not body_name:
            continue
        rm_translation = (0.0, 0.0, 0.0)
        rm_rotation_deg = (0.0, 0.0, 0.0)
        body_translation = (0.0, 0.0, 0.0)
        body_rotation_deg = (0.0, 0.0, 0.0)

        cm_marker = _marker_by_name(markers, f"{body_name}.CM")
        if cm_marker is not None:
            body_translation = cm_marker.qp
            body_rotation_deg = cm_marker.reuler
        elif body_name in parts_by_name:
            body_translation = parts_by_name[body_name].qg
            body_rotation_deg = parts_by_name[body_name].reuler

        if apply_rm_offset and surf.rm_marker is not None and surf.rm_marker in markers:
            rm_translation = markers[surf.rm_marker].qp
            rm_rotation_deg = markers[surf.rm_marker].reuler
        obj_path = mesh_out_dir / f"{body_name}.obj"
        meta_path = mesh_out_dir / f"{body_name}_normals.json"
        write_obj_with_normals(
            surf,
            obj_path,
            meta_path,
            rm_translation=rm_translation,
            rm_rotation_deg=rm_rotation_deg,
            body_translation=body_translation,
            body_rotation_deg=body_rotation_deg,
        )

    scene_json_path = scene_out_dir / f"{rmd_path.stem}.json"
    scene_xml_path = scene_out_dir / f"{rmd_path.stem}.xml"

    scene = build_scene_json(
        rmd_path=rmd_path,
        parts=parts,
        markers=markers,
        surfaces=surfaces,
        fixed_joints=fixed_joints,
        contacts=contacts,
        gravity=gravity,
        sim_info=sim_info,
        end_simulation_time=(float(end_time) if end_time is not None else infer_end_time(rmd_path)),
        obj_root=mesh_out_dir,
        scene_path=scene_json_path,
    )

    scene_json_path.write_text(json.dumps(scene, indent=2, ensure_ascii=False), encoding="utf-8")
    if scene_format in {"xml", "both"}:
        write_scene_xml(scene, scene_xml_path)

    manifest = {
        "rmd": str(rmd_path.as_posix()),
        "json": str(scene_json_path.as_posix()),
        "xml": str(scene_xml_path.as_posix()) if scene_format in {"xml", "both"} else None,
        "mesh_dir": str(mesh_out_dir.as_posix()),
        "parts": [p.name for p in parts.values()],
    }
    manifest_path = scene_out_dir / f"{rmd_path.stem}_convert_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")

    return scene_json_path, scene_xml_path if scene_format in {"xml", "both"} else None, mesh_out_dir


def main():
    parser = argparse.ArgumentParser(description="Convert RecurDyn .rmd to NexDynDiff scene and OBJ meshes.")
    parser.add_argument("rmd", type=str, help="Path to .rmd file")
    parser.add_argument("--repo-root", type=str, default=None, help="Repo root containing models/ directory (default: auto)")
    parser.add_argument("--format", choices=["json", "xml", "both"], default="both", help="Scene output format")
    parser.add_argument("--apply-rm-offset", action="store_true", help="Apply CSURFACE RM marker translation to exported OBJ vertices")
    parser.add_argument("--end-time", type=float, default=None, help="Override end_simulation_time in output scene")
    args = parser.parse_args()

    rmd_path = Path(args.rmd).resolve()
    if not rmd_path.exists():
        raise FileNotFoundError(rmd_path)

    repo_root = Path(args.repo_root).resolve() if args.repo_root else rmd_path.parents[2]
    json_path, xml_path, mesh_dir = convert(
        rmd_path,
        repo_root,
        args.format,
        apply_rm_offset=args.apply_rm_offset,
        end_time=args.end_time,
    )

    print(f"Converted: {rmd_path}")
    print(f"Scene JSON: {json_path}")
    if xml_path is not None:
        print(f"Scene XML:  {xml_path}")
    print(f"Meshes:     {mesh_dir}")


if __name__ == "__main__":
    main()
