#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path


COLORS = {
    "safe": "#167a58",
    "collision": "#c2410c",
    "ik": "#8b1e3f",
    "grid": "#a6b0a5",
    "text": "#1f2933",
    "background": "#f4f5ef",
}


def point(angle_degrees, radius, center_x, center_y, scale):
    angle = math.radians(angle_degrees)
    return (
        center_x + radius * scale * math.cos(angle),
        center_y - radius * scale * math.sin(angle),
    )


def best_samples(path):
    samples = {}
    with path.open(newline="") as csv_file:
        for row in csv.DictReader(csv_file):
            angle = int(row["angle_deg"])
            radius = float(row["radius_m"])
            state = "safe" if row["reachable"] == "True" and row["valid"] == "True" else (
                "collision" if row["reachable"] == "True" else "ik"
            )
            samples.setdefault(angle, []).append((radius, state))
    return samples


def safe_boundary(samples):
    boundary = []
    for angle, values in samples.items():
        safe_radii = [radius for radius, state in values if state == "safe"]
        if safe_radii:
            boundary.append((angle, max(safe_radii)))
    return sorted(boundary)


def draw_panel(x, y, label, samples, radius_limit, scale):
    center_x, center_y = x + 205, y + 220
    elements = [f'<text x="{x}" y="{y + 20}" class="panel-title">{label}</text>']
    for radius in (0.2, 0.4, 0.6):
        elements.append(
            f'<circle cx="{center_x}" cy="{center_y}" r="{radius * scale}" class="grid"/>'
        )
        elements.append(
            f'<text x="{center_x + 6}" y="{center_y - radius * scale - 5}" class="label">{radius:.1f} m</text>'
        )
    for angle in range(0, 360, 45):
        end_x, end_y = point(angle, radius_limit, center_x, center_y, scale)
        elements.append(f'<line x1="{center_x}" y1="{center_y}" x2="{end_x}" y2="{end_y}" class="grid"/>')
        label_x, label_y = point(angle, radius_limit + 0.055, center_x, center_y, scale)
        elements.append(f'<text x="{label_x}" y="{label_y}" class="angle">{angle}°</text>')

    boundary = safe_boundary(samples)
    if len(boundary) >= 3:
        boundary_points = [point(angle, radius, center_x, center_y, scale) for angle, radius in boundary]
        boundary_points.append(boundary_points[0])
        polygon = " ".join(f"{point_x:.1f},{point_y:.1f}" for point_x, point_y in boundary_points)
        elements.append(f'<polygon points="{polygon}" class="safe-envelope"/>')
        elements.append(f'<polyline points="{polygon}" class="safe-boundary"/>')

    for angle, values in samples.items():
        for radius, state in values:
            px, py = point(angle, radius, center_x, center_y, scale)
            elements.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="3.2" fill="{COLORS[state]}"/>')
    elements.append(f'<circle cx="{center_x}" cy="{center_y}" r="4" fill="{COLORS["text"]}"/>')
    elements.append(f'<text x="{center_x + 8}" y="{center_y + 16}" class="label">world origin</text>')
    return "\n".join(elements)


def draw_robot_inset(x, y):
    base_x = x + 215
    base_y = y + 325
    arm_points = [
        (base_x, base_y),
        (base_x, base_y - 56),
        (base_x + 16, base_y - 172),
        (base_x + 92, base_y - 250),
        (base_x + 138, base_y - 267),
    ]
    arm_path = " ".join(f"{point_x},{point_y}" for point_x, point_y in arm_points)
    elements = [
        f'<text x="{x}" y="{y + 20}" class="panel-title">CRX-3iA reference view</text>',
        f'<text x="{x}" y="{y + 45}" class="subtitle">Current mock pose: J1...J6 = 0 rad</text>',
        f'<line x1="{x + 15}" y1="{base_y}" x2="{x + 390}" y2="{base_y}" class="ground"/>',
        f'<rect x="{base_x - 42}" y="{base_y - 20}" width="84" height="20" fill="#535f66" rx="2"/>',
        f'<polyline points="{arm_path}" class="robot-arm"/>',
    ]
    for point_x, point_y in arm_points:
        elements.append(f'<circle cx="{point_x}" cy="{point_y}" r="11" class="joint"/>')
    for height, label in ((0.00, "z = 0.00 m"), (0.20, "z = 0.20 m"), (0.40, "z = 0.40 m")):
        line_y = base_y - height * 300
        elements.append(f'<line x1="{x + 20}" y1="{line_y}" x2="{x + 390}" y2="{line_y}" class="height"/>')
        elements.append(f'<text x="{x + 25}" y="{line_y - 5}" class="label">{label}</text>')
    elements.extend([
        f'<circle cx="{arm_points[-1][0]}" cy="{arm_points[-1][1]}" r="6" fill="#d06f2c"/>',
        f'<text x="{arm_points[-1][0] + 12}" y="{arm_points[-1][1] - 8}" class="label">fanuc_flange + stepper</text>',
        f'<text x="{x + 20}" y="{base_y + 25}" class="label">world XY origin / robot base</text>',
        f'<text x="{x}" y="{y + 385}" class="note">Reference only: the polar panels show actual MoveIt samples.</text>',
    ])
    return "\n".join(elements)


def main():
    parser = argparse.ArgumentParser(description="Render measured MoveIt workspace CSVs as a polar SVG map.")
    parser.add_argument("--output", type=Path, default=Path("workspace_boundary_map.svg"))
    parser.add_argument("files", nargs="*", type=Path)
    arguments = parser.parse_args()
    input_files = arguments.files or sorted(Path.cwd().glob("workspace_boundary*.csv"))
    if not input_files:
        raise SystemExit("No workspace_boundary*.csv files found")

    panels = []
    positions = [(55, 105), (510, 105), (55, 570)]
    for index, path in enumerate(input_files[:3]):
        label = path.stem.replace("workspace_boundary_", "z = ").replace("results", "z = 0.20 m")
        if label.startswith("z = z"):
            label = label.replace("z = z", "z = ").replace(".csv", "") + " m"
        panels.append(draw_panel(*positions[index], label, best_samples(path), 0.65, 300))

    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="965" height="1080" viewBox="0 0 965 1080">
<style>
  text {{ font-family: DejaVu Sans, sans-serif; fill: {COLORS["text"]}; }}
  .title {{ font-size: 28px; font-weight: bold; }} .subtitle {{ font-size: 15px; }}
  .panel-title {{ font-size: 20px; font-weight: bold; }} .label {{ font-size: 12px; }}
  .angle {{ font-size: 13px; text-anchor: middle; dominant-baseline: middle; }}
  .grid {{ fill: none; stroke: {COLORS["grid"]}; stroke-width: 1; stroke-dasharray: 3 4; }}
    .safe-envelope {{ fill: {COLORS["safe"]}; fill-opacity: 0.16; stroke: none; }}
    .safe-boundary {{ fill: none; stroke: {COLORS["safe"]}; stroke-width: 2.5; stroke-linejoin: round; }}
    .ground {{ stroke: #535f66; stroke-width: 3; }} .robot-arm {{ fill: none; stroke: #2d6f86; stroke-width: 18; stroke-linecap: round; stroke-linejoin: round; }}
    .joint {{ fill: #f0ab42; stroke: #1f2933; stroke-width: 2; }} .height {{ stroke: #80919c; stroke-width: 1; stroke-dasharray: 5 4; }} .note {{ font-size: 12px; font-style: italic; }}
</style>
<rect width="100%" height="100%" fill="{COLORS["background"]}"/>
<text x="55" y="45" class="title">CRX-3iA collision-safe workspace map</text>
<text x="55" y="72" class="subtitle">Flange target positions in world XY. Filled green: interpolated safe envelope; dots: measured MoveIt results.</text>
<circle cx="70" cy="1015" r="6" fill="{COLORS["safe"]}"/><text x="84" y="1020" class="subtitle">Collision-safe</text>
<circle cx="280" cy="1015" r="6" fill="{COLORS["collision"]}"/><text x="294" y="1020" class="subtitle">Collision-limited</text>
<circle cx="515" cy="1015" r="6" fill="{COLORS["ik"]}"/><text x="529" y="1020" class="subtitle">IK-unreachable</text>
<text x="55" y="1055" class="subtitle">Envelope connects 15° measured spokes; boundary transitions are refined to about 3 mm. Verify untested points before commanding motion.</text>
{"\n".join(panels)}
{draw_robot_inset(510, 570)}
</svg>'''
    arguments.output.write_text(svg)
    print(f"Wrote {arguments.output.resolve()}")


if __name__ == "__main__":
    main()