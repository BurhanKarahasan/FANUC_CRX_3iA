#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from orientation_access_volume import OrientationAccessVolume, quaternion_multiply, values
import rclpy


COLORS = (
    (0.00, "#d9dee2"),
    (0.25, "#d47a35"),
    (0.50, "#e6bf4a"),
    (0.75, "#78a85c"),
    (1.00, "#176b52"),
)


def task_orientations(cone_degrees):
    """Downward flange approach with a two-roll family for process/tool symmetry."""
    downward = (1.0, 0.0, 0.0, 0.0)
    roll_180 = (0.0, 0.0, 1.0, 0.0)
    samples = [("down 0 deg", downward), ("down roll 180", quaternion_multiply(downward, roll_180))]
    for tilt_degrees in (cone_degrees / 2, cone_degrees):
        tilt = math.radians(tilt_degrees)
        for azimuth_degrees in range(0, 360, 90):
            azimuth = math.radians(azimuth_degrees)
            local_tilt = (
                math.sin(tilt / 2) * math.cos(azimuth),
                math.sin(tilt / 2) * math.sin(azimuth),
                0.0,
                math.cos(tilt / 2),
            )
            tilted = quaternion_multiply(downward, local_tilt)
            samples.append((f"tilt {tilt_degrees:g} azimuth {azimuth_degrees} roll 0", tilted))
            samples.append((f"tilt {tilt_degrees:g} azimuth {azimuth_degrees} roll 180", quaternion_multiply(tilted, roll_180)))
    return samples


def screen_orientations(cone_degrees):
    downward = (1.0, 0.0, 0.0, 0.0)
    roll_180 = (0.0, 0.0, 1.0, 0.0)
    samples = [("down roll 0", downward), ("down roll 180", quaternion_multiply(downward, roll_180))]
    tilt = math.radians(cone_degrees)
    for azimuth_degrees in range(0, 360, 90):
        azimuth = math.radians(azimuth_degrees)
        local_tilt = (math.sin(tilt / 2) * math.cos(azimuth), math.sin(tilt / 2) * math.sin(azimuth), 0.0, math.cos(tilt / 2))
        samples.append((f"tilt {cone_degrees:g} azimuth {azimuth_degrees}", quaternion_multiply(downward, local_tilt)))
    return samples


def color_for(capability):
    return next(color for threshold, color in COLORS if capability <= threshold)


def render_svg(rows, output, x_values, y_values, z_values, step, cone_degrees, sample_count):
    panel_width, panel_height = 380, 340
    columns = 3
    elements = []
    for index, z in enumerate(z_values):
        panel_x = 45 + (index % columns) * 425
        panel_y = 120 + (index // columns) * 390
        matching = {(float(row["x_m"]), float(row["y_m"])): float(row["capability"]) for row in rows if float(row["z_m"]) == z}
        elements.append(f'<text x="{panel_x}" y="{panel_y - 18}" class="panel-title">z = {z:.2f} m</text>')
        for row_index, y in enumerate(reversed(y_values)):
            for column_index, x in enumerate(x_values):
                capability = matching[(x, y)]
                cell_x = panel_x + column_index * (panel_width / len(x_values))
                cell_y = panel_y + row_index * (panel_height / len(y_values))
                elements.append(f'<rect x="{cell_x:.1f}" y="{cell_y:.1f}" width="{panel_width / len(x_values) + 0.5:.1f}" height="{panel_height / len(y_values) + 0.5:.1f}" fill="{color_for(capability)}"/>')
        elements.append(f'<rect x="{panel_x}" y="{panel_y}" width="{panel_width}" height="{panel_height}" class="frame"/>')
        elements.append(f'<text x="{panel_x}" y="{panel_y + panel_height + 22}" class="axis-label">x: {x_values[0]:.1f} to {x_values[-1]:.1f} m</text>')
        elements.append(f'<text x="{panel_x + panel_width - 2}" y="{panel_y + panel_height + 22}" class="axis-label" text-anchor="end">y: {y_values[0]:.1f} to {y_values[-1]:.1f} m</text>')
    svg_height = 120 + math.ceil(len(z_values) / columns) * 390 + 120
    legend = "".join(
        f'<rect x="{45 + index * 220}" y="{svg_height - 65}" width="18" height="18" fill="{color}"/><text x="{70 + index * 220}" y="{svg_height - 51}" class="note">{int(threshold * 100)}% or less</text>'
        for index, (threshold, color) in enumerate(COLORS)
    )
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="1320" height="{svg_height}" viewBox="0 0 1320 {svg_height}">
<style>
text {{ font-family: DejaVu Sans, sans-serif; fill: #1f2933; }} .title {{ font-size: 28px; font-weight: bold; }} .subtitle {{ font-size: 16px; }} .panel-title {{ font-size: 19px; font-weight: bold; }} .axis-label {{ font-size: 12px; }} .note {{ font-size: 14px; }} .frame {{ fill: none; stroke: #37474f; stroke-width: 1.5; }}
</style>
<rect width="100%" height="100%" fill="#f4f5ef"/>
<text x="45" y="45" class="title">CRX-3iA task-oriented reachability atlas</text>
<text x="45" y="75" class="subtitle">Color is the fraction of {sample_count} collision-free IK samples for a downward process approach cone of {cone_degrees:g} degrees, including 0/180 degree flange rolls.</text>
{"".join(elements)}
{legend}
<text x="45" y="{svg_height - 20}" class="note">Each cell is a live MoveIt test at {step:.2f} m spacing. Green is the production-preferred region; amber is conditional; orange/gray should be excluded or investigated.</text>
</svg>'''
    output.write_text(svg)


def main():
    parser = argparse.ArgumentParser(description="Build a collision-aware, task-oriented reachability atlas.")
    parser.add_argument("--step", type=float, default=0.10)
    parser.add_argument("--cone-degrees", type=float, default=30.0)
    parser.add_argument("--x-min", type=float, default=-0.60)
    parser.add_argument("--x-max", type=float, default=0.30)
    parser.add_argument("--y-min", type=float, default=-0.80)
    parser.add_argument("--y-max", type=float, default=0.10)
    parser.add_argument("--z-min", type=float, default=0.00)
    parser.add_argument("--z-max", type=float, default=0.60)
    parser.add_argument("--ik-timeout", type=float, default=0.10)
    parser.add_argument("--service-timeout", type=float, default=0.25)
    parser.add_argument("--profile", choices=("screen", "full"), default="full")
    parser.add_argument("--tcp-x", type=float, default=0.0)
    parser.add_argument("--tcp-y", type=float, default=0.0)
    parser.add_argument("--tcp-z", type=float, default=0.0)
    parser.add_argument("--csv", type=Path, default=Path("task_reachability_atlas.csv"))
    parser.add_argument("--svg", type=Path, default=Path("task_reachability_atlas.svg"))
    arguments = parser.parse_args()
    orientation_set = screen_orientations(arguments.cone_degrees) if arguments.profile == "screen" else task_orientations(arguments.cone_degrees)
    tcp_offset = (arguments.tcp_x, arguments.tcp_y, arguments.tcp_z)
    x_values = values(arguments.x_min, arguments.x_max, arguments.step)
    y_values = values(arguments.y_min, arguments.y_max, arguments.step)
    z_values = values(arguments.z_min, arguments.z_max, arguments.step)
    rclpy.init()
    node = OrientationAccessVolume()
    try:
        node.wait_for_services()
        node.ik_timeout_seconds = arguments.ik_timeout
        node.service_timeout_seconds = arguments.service_timeout
        rows = []
        completed_slices = set()
        if arguments.csv.exists():
            with arguments.csv.open(newline="") as csv_file:
                rows = list(csv.DictReader(csv_file))
            completed_slices = {float(row["z_m"]) for row in rows}
            print(f"Resuming with {len(completed_slices)} completed height slices")
        fieldnames = (
            "x_m", "y_m", "z_m", "orientation_samples", "safe_orientations",
            "capability", "ik_timeout_s", "failed_orientations",
        )
        for z in z_values:
            if z in completed_slices:
                continue
            slice_rows = []
            for y in y_values:
                for x in x_values:
                    successes = 0
                    failures = []
                    for label, quaternion in orientation_set:
                        safe, failure_kind = node.evaluate(x, y, z, quaternion, tcp_offset=tcp_offset)
                        if safe:
                            successes += 1
                        else:
                            failures.append(f"{label} ({failure_kind})")
                    slice_rows.append({
                        "x_m": x,
                        "y_m": y,
                        "z_m": z,
                        "orientation_samples": len(orientation_set),
                        "safe_orientations": successes,
                        "capability": f"{successes / len(orientation_set):.4f}",
                        "ik_timeout_s": arguments.ik_timeout,
                        "failed_orientations": "; ".join(failures),
                    })
            rows.extend(slice_rows)
            with arguments.csv.open("w", newline="") as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)
            print(f"Completed z = {z:.2f} m")
        render_svg(rows, arguments.svg, x_values, y_values, z_values, arguments.step, arguments.cone_degrees, len(orientation_set))
        full = sum(float(row["capability"]) == 1.0 for row in rows)
        partial = sum(0.0 < float(row["capability"]) < 1.0 for row in rows)
        print(f"{full} full-capability cells, {partial} conditional cells, {len(rows) - full - partial} excluded cells")
        print(f"Wrote {arguments.csv.resolve()} and {arguments.svg.resolve()}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()