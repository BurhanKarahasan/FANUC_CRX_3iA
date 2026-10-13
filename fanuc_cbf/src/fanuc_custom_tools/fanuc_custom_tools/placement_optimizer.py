#!/usr/bin/env python3

"""Score every candidate workspace placement against a precomputed dexterity field.

The workspace box is a region of interest rather than an obstacle, so the dexterity
field belongs to the robot alone and does not move when the box moves. That means the
placement study needs one IK sweep, not one per candidate: map the robot's envelope
once, then slide the box through the field and integrate.

A 3D summed-area table makes each placement O(1) regardless of box size, so an
exhaustive search over thousands of placements takes seconds instead of days of IK.
"""

import argparse
import csv
import struct
from pathlib import Path

USABLE = 0.5  # dexterity at or above this counts as practically orientable


def read_binary_stl_bounds(path, scale):
    with open(path, "rb") as mesh_file:
        mesh_file.read(80)
        count = struct.unpack("<I", mesh_file.read(4))[0]
        lo = [float("inf")] * 3
        hi = [float("-inf")] * 3
        for _ in range(count):
            values = struct.unpack("<12fH", mesh_file.read(50))
            for offset in (3, 6, 9):
                for axis in range(3):
                    v = values[offset + axis] * scale
                    lo[axis] = min(lo[axis], v)
                    hi[axis] = max(hi[axis], v)
        return lo, hi


class Field:
    """Dexterity samples on a regular grid, with O(1) box sums."""

    def __init__(self, path, step):
        self.step = step
        samples = {}
        for row in csv.DictReader(open(path)):
            key = (round(float(row["x"]) / step), round(float(row["y"]) / step),
                   round(float(row["z"]) / step))
            samples[key] = (float(row["dexterity"]), int(row["n_reachable"]))
        if not samples:
            raise SystemExit(f"no samples in {path}")
        self.i0 = min(k[0] for k in samples)
        self.j0 = min(k[1] for k in samples)
        self.k0 = min(k[2] for k in samples)
        self.ni = max(k[0] for k in samples) - self.i0 + 1
        self.nj = max(k[1] for k in samples) - self.j0 + 1
        self.nk = max(k[2] for k in samples) - self.k0 + 1
        print(f"field {self.ni}x{self.nj}x{self.nk} cells "
              f"({len(samples)} sampled) at {step} m")

        # Three integrals: dexterity, reachable cells, usable cells.
        shape = (self.ni + 1) * (self.nj + 1) * (self.nk + 1)
        self.dex = [0.0] * shape
        self.reach = [0] * shape
        self.usable = [0] * shape
        for (i, j, k), (dexterity, n) in samples.items():
            idx = self._idx(i - self.i0 + 1, j - self.j0 + 1, k - self.k0 + 1)
            self.dex[idx] = dexterity
            self.reach[idx] = 1 if n > 0 else 0
            self.usable[idx] = 1 if dexterity >= USABLE else 0
        for table in (self.dex, self.reach, self.usable):
            self._accumulate(table)

    def _idx(self, a, b, c):
        return (a * (self.nj + 1) + b) * (self.nk + 1) + c

    def _accumulate(self, t):
        ni, nj, nk = self.ni, self.nj, self.nk
        for a in range(1, ni + 1):
            for b in range(1, nj + 1):
                for c in range(1, nk + 1):
                    t[self._idx(a, b, c)] += (
                        t[self._idx(a - 1, b, c)]
                        + t[self._idx(a, b - 1, c)]
                        + t[self._idx(a, b, c - 1)]
                        - t[self._idx(a - 1, b - 1, c)]
                        - t[self._idx(a - 1, b, c - 1)]
                        - t[self._idx(a, b - 1, c - 1)]
                        + t[self._idx(a - 1, b - 1, c - 1)]
                    )

    def total(self, t, lo, hi):
        """Inclusive sum over a box given in local (already clamped) indices."""
        a0, b0, c0 = lo
        a1, b1, c1 = hi
        return (
            t[self._idx(a1, b1, c1)]
            - t[self._idx(a0 - 1, b1, c1)] - t[self._idx(a1, b0 - 1, c1)]
            - t[self._idx(a1, b1, c0 - 1)]
            + t[self._idx(a0 - 1, b0 - 1, c1)] + t[self._idx(a0 - 1, b1, c0 - 1)]
            + t[self._idx(a1, b0 - 1, c0 - 1)]
            - t[self._idx(a0 - 1, b0 - 1, c0 - 1)]
        )

    def score(self, world_lo, world_hi):
        """Stats for a world-frame box. Cells outside the field count as unreachable."""
        lo = []
        hi = []
        cells = 1
        for axis, (base, n) in enumerate(
            ((self.i0, self.ni), (self.j0, self.nj), (self.k0, self.nk))
        ):
            a = round(world_lo[axis] / self.step) - base + 1
            b = round(world_hi[axis] / self.step) - base + 1
            cells *= b - a + 1
            lo.append(max(a, 1))
            hi.append(min(b, n))
            if lo[-1] > hi[-1]:
                return None
        return {
            "cells": cells,
            "dex_sum": self.total(self.dex, lo, hi),
            "reach": self.total(self.reach, lo, hi),
            "usable": self.total(self.usable, lo, hi),
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--field", default="reachability_field.csv")
    parser.add_argument("--mesh", default="src/fanuc_custom_tools/assets/KUBTEC_workspace_20cm.stl")
    parser.add_argument("--mesh-scale", type=float, default=0.001)
    parser.add_argument("--step", type=float, default=0.05)
    parser.add_argument("--top", type=int, default=12)
    parser.add_argument("--clear-radius", type=float, default=0.16,
                        help="half-width of the robot's own column at the origin. A "
                             "placement whose footprint covers it needs the base "
                             "standing inside the volume, which is not buildable.")
    parser.add_argument("--output", type=Path, default=Path("placement_scores.csv"))
    args = parser.parse_args()

    field = Field(args.field, args.step)
    lo, hi = read_binary_stl_bounds(args.mesh, args.mesh_scale)
    dims = [hi[i] - lo[i] for i in range(3)]
    print(f"workspace box {dims[0]:.3f} x {dims[1]:.3f} x {dims[2]:.3f} m")

    step = args.step
    results = []
    # yaw 0 and 90 give distinct footprints; 180/270 are the same boxes translated,
    # which the origin sweep already covers.
    for yaw_deg, (dx, dy) in ((0, (dims[0], dims[1])), (90, (dims[1], dims[0]))):
        span = [dx, dy, dims[2]]
        for ix in range(-16, 17):
            for iy in range(-16, 17):
                # Negative z0 puts the box floor below the robot base: the robot
                # stands on a pedestal and reaches down into the volume.
                for iz in range(-9, 9):
                    origin = (ix * step, iy * step, iz * step)
                    world_lo = origin
                    world_hi = tuple(origin[i] + span[i] for i in range(3))
                    stats = field.score(world_lo, world_hi)
                    if stats is None or stats["cells"] == 0:
                        continue
                    r = args.clear_radius
                    base_clear = (
                        world_lo[0] > r or world_hi[0] < -r
                        or world_lo[1] > r or world_hi[1] < -r
                    )
                    results.append({
                        "yaw_deg": yaw_deg,
                        "box_x0": round(origin[0], 3),
                        "box_y0": round(origin[1], 3),
                        "box_z0": round(origin[2], 3),
                        "base_clear": int(base_clear),
                        "cells": stats["cells"],
                        "reach_frac": round(stats["reach"] / stats["cells"], 4),
                        "usable_frac": round(stats["usable"] / stats["cells"], 4),
                        "mean_dex": round(stats["dex_sum"] / stats["cells"], 4),
                        "usable_litres": round(
                            stats["usable"] * (step ** 3) * 1000, 2),
                    })

    results.sort(key=lambda r: -r["usable_litres"])
    with args.output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(results[0].keys()))
        writer.writeheader()
        writer.writerows(results)

    print(f"\nscored {len(results)} placements -> {args.output.resolve()}")

    def table(label, rows):
        print(f"\n{label}")
        print(f"{'yaw':>4} {'box x0':>7} {'box y0':>7} {'box z0':>7} "
              f"{'reach':>7} {'usable':>7} {'meanDex':>8} {'usable L':>9}")
        for row in rows[:args.top]:
            print(f"{row['yaw_deg']:>4} {row['box_x0']:>7.2f} {row['box_y0']:>7.2f} "
                  f"{row['box_z0']:>7.2f} {row['reach_frac']:>6.1%} "
                  f"{row['usable_frac']:>6.1%} {row['mean_dex']:>8.3f} "
                  f"{row['usable_litres']:>9.2f}")

    table(f"UNCONSTRAINED top {args.top} by usable volume (dexterity >= {USABLE}):",
          results)
    clear = [r for r in results if r["base_clear"]]
    table(f"BUILDABLE top {args.top} (robot column clear of the volume, "
          f"|x| or |y| > {args.clear_radius} m):", clear)
    if clear:
        best_any = results[0]["usable_litres"]
        best_clear = clear[0]["usable_litres"]
        print(f"\ncost of keeping the base outside the volume: "
              f"{best_any:.1f} L -> {best_clear:.1f} L "
              f"({(1 - best_clear / best_any) * 100:.0f}% of usable volume)")


if __name__ == "__main__":
    main()
