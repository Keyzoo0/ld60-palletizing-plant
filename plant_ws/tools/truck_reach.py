"""Reach check for the truck robot: pick a row of 4 boxes off the pallet, stack rows on the truck floor.

Usage: python3 truck_reach.py BASE_X PEDESTAL_H STACK_X0 COL_Y [ROWS] [LAYERS]
The robot faces -X (towards the pallet on the truck station at x 13.2). Rows on the pallet run along X
(gripper yaw 0); rows in the truck run along Y (gripper yaw 90), stacked from STACK_X0 towards the cab.
"""
import math
import os
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, '/home/zainul/ros2_ws/src/tm12s_sim')
from tm12s_sim.kin_check import Chain, pose  # noqa: E402

PALLET = (13.2, 0.0)
PALLET_TOP = 0.828
BOX_H = 0.10
ROW_PITCH = 0.11
rng = np.random.default_rng(0)


def solve(ch, lp, rel_yaw, tries=40):
    best = None
    for _ in range(tries):
        seed = np.radians([math.degrees(math.atan2(lp[1], lp[0])) + rng.uniform(-40, 40),
                           rng.uniform(-60, 20), rng.uniform(40, 150), rng.uniform(-90, 90),
                           rng.uniform(60, 120), rng.uniform(-180, 180)])
        q, ok = ch.ik(pose([v * 1000 for v in lp], (180, 0, rel_yaw)), seed, iters=300)
        if ok and q[2] > 0 and (best is None or abs(q[1]) < abs(best[1])):
            best = q
    return best


def main():
    a = sys.argv
    bx, H, x0, cy = (float(v) for v in a[1:5])
    rows = int(a[5]) if len(a) > 5 else 4
    layers = int(a[6]) if len(a) > 6 else 2
    single = subprocess.run(['xacro', os.path.expanduser('~/ros2_ws/src/tm12s_sim/urdf/tm12s_cell.urdf.xacro'),
                             'sim_gz:=false'], capture_output=True, text=True, check=True).stdout
    with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False) as fh:
        fh.write(single)
    ch = Chain(fh.name)
    base_yaw = math.pi                                    # facing -X
    targets = []
    for layer in (2, 1):
        for y in (-0.165, -0.055, 0.055, 0.165):
            z = PALLET_TOP + BOX_H * layer + 0.003
            targets += [(f'pallet L{layer} y{y:+.3f}', (PALLET[0], y, z), 0.0), ('   up', (PALLET[0], y, z + 0.15), 0.0)]
    for layer in range(1, layers + 1):
        for side in (1, -1):
            for r in range(rows):
                x = x0 + r * ROW_PITCH
                z = BOX_H * layer + 0.003
                targets += [(f'stack L{layer} {side:+d} r{r}', (x, side * cy, z), 90.0),
                            ('   up', (x, side * cy, z + 0.25), 90.0)]
    bad = 0
    for name, p, tcp_yaw in targets:
        dx, dy = p[0] - bx, p[1]
        c, s = math.cos(-base_yaw), math.sin(-base_yaw)
        lp = (c * dx - s * dy, s * dx + c * dy, p[2] - H)
        q = solve(ch, lp, tcp_yaw - math.degrees(base_yaw))
        bad += q is None
        print(f'{name:22s} r={math.hypot(lp[0], lp[1]):.2f} z_rel={lp[2]:+.3f} ->',
              'UNREACHABLE' if q is None else np.round(np.degrees(q), 1))
    os.unlink(fh.name)
    print('ALL REACHABLE' if not bad else f'{bad} UNREACHABLE')


if __name__ == '__main__':
    main()
