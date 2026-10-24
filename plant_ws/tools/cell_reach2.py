"""Per-target reach check with many IK seeds (elbow-up solutions only).

Usage: python3 cell_reach2.py URDF BASE_YAW_DEG TCP_YAW_DEG BASE_X BASE_Y PEDESTAL_H [CONV_X CONV_Y] [QX QY]
"""
import math
import sys

import numpy as np

sys.path.insert(0, '/home/zainul/ros2_ws/src/tm12s_sim')
from tm12s_sim.kin_check import Chain, pose  # noqa: E402

PALLET_TOP = 0.828
CONV_TOP = 0.75
rng = np.random.default_rng(0)


def local(p, base, yaw):
    dx, dy = p[0] - base[0], p[1] - base[1]
    c, s = math.cos(-yaw), math.sin(-yaw)
    return (c * dx - s * dy, s * dx + c * dy, p[2])


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
    ch = Chain(a[1])
    base_yaw, tcp_yaw = math.radians(float(a[2])), float(a[3])
    base, H = (float(a[4]), float(a[5])), float(a[6])
    conv = (float(a[7]), float(a[8])) if len(a) > 8 else (0.95, 1.07)
    qx, qy = (float(a[9]), float(a[10])) if len(a) > 10 else (1, 1)
    targets = [('pick', (conv[0], conv[1], CONV_TOP + 0.103)), ('pick_up', (conv[0], conv[1], CONV_TOP + 0.28))]
    for layer in (1, 2):
        for x in (0.075 * qx, 0.225 * qx):
            for y in (0.055 * qy, 0.165 * qy):
                z = PALLET_TOP + 0.1 * layer + 0.003
                targets += [(f'L{layer} ({x:+.3f},{y:+.3f})', (x, y, z)), ('   up', (x, y, z + 0.15))]
    bad = 0
    for name, p in targets:
        lp = local(p, base, base_yaw)
        lp = (lp[0], lp[1], lp[2] - H)
        q = solve(ch, lp, tcp_yaw - math.degrees(base_yaw))
        bad += q is None
        print(f'{name:22s} r={math.hypot(lp[0], lp[1]):.2f} z_rel={lp[2]:.3f} ->',
              'UNREACHABLE' if q is None else np.round(np.degrees(q), 1))
    print('ALL REACHABLE' if not bad else f'{bad} UNREACHABLE')


if __name__ == '__main__':
    main()
