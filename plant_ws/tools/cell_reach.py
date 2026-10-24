"""Reach study for the 4 palletizing cells (numpy IK on the official TM12S URDF).

Usage: python3 cell_reach.py /tmp/tm12s.urdf
"""
import itertools
import math
import sys

import numpy as np

sys.path.insert(0, '/home/zainul/ros2_ws/src/tm12s_sim')
from tm12s_sim.kin_check import Chain, pose  # noqa: E402

PALLET_TOP = 0.748 + 0.08
BOX = (0.15, 0.11, 0.10)
XS = [-0.225, -0.075, 0.075, 0.225]
YS = [-0.165, -0.055, 0.055, 0.165]
CONV_TOP = 0.75


def local(p, base, yaw):
    dx, dy = p[0] - base[0], p[1] - base[1]
    c, s = math.cos(-yaw), math.sin(-yaw)
    return (c * dx - s * dy, s * dx + c * dy, p[2])


def study(ch, name, base, pick, quad, base_yaw, tcp_yaw):
    sx, sy = quad
    slots = []
    for layer in (0, 1):
        for x in XS:
            for y in YS:
                if (x > 0) == (sx > 0) and (y > 0) == (sy > 0):
                    slots.append((x, y, PALLET_TOP + BOX[2] * (layer + 1) + 0.003))
    targets = [('pick', (pick[0], pick[1], CONV_TOP + BOX[2] + 0.003)),
               ('pick_up', (pick[0], pick[1], CONV_TOP + BOX[2] + 0.18))]
    for i, s in enumerate(slots):
        targets += [(f's{i}', s), (f's{i}_up', (s[0], s[1], s[2] + 0.15))]
    seed = np.radians([0, -18, 112, -4, 90, 111])
    worst = 0
    ok_all = True
    js = []
    for tname, p in targets:
        lp = local(p, base, base_yaw)
        rel_yaw = math.degrees(tcp_yaw - base_yaw)
        s = seed.copy()
        s[0] = math.atan2(lp[1], lp[0]) + math.radians(21)
        q, ok = ch.ik(pose([v * 1000 for v in lp], (180, 0, rel_yaw)), s)
        ok_all &= ok
        r = math.hypot(lp[0], lp[1])
        worst = max(worst, r)
        js.append(np.degrees(q))
    j = np.array(js)
    spread = np.ptp(j, axis=0)
    print(f'{name} base_yaw={math.degrees(base_yaw):6.1f} tcp_yaw={math.degrees(tcp_yaw):6.1f} '
          f'ok={ok_all} max_r={worst:.2f} m  J-range(deg)={np.round(spread, 0)}')
    return ok_all, spread


def main():
    ch = Chain(sys.argv[1])
    cells = {
        'A1': ((0.35, 0.95), (0.95, 1.07), (+1, +1)),
        'A2': ((-0.35, 0.95), (-0.95, 1.07), (-1, +1)),
        'A3': ((-0.35, -0.95), (-0.95, -1.07), (-1, -1)),
        'A4': ((0.35, -0.95), (0.95, -1.07), (+1, -1)),
    }
    for name, (base, pick, quad) in cells.items():
        best = None
        for by, ty in itertools.product(range(-180, 180, 45), range(-180, 180, 90)):
            ok, spread = study(ch, name, base, pick, quad, math.radians(by), math.radians(ty)) if False else (None, None)
        # evaluate a few sensible candidates explicitly
        for by in (-135, -90, -45, 0, 45, 90, 135, 180):
            for ty in (0, 90, 180, -90):
                ok, spread = study(ch, name, base, pick, quad, math.radians(by), math.radians(ty))
                score = spread[0] + 0.5 * sum(spread[1:])
                if ok and (best is None or score < best[0]):
                    best = (score, by, ty)
        print(f'==> {name} best base_yaw={best[1]} tcp_yaw={best[2]} (score {best[0]:.0f})\n' if best else f'==> {name}: NO SOLUTION\n')


if __name__ == '__main__':
    main()
