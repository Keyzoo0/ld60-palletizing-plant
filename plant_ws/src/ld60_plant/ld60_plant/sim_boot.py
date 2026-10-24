"""Staged boot: no box is ever held by two DetachableJoints at the same time.

Every DetachableJoint attaches as soon as its parent model and its child exist. When the 120 boxes got
their pallet weld, their arm's vacuum joint and the truck gripper joint all at once, DART turned the
extra attachments into "welded mirror" bodies, which slowed the physics down and could make it blow
up. So the world starts paused and the models arrive one group of joints at a time:

    1. world (boxes + pallet welds + AMR pallet lock)  -> release, confirmed on each state topic
    2. arms (one vacuum joint per box)                  -> release, confirmed
    3. truck gripper bar (one joint per box, welds itself to the T1 wrist) -> release, confirmed
    4. unpause

    python3 -m ld60_plant.sim_boot --config plant.yaml --arms plant_arms_gz.urdf [--bar t1_bar.sdf]
"""
import argparse
import subprocess
import sys
import threading
import time

import yaml
from gz.msgs.empty_pb2 import Empty
from gz.msgs.stringmsg_pb2 import StringMsg
from gz.transport import Node as GzNode

WORLD = 'plant'


def log(msg):
    print(f'[sim_boot] {msg}', flush=True)


class Joints:
    """Detach a set of DetachableJoints and wait until each one reports "detached"."""

    def __init__(self, node, prefixes):
        self.node = node
        self.state = {}
        self.lock = threading.Lock()
        self.pubs = {}
        for p in prefixes:
            self.node.subscribe(StringMsg, f'{p}/state', self._cb(p))
            self.pubs[p] = self.node.advertise(f'{p}/detach', Empty)

    def _cb(self, p):
        def cb(msg):
            with self.lock:
                self.state[p] = msg.data
        return cb

    def release(self, what, timeout=40.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15 and not all(pub.has_connections() for pub in self.pubs.values()):
            time.sleep(0.1)
        todo = set(self.pubs)
        last = {}
        while todo and time.monotonic() - t0 < timeout:
            now = time.monotonic()
            for p in list(todo):
                with self.lock:
                    st = self.state.get(p)
                if st == 'detached':
                    todo.discard(p)
                elif now - last.get(p, 0) > 1.0:      # (re)send until confirmed, never after
                    self.pubs[p].publish(Empty())
                    last[p] = now
            time.sleep(0.05)
        if todo:
            log(f'{what}: {len(todo)} joints did not confirm the release: {sorted(todo)[:5]}')
            return False
        log(f'{what}: {len(self.pubs)} joints released in {time.monotonic() - t0:.1f} s')
        return True


def watch(node, topic):
    """Latest message on a state topic (subscribe before the event happens)."""
    got = {}
    node.subscribe(StringMsg, topic, lambda m: got.__setitem__('v', m.data))
    return got


def wait_value(got, value, timeout=30.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if got.get('v') == value:
            return True
        time.sleep(0.05)
    return False


def gz_service(args, timeout=10):
    try:
        return subprocess.run(['gz', 'service'] + args, capture_output=True, text=True, timeout=timeout).stdout
    except (OSError, subprocess.TimeoutExpired):
        return ''


def spawn(path, name, xyz=(0.0, 0.0, 0.0)):
    r = subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', WORLD, '-file', path, '-name', name,
                        '-x', str(xyz[0]), '-y', str(xyz[1]), '-z', str(xyz[2])],
                       capture_output=True, text=True, timeout=120)
    if r.returncode != 0:
        log(f'spawning {name} failed:\n{r.stdout}{r.stderr}')
        sys.exit(1)
    log(f'{name} spawned')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', required=True)
    ap.add_argument('--arms', required=True)
    ap.add_argument('--bar', default='')
    args, _ = ap.parse_known_args()
    with open(args.config) as fh:
        cfg = yaml.safe_load(fh)
    n = cfg['production']['pool_per_lane']
    boxes = [f'box_{lane}_{k:02d}' for lane in cfg['cells'] for k in range(1, n + 1)]
    gz = GzNode()

    t0 = time.monotonic()
    while 'data: true' not in gz_service(['-s', f'/world/{WORLD}/control', '--reqtype', 'gz.msgs.WorldControl',
                                          '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', 'pause: true']):
        if time.monotonic() - t0 > 120:
            log('Gazebo world did not come up')
            sys.exit(1)
        time.sleep(1.0)

    # 1. world: pallet welds + AMR pallet lock
    Joints(gz, [f'/pallet/{b}' for b in boxes] + ['/ld60/pallet_lock']).release('pallet welds')
    # 2. arms: vacuum joints
    spawn(args.arms, 'plant_arms')
    Joints(gz, [f'/vacuum/{b}' for b in boxes]).release('vacuum joints')
    # 3. truck gripper bar: welds itself to the T1 wrist, then its box joints are released
    t = cfg.get('truck_robot')
    if t and args.bar:
        bar = t['name'].lower() + '_bar'
        grips = Joints(gz, [f'/tgrip/{b}' for b in boxes])
        bar_state = watch(gz, f'/{bar}/state')
        home = (t['pallet'][0] + 0.45, t['pallet'][1], cfg['production']['pallet_top'] + 0.42)
        spawn(args.bar, bar, home)
        if not wait_value(bar_state, 'attached', 30):
            log(f'{bar} did not attach to the {t["name"]} wrist')
        grips.release('truck gripper joints')
    # 4. run
    for _ in range(10):
        if 'data: true' in gz_service(['-s', f'/world/{WORLD}/control', '--reqtype', 'gz.msgs.WorldControl',
                                       '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', 'pause: false']):
            break
        time.sleep(0.5)
    log(f'world running ({time.monotonic() - t0:.0f} s boot)')


if __name__ == '__main__':
    main()
