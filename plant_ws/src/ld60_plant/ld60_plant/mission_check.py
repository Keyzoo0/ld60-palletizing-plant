"""End-to-end check of the AMR: run missions and follow every state change.

    python3 -m ld60_plant.mission_check --cycles 1 --speed 1.0
Exit code 0 = all missions finished without error.
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Check(Node):
    def __init__(self):
        super().__init__('amr_mission_check')
        self.state, self.replies, self.n = None, {}, 0
        self.pub = self.create_publisher(String, '/amr/cmd', 10)
        self.create_subscription(String, '/amr/state', lambda m: setattr(self, 'state', json.loads(m.data)), 10)
        self.create_subscription(String, '/amr/reply', self._reply, 10)
        self.create_subscription(String, '/amr/log', lambda m: print('   log:', json.loads(m.data)['msg']), 50)

    def _reply(self, m):
        r = json.loads(m.data)
        self.replies[r['id']] = r

    def cmd(self, cmd_name, **kw):
        self.n += 1
        rid = f'm{self.n}'
        self.pub.publish(String(data=json.dumps({'id': rid, 'cmd': cmd_name, **kw})))
        end = time.monotonic() + 10
        while rid not in self.replies and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.replies.get(rid, {'ok': False, 'msg': 'no reply'})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cycles', type=int, default=1)
    ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--timeout', type=float, default=600)
    a = ap.parse_args()
    rclpy.init()
    c = Check()
    t0 = time.monotonic()
    while (c.state is None or c.state['status'] == 'BOOTING') and time.monotonic() - t0 < 120:
        rclpy.spin_once(c, timeout_sec=0.1)
    if c.state is None:
        print('FAIL: no /amr/state')
        return 1
    for _ in range(20):
        rclpy.spin_once(c, timeout_sec=0.05)
    print('speed:', c.cmd('set_speed', value=a.speed)['msg'])
    start_done = c.state['missions_done']
    r = c.cmd('mission', name='pallet_cycle', loop=a.cycles > 1)
    print('mission:', r)
    if not r['ok']:
        return 1
    last = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < a.timeout:
        rclpy.spin_once(c, timeout_sec=0.05)
        st = c.state
        key = (st['status'], st['task'])
        if key != last:
            last = key
            print(f'  [{time.monotonic() - t0:6.1f}s] {st["status"]:9s} {st["task"]:32s} pose={st["pose"]} '
                  f'payload={st["payload"]} lock={st["locked"]} batt={st["battery"]}')
        if st['status'] in ('ERROR', 'ESTOP'):
            print('FAIL:', st['error'])
            return 1
        if st['missions_done'] - start_done >= a.cycles:
            c.cmd('stop')
            print(f'PASS: {a.cycles} mission(s), last cycle {st["last_cycle"]} s, odometer {st["odometer"]} m, '
                  f'pallet at {st["pallet"]}')
            return 0
    print('FAIL: timeout')
    c.cmd('stop')
    return 1


if __name__ == '__main__':
    sys.exit(main())
