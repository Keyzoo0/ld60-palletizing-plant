"""Soak test for the palletizing plant: start production and watch until N pallets are delivered.

    ros2 run ld60_plant plant_check --pallets 2 --timeout 1800
Exit code 0 = target reached with no plant error.
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
        super().__init__('plant_check')
        self.state = None
        self.replies = {}
        self.pub = self.create_publisher(String, '/plant/cmd', 10)
        self.create_subscription(String, '/plant/state', self._on_state, 10)
        self.create_subscription(String, '/plant/reply', self._on_reply, 10)
        self.create_subscription(String, '/plant/log', self._on_log, 100)

    def _on_state(self, msg):
        self.state = json.loads(msg.data)
        self.state_t = time.monotonic()

    def _on_reply(self, msg):
        r = json.loads(msg.data)
        self.replies[r.get('id')] = r

    def _on_log(self, msg):
        e = json.loads(msg.data)
        print(f"  [log {e.get('level')}] {e.get('msg')}", flush=True)

    def send(self, cmd_name, **kw):
        end = time.monotonic() + 10                     # a message sent before DDS matched the subscriber is lost
        while self.pub.get_subscription_count() == 0 and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        rid = f'c{time.monotonic_ns()}'
        self.pub.publish(String(data=json.dumps({'id': rid, 'cmd': cmd_name, **kw})))
        end = time.monotonic() + 10
        while rid not in self.replies and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.replies.get(rid, {'ok': False, 'msg': 'no reply'})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pallets', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=1800)
    ap.add_argument('--speed', type=float, default=None)
    ap.add_argument('--fill', action='store_true', help='start with a full pallet (tests the AMR logistics quickly)')
    args, _ = ap.parse_known_args()
    rclpy.init()
    n = Check()
    end = time.monotonic() + 300                       # the plant boots in 2-4 minutes
    while (n.state is None or n.state['status'] != 'READY') and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.state and n.state['status'] == 'RUNNING':
            break
    if n.state is None:
        print('FAIL: no /plant/state')
        sys.exit(2)
    if args.speed:
        print('set_speed:', n.send('set_speed', value=args.speed)['msg'])
    if args.fill:
        print('fill_pallet:', n.send('fill_pallet')['msg'])
    if n.state['status'] == 'RUNNING':
        print('already running')
    else:
        print('start:', n.send('start'))
    t0 = time.monotonic()
    last = 0
    code = 1
    while time.monotonic() - t0 < args.timeout:
        rclpy.spin_once(n, timeout_sec=0.1)
        s = n.state
        if time.monotonic() - n.state_t > 30:
            print(f'FAIL after {time.monotonic() - t0:.0f}s: no /plant/state for 30 s (simulation crashed?)')
            code = 4
            break
        if s['status'] == 'ERROR':
            print(f"FAIL after {time.monotonic() - t0:.0f}s: {s['error']}")
            code = 3
            break
        if time.monotonic() - last > 10:
            last = time.monotonic()
            lanes = ' '.join(f"{k}:{v['status'][:9]}/{v['placed']}/belt{v['on_belt']}" for k, v in s['lanes'].items())
            print(f"t={time.monotonic() - t0:5.0f}s fill={s['pallet_fill']:2d}/{s['pallet_target']} "
                  f"made={s['boxes_made']} pal={s['boxes_palletized']} log={s['logistics']} "
                  f"del={s['pallets_delivered']} zone={s['zone_owner']} | {lanes}", flush=True)
        if s['pallets_delivered'] >= args.pallets:
            print(f"PASS: {s['pallets_delivered']} pallets in {time.monotonic() - t0:.0f}s, cycles {s['pallet_cycles']}, "
                  f"throughput {s['throughput_h']}/h, AGV wait {s['agv_wait_s']}s, arm wait {s['arm_wait_pallet_s']}s")
            code = 0
            break
    else:
        print('FAIL: timeout')
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
