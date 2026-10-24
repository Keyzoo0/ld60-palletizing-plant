"""AMR manager for the simulated Omron LD-60 (plays the role of ARAM + a tiny fleet manager).

Drives the robot along the fixed route, docks at the pallet station / truck, transfers the pallet
with the roller tops, locks it while driving, simulates the battery, and talks to the web
dashboard through rosbridge with plain JSON strings:
    /amr/cmd   (in)   {"id": .., "cmd": "goto" | "load" | "unload" | "mission" | "stop" | ...}
    /amr/reply (out)  {"id": .., "ok": bool, "msg": str}
    /amr/state (out)  robot status, pose, battery, payload, plan ... (5 Hz)
    /amr/map   (out, latched) layout, route, goals for drawing the map
    /amr/log   (out)  log lines
Localization is Gazebo ground truth (the fixed-route setup has no lidar).
"""
import json
import math
import threading
import time
import traceback

import rclpy
import rclpy.executors
import yaml
from gz.msgs.double_pb2 import Double
from gz.msgs.empty_pb2 import Empty
from gz.msgs.pose_v_pb2 import Pose_V
from gz.msgs.stringmsg_pb2 import StringMsg
from gz.msgs.twist_pb2 import Twist
from gz.transport import Node as GzNode
from gz.transport import SubscribeOptions
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from ld60_plant.route import RouteGraph, follow_line, rotate_step, wrap

WORLD = 'plant'
ROBOT = 'ld60'
PALLET = 'pallet'
DT = 0.05                     # control period (s)
DECK_Z = 0.75


class TaskError(Exception):
    pass


class Aborted(Exception):
    pass


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class AmrManager(Node):
    def __init__(self):
        super().__init__('amr_manager')
        cfg_path = self.declare_parameter('config', '').value
        with open(cfg_path) as fh:
            self.cfg = yaml.safe_load(fh)
        self.p = self.cfg['robot']
        self.graph = RouteGraph(self.cfg)
        self.stations = self.cfg['stations']

        self.lock = threading.RLock()
        self.poses = {}               # name -> (x, y, z, yaw)
        self.lock_state = None        # Gazebo truth: True attached / False detached
        self.status = 'BOOTING'
        self.task = ''
        self.error = ''
        self.at_goal = 'Home'
        self.queue = []
        self.mission = None
        self.mission_loop = False
        self.missions_done = 0
        self.plan_points = []
        self.target = None
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.speed_limit = float(self.p['cruise_speed'])
        self.battery = 100.0
        self.odometer = 0.0
        self.last_xy = None
        self.charging = False
        self.rollers_on = False
        self.locked = False
        self.logs = []
        self.interrupt = threading.Event()
        self.reason = None
        self.paused = False
        self.startup_until = None
        self.cycle_start = None
        self.last_cycle = None

        # ---------------- Gazebo ----------------
        self.gz = GzNode()
        opts = SubscribeOptions()
        opts.msgs_per_sec = 50
        # each model's own PosePublisher (30 Hz): parsing the whole plant's dynamic poses cost ~70 % CPU
        for name in (ROBOT, PALLET):
            self.gz.subscribe(Pose_V, f'/model/{name}/pose', self._pose_cb(name), opts)
        self.gz.subscribe(StringMsg, '/ld60/pallet_lock/state', self._on_lock_state)
        self.cmd_pub = self.gz.advertise(f'/model/{ROBOT}/cmd_vel', Twist)
        self.deck_pub = self.gz.advertise(f'/model/{ROBOT}/link/roller_deck/track_cmd_vel', Double)
        self.station_pub = {name: self.gz.advertise(f'/model/{name}/link/deck_link/track_cmd_vel', Double)
                            for name in self.stations}
        self.attach_pub = self.gz.advertise('/ld60/pallet_lock/attach', Empty)
        self.detach_pub = self.gz.advertise('/ld60/pallet_lock/detach', Empty)

        # ---------------- ROS ----------------
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/amr/cmd', self._on_cmd, 50)
        self.reply_pub = self.create_publisher(String, '/amr/reply', 50)
        self.state_pub = self.create_publisher(String, '/amr/state', 10)
        self.map_pub = self.create_publisher(String, '/amr/map', latched)
        self.log_pub = self.create_publisher(String, '/amr/log', 100)
        self.odom_pub = self.create_publisher(Odometry, '/ld60/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(0.2, self._publish_state)
        self.create_timer(0.1, self._publish_odom)
        self._publish_map()

        self.running = True
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()

    # ================================================================ Gazebo callbacks
    def _pose_cb(self, name):
        def cb(msg):
            self._on_poses(msg, name)
        return cb

    def _on_poses(self, msg, name):
        try:
            with self.lock:
                p = next((q for q in msg.pose if q.name.split('::')[-1] == name), msg.pose[0] if msg.pose else None)
                if p is not None:
                    self.poses[name] = (p.position.x, p.position.y, p.position.z, yaw_of(p.orientation))
                if self.startup_until is None and ROBOT in self.poses and PALLET in self.poses:
                    self.startup_until = time.monotonic() + 3.0
        except Exception:  # noqa: BLE001 - never raise into gz-transport threads
            pass

    def _on_lock_state(self, msg):
        try:
            self.lock_state = msg.data.strip().lower() == 'attached'
        except Exception:  # noqa: BLE001
            pass

    def shutdown_gz(self):
        self._drive(0.0, 0.0)
        self._rollers(0.0, None)
        for topic in [f'/model/{ROBOT}/pose', f'/model/{PALLET}/pose', '/ld60/pallet_lock/state']:
            try:
                self.gz.unsubscribe(topic)
            except Exception:  # noqa: BLE001
                pass

    # ================================================================ helpers
    def now(self):
        """Simulation time (s): watchdogs and timings must not trip when the host runs the sim slowly."""
        return self.get_clock().now().nanoseconds * 1e-9

    def log(self, level, msg):
        entry = {'t': time.strftime('%H:%M:%S'), 'level': level, 'msg': msg}
        with self.lock:
            self.logs.append(entry)
            self.logs = self.logs[-300:]
        if level == 'error':
            self.get_logger().error(msg)
        elif level == 'warn':
            self.get_logger().warning(msg)
        else:
            self.get_logger().info(msg)
        self.log_pub.publish(String(data=json.dumps(entry)))

    def pose(self):
        with self.lock:
            return self.poses.get(ROBOT)

    def pallet_pose(self):
        with self.lock:
            return self.poses.get(PALLET)

    def _drive(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.cmd_pub.publish(t)
        self.v_cmd, self.w_cmd = v, w

    def _rollers(self, deck, station, station_v=0.0):
        self.deck_pub.publish(Double(data=float(deck)))
        for name, pub in self.station_pub.items():
            pub.publish(Double(data=float(station_v if name == station else 0.0)))
        self.rollers_on = abs(deck) > 1e-6 or abs(station_v) > 1e-6

    def pallet_on_deck(self):
        """Pallet sits on the AMR roller top: (bool, x along the robot axis)."""
        r, pl = self.pose(), self.pallet_pose()
        if r is None or pl is None:
            return False, None
        dx, dy = pl[0] - r[0], pl[1] - r[1]
        px = math.cos(r[3]) * dx + math.sin(r[3]) * dy
        py = -math.sin(r[3]) * dx + math.cos(r[3]) * dy
        on = abs(px) < 0.36 and abs(py) < 0.12 and abs(pl[2] - (DECK_Z + 0.04)) < 0.06
        return on, px

    def _set_lock(self, locked):
        (self.attach_pub if locked else self.detach_pub).publish(Empty())
        self.locked = locked
        t0 = self.now()
        while self.now() - t0 < 1.0 and self.lock_state is not None and self.lock_state != locked:
            time.sleep(0.02)

    # ================================================================ worker
    def _worker(self):
        last = self.now()
        while self.running:
            now = self.now()
            # boot: DetachableJoint starts attached -> free the pallet before anything moves
            if self.status == 'BOOTING':
                if self.startup_until is not None:
                    self.detach_pub.publish(Empty())
                    if time.monotonic() > self.startup_until:      # wall clock: the sim may still be paused
                        with self.lock:
                            self.status = 'IDLE'
                        self.locked = False
                        self.log('info', 'LD-60 ready at Home. Localization: ground truth (fixed route, no lidar).')
                time.sleep(0.1)
                continue
            self._battery_tick(now - last)
            last = now
            task = None
            with self.lock:
                if self.status == 'IDLE' and self.queue:
                    task = self.queue.pop(0)
                elif self.status == 'IDLE' and not self.queue and self.mission and self.mission_loop:
                    self._enqueue_mission(self.mission)
            if task is None:
                if self.status in ('IDLE', 'ERROR', 'ESTOP'):
                    self._drive(0.0, 0.0) if abs(self.v_cmd) + abs(self.w_cmd) > 0 else None
                time.sleep(DT)
                continue
            self._run_task(task)

    def _run_task(self, task):
        kind = task['kind']
        with self.lock:
            self.status = {'goto': 'MOVING', 'load': 'TRANSFER', 'unload': 'TRANSFER',
                           'wait': 'WAITING', 'charge': 'CHARGING'}.get(kind, 'BUSY')
            self.task = task.get('label', kind)
            self.interrupt.clear()
            self.reason = None
        try:
            if kind == 'goto':
                self._do_goto(task['goal'])
            elif kind == 'load':
                self._do_transfer(load=True)
            elif kind == 'unload':
                self._do_transfer(load=False)
            elif kind == 'wait':
                self._sleep(task['seconds'])
            elif kind == 'charge':
                self._do_charge(task.get('until', 95.0))
            elif kind == 'mission_end':
                self.missions_done += 1
                if self.cycle_start is not None:
                    self.last_cycle = round(self.now() - self.cycle_start, 1)
                self.cycle_start = None
                self.log('info', f'Mission "{task["name"]}" complete (#{self.missions_done}'
                                 f'{", " + str(self.last_cycle) + " s" if self.last_cycle else ""})')
            with self.lock:
                if self.status not in ('ESTOP', 'ERROR'):
                    self.status = 'IDLE'
                self.task = ''
        except Aborted:
            self._drive(0.0, 0.0)
            self._rollers(0.0, None)
            with self.lock:
                if self.reason == 'estop':
                    self.status = 'ESTOP'
                else:
                    self.status = 'IDLE'
                self.task = ''
                self.queue.clear()
                self.mission = None
        except TaskError as e:
            self._drive(0.0, 0.0)
            self._rollers(0.0, None)
            with self.lock:
                self.status = 'ERROR'
                self.error = str(e)
                self.task = ''
                self.queue.clear()
                self.mission = None
            self.log('error', str(e))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(traceback.format_exc())
            self._drive(0.0, 0.0)
            self._rollers(0.0, None)
            with self.lock:
                self.status = 'ERROR'
                self.error = f'internal error: {e}'
                self.queue.clear()
                self.mission = None

    def _check(self):
        """Called every control tick: honours pause, stop and e-stop."""
        if self.interrupt.is_set():
            raise Aborted()
        if self.paused:
            self._drive(0.0, 0.0)
            prev = self.status
            with self.lock:
                self.status = 'PAUSED'
            while self.paused and not self.interrupt.is_set():
                time.sleep(DT)
            if self.interrupt.is_set():
                raise Aborted()
            with self.lock:
                self.status = prev

    def _sleep(self, seconds):
        end = self.now() + seconds
        while self.now() < end:
            self._check()
            time.sleep(DT)

    # ---------------------------------------------------------------- driving
    def _do_goto(self, goal):
        if goal not in self.graph.goals:
            raise TaskError(f'Unknown goal {goal}')
        on, _ = self.pallet_on_deck()
        if on and not self.locked:
            raise TaskError('Pallet on the roller top is not locked - refusing to drive')
        if self.battery < 3.0 and goal != 'Home':
            raise TaskError('Battery critically low - send the robot Home to charge')
        r = self.pose()
        steps = self.graph.plan((r[0], r[1], r[3]), self.at_goal, goal, self.p)
        self.target = goal
        self.plan_points = [(r[0], r[1])] + [s['to'] for s in steps if s['type'] == 'line']
        self.log('info', f'Going to {goal} ({sum(1 for s in steps if s["type"] == "line")} segments)')
        self.at_goal = None
        self.charging = False
        for s in steps:
            if s['type'] == 'rotate':
                self._rotate(s['yaw'])
            else:
                self._line(s)
        self._drive(0.0, 0.0)
        r = self.pose()
        g = self.graph.goals[goal]
        dpos = math.hypot(r[0] - g['x'], r[1] - g['y'])
        dyaw = abs(math.degrees(wrap(r[3] - math.radians(g['yaw'])))) if g.get('yaw') is not None else 0.0
        tol_d = self.p['dock_distance_tol'] * 3 if g.get('dock') else self.p['goal_distance_tol']
        if dpos > max(tol_d, 0.03) or dyaw > max(self.p['goal_angle_tol'], 2.0):
            raise TaskError(f'Arrived at {goal} out of tolerance ({dpos * 1000:.0f} mm, {dyaw:.1f} deg)')
        self.at_goal = goal
        self.target = None
        self.plan_points = []
        self.log('info', f'Arrived at {goal}: error {dpos * 1000:.1f} mm, {dyaw:.2f} deg')

    def _rotate(self, target):
        with self.lock:
            if self.status != 'PAUSED':
                self.status = 'MOVING'
        w = 0.0
        t0 = self.now()
        tol = math.radians(0.5)
        while True:
            self._check()
            r = self.pose()
            w, err = rotate_step(r[3], target, w, DT, self.p)
            if abs(err) < tol:
                break
            if self.now() - t0 > 20:
                raise TaskError('Rotation timeout - is the robot blocked?')
            self._drive(0.0, w)
            time.sleep(DT)
        self._drive(0.0, 0.0)
        time.sleep(0.1)

    def _line(self, seg):
        with self.lock:
            if self.status != 'PAUSED':
                self.status = 'DOCKING' if seg['dock'] or seg['reverse'] else 'MOVING'
        v = 0.0
        L = math.hypot(seg['to'][0] - seg['from'][0], seg['to'][1] - seg['from'][1])
        budget = L / max(0.1, min(self.speed_limit, self.p['dock_speed'] if seg['dock'] else self.speed_limit)) * 3 + 15
        t0 = self.now()
        tol = self.p['dock_distance_tol'] if seg['dock'] else self.p['goal_distance_tol']
        while True:
            self._check()
            r = self.pose()
            v, w, remaining, cross = follow_line((r[0], r[1], r[3]), seg, v, DT, self.p, self.speed_limit)
            if remaining <= tol:
                break
            if abs(cross) > 0.30:
                raise TaskError(f'Off path by {abs(cross) * 100:.0f} cm on the way to {seg["name"]}')
            if self.now() - t0 > budget:
                raise TaskError(f'Segment to {seg["name"]} timed out - robot blocked?')
            self._drive(v, w)
            time.sleep(DT)
        self._drive(0.0, 0.0)

    # ---------------------------------------------------------------- pallet transfer
    def _do_transfer(self, load):
        st_name = self.graph.goals.get(self.at_goal or '', {}).get('station')
        if not st_name:
            raise TaskError('Not docked at a roller station (go to PalletStation or TruckDock first)')
        st = self.stations[st_name]
        st_yaw = math.radians(st['yaw'])
        r, pl = self.pose(), self.pallet_pose()
        on, px = self.pallet_on_deck()
        if load:
            if on:
                raise TaskError('Already carrying the pallet')
            # pallet must be on this station
            if math.hypot(pl[0] - st['x'], pl[1] - st['y']) > 0.6:
                raise TaskError(f'No pallet on {st["label"]}')
        else:
            if not on:
                raise TaskError('No pallet on the roller top')
            if math.hypot(pl[0] - st['x'], pl[1] - st['y']) < 0.45 and abs(px) > 0.3:
                raise TaskError('Station already has a pallet')
        v = float(self.p['roller_speed'])
        self.log('info', f'{"Loading" if load else "Unloading"} pallet at {st["label"]}')
        if not load:
            self._set_lock(False)
        t0 = self.now()
        progress_t, progress_val = self.now(), None
        try:
            while True:
                self._check()
                r, pl = self.pose(), self.pallet_pose()
                if load:
                    on, px = self.pallet_on_deck()
                    tx, ty = r[0], r[1]
                else:
                    tx, ty = st['x'], st['y']
                # signed distance still to travel along the hand-over axis (world, robot heading)
                hx, hy = math.cos(r[3]), math.sin(r[3])
                remaining = (tx - pl[0]) * hx + (ty - pl[1]) * hy
                if abs(remaining) < (0.008 if load else 0.02):
                    break
                d = math.copysign(1.0, remaining)            # +1 = along the robot heading
                deck = v * d                                  # robot rollers run along robot +X
                station = v * d * (hx * math.cos(st_yaw) + hy * math.sin(st_yaw))
                if abs(remaining) < 0.05:                     # slow down for an exact stop
                    deck *= 0.4
                    station *= 0.4
                self._rollers(deck, st_name, station)
                if self.now() - getattr(self, '_trace_t', 0) > 1.0:
                    self._trace_t = self.now()
                    self.get_logger().debug(f'transfer: remaining {remaining * 1000:.0f} mm, pallet z {pl[2]:.3f}, '
                                           f'deck cmd {deck:.3f}, station cmd {station:.3f}')
                # stall watchdog: the pallet must keep moving while the rollers run
                if progress_val is None or abs(remaining - progress_val) > 0.005:
                    progress_t, progress_val = self.now(), remaining
                elif self.now() - progress_t > 3.0:
                    raise TaskError(f'Pallet stuck during hand-over at {st["label"]} '
                                    f'({abs(remaining) * 100:.0f} cm to go) - check the deck heights / docking')
                if self.now() - t0 > 60:              # the stall watchdog above catches a stuck pallet
                    raise TaskError('Pallet transfer timed out - check the rollers / pallet')
                time.sleep(DT)
        finally:
            self._rollers(0.0, None)
        time.sleep(0.3)
        if load:
            self._set_lock(True)
            self.log('info', f'Pallet loaded and locked ({self.now() - t0:.1f} s)')
        else:
            on, _ = self.pallet_on_deck()
            if on:
                raise TaskError('Pallet did not leave the roller top')
            self.log('info', f'Pallet delivered to {st["label"]} ({self.now() - t0:.1f} s)')

    def _do_charge(self, until):
        if self.at_goal != 'Home':
            raise TaskError('Charging only works on the charger at Home')
        self.charging = True
        self.log('info', f'Charging to {until:.0f} %')
        while self.battery < until:
            self._check()
            time.sleep(0.2)
        self.charging = False
        self.log('info', 'Charging done')

    def _battery_tick(self, dt):
        b = self.cfg['battery']
        if self.charging:
            self.battery = min(100.0, self.battery + b['charge_rate'] * dt)
        else:
            moving = abs(self.v_cmd) > 1e-3 or abs(self.w_cmd) > 1e-3
            rate = b['drain_moving'] if moving else b['drain_idle']
            if self.rollers_on:
                rate += b['drain_rollers']
            self.battery = max(0.0, self.battery - rate * dt)
        r = self.pose()
        if r is not None:
            if self.last_xy is not None:
                self.odometer += math.hypot(r[0] - self.last_xy[0], r[1] - self.last_xy[1])
            self.last_xy = (r[0], r[1])

    # ================================================================ missions
    def _enqueue_mission(self, name):
        if name == 'pallet_cycle':
            # Phase 1 test: carry the pallet to the truck and bring it back
            self.queue += [
                {'kind': 'goto', 'goal': 'PalletStation', 'label': 'Go to Pallet Station'},
                {'kind': 'load', 'label': 'Load pallet (roller in)'},
                {'kind': 'goto', 'goal': 'TruckDock', 'label': 'Drive to truck'},
                {'kind': 'unload', 'label': 'Unload pallet into truck'},
                {'kind': 'wait', 'seconds': 3.0, 'label': 'Truck unloading (simulated)'},
                {'kind': 'load', 'label': 'Take empty pallet back'},
                {'kind': 'goto', 'goal': 'PalletStation', 'label': 'Return to station'},
                {'kind': 'unload', 'label': 'Return pallet to station'},
                {'kind': 'mission_end', 'name': name, 'label': 'Mission done'},
            ]
            self.cycle_start = self.now()
        elif name == 'charge':
            self.queue += [{'kind': 'goto', 'goal': 'Home', 'label': 'Go home'},
                           {'kind': 'charge', 'until': 95.0, 'label': 'Charging'}]
        else:
            raise TaskError(f'Unknown mission {name}')

    # ================================================================ commands
    def _reply(self, rid, ok, msg=''):
        self.reply_pub.publish(String(data=json.dumps({'id': rid, 'ok': ok, 'msg': msg})))

    def _on_cmd(self, msg):
        try:
            req = json.loads(msg.data)
            rid, cmd = req.get('id'), req.get('cmd')
        except (ValueError, AttributeError):
            return
        try:
            ok, text = self._handle(cmd, req)
        except Exception as e:  # noqa: BLE001
            ok, text = False, f'{cmd} failed: {e}'
        self._reply(rid, ok, text)

    def _handle(self, cmd, req):
        st = self.status
        busy = st not in ('IDLE',)
        if cmd == 'get_map':
            self._publish_map()
            return True, ''
        if cmd == 'get_logs':
            for e in list(self.logs)[-80:]:
                self.log_pub.publish(String(data=json.dumps(e)))
            return True, ''
        if cmd == 'estop':
            self.reason = 'estop'
            self.interrupt.set()
            self._drive(0.0, 0.0)
            self._rollers(0.0, None)
            with self.lock:
                self.status = 'ESTOP'
                self.error = 'Emergency stop'
                self.queue.clear()
                self.mission = None
            self.log('error', 'EMERGENCY STOP')
            return True, 'E-STOP'
        if cmd == 'stop':
            self.reason = 'stop'
            self.interrupt.set()
            self.paused = False
            with self.lock:
                self.queue.clear()
                self.mission = None
            self.log('info', 'Stop: tasks cancelled')
            return True, 'Stopped'
        if cmd == 'pause':
            if st in ('IDLE', 'ERROR', 'ESTOP', 'BOOTING'):
                return False, 'Nothing to pause'
            self.paused = True
            return True, 'Paused'
        if cmd == 'resume':
            self.paused = False
            return True, 'Resumed'
        if cmd == 'reset':
            if st not in ('ERROR', 'ESTOP'):
                return False, 'Nothing to reset'
            self.interrupt.clear()
            with self.lock:
                self.status = 'IDLE'
                self.error = ''
            self.log('info', 'Reset')
            return True, 'Reset'
        if cmd == 'set_speed':
            v = float(req.get('value', 0))
            if not 0.1 <= v <= self.p['max_speed']:
                return False, f'Speed must be 0.1 .. {self.p["max_speed"]} m/s'
            self.speed_limit = v
            return True, f'Max speed {v:.2f} m/s'
        if cmd in ('goto', 'load', 'unload', 'mission', 'charge'):
            if st in ('ERROR', 'ESTOP', 'BOOTING'):
                return False, f'Robot is {st} - reset first'
            with self.lock:
                if cmd == 'goto':
                    goal = req.get('goal')
                    if goal not in self.graph.goals:
                        return False, f'Unknown goal {goal}'
                    if busy:
                        return False, 'Robot is busy - stop the current task first'
                    self.queue.append({'kind': 'goto', 'goal': goal, 'label': f'Go to {goal}'})
                elif cmd in ('load', 'unload'):
                    if busy:
                        return False, 'Robot is busy'
                    self.queue.append({'kind': cmd, 'label': f'{cmd.title()} pallet'})
                elif cmd == 'charge':
                    if busy:
                        return False, 'Robot is busy'
                    self._enqueue_mission('charge')
                else:
                    name = req.get('name', 'pallet_cycle')
                    if busy or self.queue:
                        return False, 'Robot is busy'
                    self.mission = name
                    self.mission_loop = bool(req.get('loop', False))
                    self._enqueue_mission(name)
                    self.log('info', f'Mission "{name}" started{" (loop)" if self.mission_loop else ""}')
            return True, 'Queued'
        return False, f'Unknown command {cmd!r}'

    # ================================================================ publishing
    def _publish_map(self):
        g = self.graph
        data = {
            'layout': self.cfg.get('layout', {}),
            'goals': {k: v for k, v in g.goals.items()},
            'nodes': {k: v for k, v in g.nodes.items()},
            'edges': self.cfg['edges'],
            'stations': self.stations,
            'robot': {'length': self.p['length'], 'width': self.p['width'], 'max_speed': self.p['max_speed']},
        }
        self.map_pub.publish(String(data=json.dumps(data)))

    def _publish_state(self):
        r, pl = self.pose(), self.pallet_pose()
        on, _ = self.pallet_on_deck() if r else (False, None)
        with self.lock:
            st = {
                'status': self.status, 'paused': self.paused, 'task': self.task, 'error': self.error,
                'pose': [round(r[0], 3), round(r[1], 3), round(math.degrees(r[3]), 1)] if r else None,
                'pallet': [round(pl[0], 3), round(pl[1], 3), round(pl[2], 3), round(math.degrees(pl[3]), 1)] if pl else None,
                'v': round(self.v_cmd, 2), 'w': round(math.degrees(self.w_cmd), 1),
                'speed_limit': self.speed_limit, 'battery': round(self.battery, 1), 'charging': self.charging,
                'payload': on, 'locked': self.locked, 'rollers': self.rollers_on,
                'at_goal': self.at_goal, 'target': self.target, 'plan': self.plan_points,
                'queue': [t.get('label', t['kind']) for t in self.queue],
                'mission': self.mission, 'loop': self.mission_loop, 'missions_done': self.missions_done,
                'last_cycle': self.last_cycle, 'odometer': round(self.odometer, 1),
                'cycle_elapsed': round(self.now() - self.cycle_start, 1) if self.cycle_start else None,
            }
        self.state_pub.publish(String(data=json.dumps(st)))

    def _publish_odom(self):
        r = self.pose()
        if r is None:
            return
        now = self.get_clock().now().to_msg()
        qz, qw = math.sin(r[3] / 2), math.cos(r[3] / 2)
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = 'world'
        od.child_frame_id = 'ld60/base_link'
        od.pose.pose.position.x, od.pose.pose.position.y = r[0], r[1]
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.twist.twist.linear.x, od.twist.twist.angular.z = self.v_cmd, self.w_cmd
        self.odom_pub.publish(od)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'world'
        t.child_frame_id = 'ld60/base_link'
        t.transform.translation.x, t.transform.translation.y = r[0], r[1]
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = AmrManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.running = False
        node.shutdown_gz()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
