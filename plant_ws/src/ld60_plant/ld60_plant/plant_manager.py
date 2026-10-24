"""Plant manager: machines, conveyors, 4 palletizing TM12S cells, the AMR pallet logistics and the truck robot.

    machines   - put a box on their conveyor every `machine_period` s (boxes are recycled from a pool)
    conveyors  - run until a box reaches the end stop (photo-eye), then stop until it is picked
    arms       - pick at the conveyor end, wait for the pallet, take the pallet-zone interlock,
                 place into their own quadrant (4 x 4 x 2 pattern), release the interlock
    logistics  - send the LD-60 to the station when the pallet holds `dispatch_at` boxes, take the
                 full pallet to the truck, bring the empty pallet back, park the AMR at Home (charging
                 when needed)
    truck robot  - 5th TM12S in the truck: a 4-cup bar takes the pallet row by row (top layer first) and
                 stacks the rows in the cargo area; a full truck departs and an empty one docks
JSON over rosbridge: /plant/cmd (in), /plant/reply, /plant/state (4 Hz), /plant/log.
"""
import json
import math
import subprocess
import threading
import time
import traceback

import rclpy
import rclpy.executors
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from gz.msgs.double_pb2 import Double
from gz.msgs.empty_pb2 import Empty
from gz.msgs.pose_v_pb2 import Pose_V
from gz.transport import Node as GzNode
from gz.transport import SubscribeOptions
from moveit_msgs.msg import (BoundingVolume, CollisionObject, Constraints, JointConstraint,
                             MoveItErrorCodes, OrientationConstraint, PlanningScene, PositionConstraint, RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

from ld60_plant.world_gen import box_name, conveyor_geometry, storage_pose

WORLD = 'plant'
PIPELINE = 'pilz_industrial_motion_planner'
ERRORS = {getattr(MoveItErrorCodes, k): k for k in dir(type(MoveItErrorCodes))
          if k.isupper() and isinstance(getattr(MoveItErrorCodes, k, None), int)}


class PlantError(Exception):
    pass


class Stopped(Exception):
    pass


def quat_from_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                              math.cos(y / 2), math.sin(y / 2))
    return Quaternion(x=sr * cp * cy - cr * sp * sy, y=cr * sp * cy + sr * cp * sy,
                      z=cr * cp * sy - sr * sp * cy, w=cr * cp * cy + sr * sp * sy)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Lane:
    """One production cell: machine + conveyor + TM12S."""

    def __init__(self, name, cell, prod, pedestal):
        self.name = name
        self.p = name.lower()
        self.cell = cell
        self.geo = conveyor_geometry(cell, prod)
        self.joints = [f'{self.p}_joint_{i}' for i in range(1, 7)]
        self.tcp_link = f'{self.p}_vacuum_tcp'
        self.tcp_yaw = math.radians(cell['tcp_yaw'])
        self.quadrant = cell['quadrant']
        self.boxes = [box_name(name, k) for k in range(1, prod['pool_per_lane'] + 1)]
        self.status = 'IDLE'
        self.task = ''
        self.placed = 0            # boxes placed on the current pallet
        self.held = None
        self.last_spawn = 0.0
        self.made = 0
        self.palletized = 0
        self.busy_time = 0.0
        self.belt_running = False
        self.picking = False                        # belt held still until the box is lifted clear
        self.thread = None


class TruckArm:
    """5th TM12S inside the truck with a 4-cup bar gripper: pallet row -> truck stack."""

    def __init__(self, t, box_h):
        self.name = t['name']
        self.p = self.name.lower()
        self.cell = t
        self.joints = [f'{self.p}_joint_{i}' for i in range(1, 7)]
        self.tcp_link = f'{self.p}_vacuum_tcp'
        self.tcp_yaw = 0.0
        st = t['stack']
        # truck stack: layer by layer (the wrist must never go down next to a full-height column),
        # each layer from the cab end towards the door
        self.slots = []
        for layer in range(1, st['layers'] + 1):
            for r in reversed(range(st['rows'])):
                for side in (1, -1):
                    self.slots.append((st['x0'] + r * st['pitch'], side * st['col_y'], layer * box_h + 0.003))
        self.status = 'IDLE'
        self.task = ''
        self.held = []
        self.homed = False
        self.busy_time = 0.0
        self.rows = []                              # rows (lists of boxes) in the truck, in loading order
        self.boxes_loaded = 0
        self.trucks_departed = 0
        self.truck_state = 'DOCKED'                 # DOCKED DEPARTING


class PlantManager(Node):
    def __init__(self):
        super().__init__('plant_manager')
        self.cb = ReentrantCallbackGroup()
        with open(self.declare_parameter('config', '').value) as fh:
            self.cfg = yaml.safe_load(fh)
        with open(self.declare_parameter('homes', '').value) as fh:
            self.homes = {k: [math.radians(v) for v in q] for k, q in yaml.safe_load(fh)['homes_deg'].items()}
        self.prod = self.cfg['production']
        self.lanes = {n: Lane(n, c, self.prod, self.cfg['pedestal_height']) for n, c in self.cfg['cells'].items()}
        self.box_lane = {b: ln for ln in self.lanes.values() for b in ln.boxes}
        self.slots = self._make_slots()
        self.truck = TruckArm(self.cfg['truck_robot'], self.prod['box']['size'][2])
        self.arms = {**self.lanes, self.truck.name: self.truck}

        self.lock = threading.RLock()
        # pallet-zone interlock: arms in neighbouring quadrants never work above the pallet together,
        # diagonal pairs (A1+A3, A2+A4) may; waiters are served first come, first served
        self.zone_cv = threading.Condition()
        self.zone_active = set()
        self.zone_queue = {}                        # lane -> ticket
        self.zone_ticket = 0
        self.poses = {}                             # entity -> (x, y, z, yaw)
        self.pose_rx = 0                            # pose messages received
        self.joint_pos = {}
        self.amr = {}
        self.status = 'BOOTING'                     # BOOTING READY RUNNING STOPPING ERROR
        self.error = ''
        self.auto_logistics = True
        self.arm_speed = 0.8
        self.logistics = 'IDLE'                     # IDLE DISPATCHED LOADING TO_TRUCK UNLOADING RETURNING
        self.pallets_delivered = 0
        self.pallet_cycle_start = None
        self.pallet_cycles = []
        self.agv_wait_total = 0.0
        self.agv_arrived = None
        self.last_stray_check = 0.0
        self.arm_wait_pallet = 0.0
        self.amr_busy = 0.0                         # s the AMR was not idle (utilisation KPI)
        self.last_state_t = None
        self.t_start = None
        self.logs = []
        self.stop_evt = threading.Event()
        self.started = False

        # ---------------- Gazebo ----------------
        self.gz = GzNode()
        self.spare_nodes = []
        opts = SubscribeOptions()
        opts.msgs_per_sec = 10                      # boxes are slow; more only costs CPU
        self.gz.subscribe(Pose_V, f'/world/{WORLD}/dynamic_pose/info', self._on_poses, opts)
        self.belt_pub = {n: self.gz.advertise(f'/model/conveyor_{n}/link/belt_link/track_cmd_vel', Double)
                         for n in self.lanes}
        self.attach_pub, self.detach_pub = {}, {}
        self.weld_pub, self.unweld_pub = {}, {}     # pallet <-> box weld (see world_gen.pallet_joints_sdf)
        for ln in self.lanes.values():
            for b in ln.boxes:
                self.attach_pub[b] = self.gz.advertise(f'/vacuum/{b}/attach', Empty)
                self.detach_pub[b] = self.gz.advertise(f'/vacuum/{b}/detach', Empty)
                self.weld_pub[b] = self.gz.advertise(f'/pallet/{b}/attach', Empty)
                self.unweld_pub[b] = self.gz.advertise(f'/pallet/{b}/detach', Empty)
        self.tgrip_on = {b: self.gz.advertise(f'/tgrip/{b}/attach', Empty) for b in self.box_lane}
        self.tgrip_off = {b: self.gz.advertise(f'/tgrip/{b}/detach', Empty) for b in self.box_lane}
        self.welded = set()

        # ---------------- ROS ----------------
        self.create_subscription(JointState, '/joint_states', self._on_joints, 20, callback_group=self.cb)
        self.create_subscription(String, '/amr/state', self._on_amr, 10, callback_group=self.cb)
        self.amr_cmd = self.create_publisher(String, '/amr/cmd', 20)
        self.create_subscription(String, '/amr/reply', self._on_amr_reply, 20, callback_group=self.cb)
        self.create_subscription(String, '/plant/cmd', self._on_cmd, 20, callback_group=self.cb)
        self.reply_pub = self.create_publisher(String, '/plant/reply', 20)
        self.state_pub = self.create_publisher(String, '/plant/state', 10)
        self.log_pub = self.create_publisher(String, '/plant/log', 100)
        self.ik = self.create_client(GetPositionIK, '/compute_ik', callback_group=self.cb)
        self.planner = self.create_client(GetMotionPlan, '/plan_kinematic_path', callback_group=self.cb)
        self.scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene', callback_group=self.cb)
        self.fjt = {n: ActionClient(self, FollowJointTrajectory, f'/{ln.p}_controller/follow_joint_trajectory',
                                    callback_group=self.cb) for n, ln in self.arms.items()}
        self.amr_replies = {}
        self.amr_seq = 0
        self.amr_rx = 0                             # /amr/state messages received
        self.create_timer(0.25, self._publish_state, callback_group=self.cb)
        self.create_timer(0.1, self._tick, callback_group=self.cb)
        threading.Thread(target=self._boot, daemon=True).start()

    # ================================================================ geometry
    def _make_slots(self):
        """Per lane: 8 slot offsets (pallet frame) in placing order - layer by layer, far side first."""
        h = self.prod['box']['size'][2]
        out = {}
        for n, ln in self.lanes.items():
            qx, qy = ln.quadrant
            slots = []
            for layer in range(self.prod['layers']):
                cells = [(x, y) for x in self.prod['pattern_x'] for y in self.prod['pattern_y']
                         if (x > 0) == (qx > 0) and (y > 0) == (qy > 0)]
                cells.sort(key=lambda c: -(abs(c[0]) + abs(c[1])))       # outer corner first
                for x, y in cells:
                    slots.append((x, y, h * (layer + 1)))              # top of the placed box above pallet deck
            out[n] = slots
        return out

    def pallet_pose(self):
        with self.lock:
            return self.poses.get('pallet')

    def pallet_on_station(self):
        p = self.pallet_pose()
        return p is not None and math.hypot(p[0], p[1]) < 0.08 and abs(p[2] - 0.788) < 0.03

    def slot_world(self, lane, i):
        """TCP target for slot i: box top + 3 mm, in the world frame (follows the real pallet pose)."""
        px, py, pz, pyaw = self.pallet_pose()
        x, y, top = self.slots[lane][i]
        c, s = math.cos(pyaw), math.sin(pyaw)
        return (px + c * x - s * y, py + s * x + c * y, pz + 0.04 + top + 0.003)        # pallet deck = centre + 40 mm

    def pick_world(self, ln):
        x, y = ln.geo['pick']
        return (x, y, self.prod['conveyor_top'] + self.prod['box']['size'][2] + 0.003)

    # ================================================================ callbacks
    def _on_poses(self, msg):
        try:
            with self.lock:
                for p in msg.pose:
                    if p.name.startswith('box_') or p.name in ('pallet', 'ld60'):
                        self.poses[p.name] = (p.position.x, p.position.y, p.position.z, yaw_of(p.orientation))
                self.pose_rx += 1
        except Exception:  # noqa: BLE001 - never raise into gz-transport threads
            pass

    def _on_joints(self, msg):
        for n, v in zip(msg.name, msg.position):
            self.joint_pos[n] = v

    def _on_amr(self, msg):
        try:
            self.amr = json.loads(msg.data)
            self.amr_rx += 1
        except ValueError:
            pass

    def _on_amr_reply(self, msg):
        try:
            r = json.loads(msg.data)
            self.amr_replies[r['id']] = r
        except (ValueError, KeyError):
            pass

    def shutdown_gz(self):
        for n in self.lanes:
            self.belt_pub[n].publish(Double(data=0.0))
        try:
            self.gz.unsubscribe(f'/world/{WORLD}/dynamic_pose/info')
        except Exception:  # noqa: BLE001
            pass

    # ================================================================ logging
    def log(self, level, msg):
        e = {'t': time.strftime('%H:%M:%S'), 'level': level, 'msg': msg}
        with self.lock:
            self.logs.append(e)
            self.logs = self.logs[-300:]
        if level == 'error':
            self.get_logger().error(msg)
        elif level == 'warn':
            self.get_logger().warning(msg)
        else:
            self.get_logger().info(msg)
        self.log_pub.publish(String(data=json.dumps(e)))

    # ================================================================ boot
    def _boot(self):
        try:
            for name, cli in [('IK', self.ik), ('planner', self.planner), ('scene', self.scene)]:
                while not cli.wait_for_service(timeout_sec=2.0):
                    self.get_logger().info(f'waiting for MoveIt {name}...')
            for n, ac in self.fjt.items():
                while not ac.wait_for_server(timeout_sec=2.0):
                    self.get_logger().info(f'waiting for {n} controller...')
            while not self.joint_pos or not self.poses.get('pallet'):
                time.sleep(0.2)
            # gz-transport drops messages sent before a subscriber is discovered: wait for all grip topics
            # (an announcement can get lost among hundreds of topics: re-advertise what is still unconnected)
            kinds = [(self.attach_pub, '/vacuum/{}/attach'), (self.detach_pub, '/vacuum/{}/detach'),
                     (self.weld_pub, '/pallet/{}/attach'), (self.unweld_pub, '/pallet/{}/detach'),
                     (self.tgrip_on, '/tgrip/{}/attach'), (self.tgrip_off, '/tgrip/{}/detach')]
            missing = []
            for attempt in range(6):
                t0 = time.monotonic()
                while time.monotonic() - t0 < 5 and \
                        not all(p.has_connections() for d, _ in kinds for p in d.values()):
                    time.sleep(0.2)
                missing = [(d, fmt, b) for d, fmt in kinds for b, p in d.items() if not p.has_connections()]
                if not missing:
                    break
                spare = GzNode()                    # a node cannot advertise the same topic twice
                self.spare_nodes.append(spare)
                for d, fmt, b in missing:
                    d[b] = spare.advertise(fmt.format(b), Empty)
            if missing:
                self.log('warn', f'{len(missing)} gripper topics not connected to Gazebo: '
                                 + ', '.join(fmt.format(b) for _, fmt, b in missing[:6]))
            self._load_scene()
            # (sim_boot has released every joint and confirmed it on the joint's state topic)
            self._recycle([b for b in self.box_lane if not self._in_storage(b)])
            with self.lock:
                self.status = 'READY'
            self.log('info', 'Plant ready: 4 cells, AMR logistics automatic. Press Start production.')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(traceback.format_exc())
            with self.lock:
                self.status, self.error = 'ERROR', f'boot failed: {e}'

    def _box(self, oid, size, xyz, yaw=0.0):
        co = CollisionObject(id=oid, operation=CollisionObject.ADD)
        co.header.frame_id = 'world'
        co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size]))
        co.primitive_poses.append(Pose(position=Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])),
                                      orientation=quat_from_rpy(0.0, 0.0, float(yaw))))
        return co

    def _load_scene(self):
        objs = [self._box('floor', (8, 12, 0.02), (0, 0, -0.012)),
                self._box('pallet_station', (1.0, 0.5, 0.75), (0, 0, 0.375)),
                # keep-out: AMR docked at the station (roller top + some margin)
                self._box('amr_dock_keepout', (0.9, 0.7, 0.95), (0.95, 0, 0.475))]
        top = self.prod['conveyor_top']
        for n, ln in self.lanes.items():
            g = ln.geo
            objs.append(self._box(f'conveyor_{n}', (g['length'] + 0.02, 0.32, top), (g['cx'], g['cy'], top / 2), g['yaw']))
            ux, uy = g['u']
            mx, my = g['cx'] - ux * (g['length'] / 2 + 0.45), g['cy'] - uy * (g['length'] / 2 + 0.45)
            objs.append(self._box(f'machine_{n}', (0.8, 0.9, 1.5), (mx, my, 0.75), g['yaw']))
        # truck: station, AMR docked at it, cargo walls, roof
        tx, ty = self.truck.cell['pallet']
        objs += [self._box('truck_station', (1.0, 0.5, 0.75), (tx, ty, 0.375)),
                 self._box('truck_dock_keepout', (0.9, 0.7, 0.95), (12.33, 0, 0.475)),
                 self._box('truck_floor', (6.8, 2.4, 0.02), (15.82, 0, -0.012)),
                 self._box('truck_wall_l', (6.8, 0.07, 2.7), (15.82, 1.235, 1.35)),
                 self._box('truck_wall_r', (6.8, 0.07, 2.7), (15.82, -1.235, 1.35)),
                 self._box('truck_roof', (6.87, 2.54, 0.07), (15.82, 0, 2.735))]
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = objs
        res = self._call(self.scene, ApplyPlanningScene.Request(scene=scene), 10.0)
        if res is None or not res.success:
            raise PlantError('could not load the planning scene')
        self.log('info', f'Planning scene: {len(objs)} fixtures (stations, conveyors, machines, truck, AMR keep-outs)')

    def _scene_rows(self, add, rows):
        """Truck stack rows as collision objects, so the truck robot plans around them."""
        bx, by, bz = self.prod['box']['size']
        objs = []
        for i, pos in rows:
            if add:
                x, y, top = pos
                objs.append(self._box(f'truck_row_{i}', (by + 0.004, 4 * 0.145, bz), (x, y, top - 0.003 - bz / 2)))
            else:
                objs.append(CollisionObject(id=f'truck_row_{i}', operation=CollisionObject.REMOVE))
                objs[-1].header.frame_id = 'world'
        if objs:
            scene = PlanningScene(is_diff=True)
            scene.world.collision_objects = objs
            self._call(self.scene, ApplyPlanningScene.Request(scene=scene), 10.0)

    # ================================================================ box bookkeeping
    def _in_storage(self, b):
        p = self.poses.get(b)
        return p is not None and p[0] < -20

    def _on_conveyor(self, b, ln):
        p = self.poses.get(b)
        if p is None:
            return False
        g = ln.geo
        dx, dy = p[0] - g['cx'], p[1] - g['cy']
        along = dx * math.cos(g['yaw']) + dy * math.sin(g['yaw'])
        across = -dx * math.sin(g['yaw']) + dy * math.cos(g['yaw'])
        zc = self.prod['conveyor_top'] + self.prod['box']['size'][2] / 2
        return abs(along) <= g['length'] / 2 + 0.05 and abs(across) < 0.15 and abs(p[2] - zc) < 0.04

    def _at_pick(self, ln):
        """Photo-eye at the end stop: the box (not held) sitting at the pick point."""
        px, py = ln.geo['pick']
        zc = self.prod['conveyor_top'] + self.prod['box']['size'][2] / 2
        for b in ln.boxes:
            p = self.poses.get(b)
            if p and b != ln.held and math.hypot(p[0] - px, p[1] - py) < 0.03 and abs(p[2] - zc) < 0.03:
                return b
        return None

    def _on_pallet(self, b):
        p, pl = self.poses.get(b), self.pallet_pose()
        if p is None or pl is None:
            return False
        c, s = math.cos(-pl[3]), math.sin(-pl[3])
        dx, dy = p[0] - pl[0], p[1] - pl[1]
        lx, ly = c * dx - s * dy, s * dx + c * dy
        return abs(lx) < 0.32 and abs(ly) < 0.25 and p[2] > pl[2] + 0.05

    def pallet_fill(self):
        with self.lock:
            return sum(1 for b in self.box_lane if self._on_pallet(b))

    def _gz_service(self, args, timeout=10):
        try:
            return subprocess.run(['gz', 'service'] + args, capture_output=True, text=True, timeout=timeout).stdout
        except (OSError, subprocess.TimeoutExpired) as e:
            return str(e)

    def _set_poses(self, items):
        """Teleport several boxes in one request: [(name, (x, y, z)), ...]."""
        if not items:
            return True
        req = ' '.join(f'pose: {{name: "{n}" position: {{x: {x:.4f} y: {y:.4f} z: {z:.4f}}} orientation: {{w: 1}}}}'
                       for n, (x, y, z) in items)
        out = self._gz_service(['-s', f'/world/{WORLD}/set_pose_vector', '--reqtype', 'gz.msgs.Pose_V',
                                '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', req])
        return 'data: true' in out

    def weld(self, b):
        self.weld_pub[b].publish(Empty())
        self.welded.add(b)

    def unweld(self, names):
        names = [b for b in names if b in self.welded]
        for b in names:
            self.unweld_pub[b].publish(Empty())
            self.welded.discard(b)
        if names:
            self.sim_sleep(0.3)                         # let the joints go before the boxes are moved

    def _recycle(self, names):
        lane_index = {n: i for i, n in enumerate(self.lanes)}
        items = []
        for b in names:
            ln = self.box_lane[b]
            k = int(b.rsplit('_', 1)[1])
            items.append((b, storage_pose(lane_index[ln.name], k)))
        return self._set_poses(items)

    # ================================================================ machines + conveyors (10 Hz)
    def _tick(self):
        if self.status not in ('RUNNING', 'STOPPING', 'READY', 'ERROR'):    # ERROR: still clear dropped boxes
            return
        now = self.now()
        running = self.status == 'RUNNING'
        if now - self.last_stray_check > 2.0:
            self.last_stray_check = now
            with self.lock:
                held = {ln.held for ln in self.lanes.values()} | set(self.truck.held)
                in_truck = {b for row in self.truck.rows for b in row}
                strays = [b for b, p in self.poses.items() if b in self.box_lane and p[0] > -20 and p[2] < 0.6
                          and b not in held and b not in in_truck]
            if strays:
                self.log('warn', f'{len(strays)} box(es) on the floor -> back to the machines')
                threading.Thread(target=self._recycle, args=(strays,), daemon=True).start()
        for n, ln in self.lanes.items():
            with self.lock:
                at_end = ln.picking or self._at_pick(ln) is not None
                on_belt = [b for b in ln.boxes if self._on_conveyor(b, ln) and b != ln.held]
            run_belt = running and not at_end and bool(on_belt)
            speed = float(self.prod['conveyor_speed']) if run_belt else 0.0
            self.belt_pub[n].publish(Double(data=speed))
            ln.belt_running = run_belt
            if not running or len(on_belt) >= self.prod['buffer_max'] or now - ln.last_spawn < self.prod['machine_period']:
                continue
            sx, sy = ln.geo['spawn']
            with self.lock:
                clear = all(math.hypot(self.poses[b][0] - sx, self.poses[b][1] - sy) > 0.25
                            for b in on_belt if b in self.poses)
                free = [b for b in ln.boxes if self._in_storage(b) and b != ln.held]
            if not clear or not free:
                continue
            ln.last_spawn = now
            z = self.prod['conveyor_top'] + self.prod['box']['size'][2] / 2 + 0.004
            threading.Thread(target=self._spawn, args=(ln, free[0], (sx, sy, z)), daemon=True).start()

    def _spawn(self, ln, b, xyz):
        if self._set_poses([(b, xyz)]):
            ln.made += 1

    # ================================================================ arm motion
    def now(self):
        """Simulation time (s): plant timings and KPIs follow the simulation, not a slow host."""
        return self.get_clock().now().nanoseconds * 1e-9

    def fresh_poses(self, n=2):
        """Wait for n new pose messages: on a slow host they arrive ~0.5 s apart (they follow sim time)."""
        rx0 = self.pose_rx
        end = time.monotonic() + 30
        while self.pose_rx < rx0 + n and time.monotonic() < end:
            self._check()
            time.sleep(0.02)

    def sim_sleep(self, seconds):
        end = self.now() + seconds
        while self.now() < end:
            time.sleep(0.05)

    def _check(self):
        if self.stop_evt.is_set():
            raise Stopped()

    def _call(self, client, req, timeout):
        fut = client.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done():
            if time.monotonic() > end:
                fut.cancel()
                return None
            time.sleep(0.01)
        return fut.result()

    def _q(self, ln):
        return [self.joint_pos.get(j, 0.0) for j in ln.joints]

    def _ik(self, ln, xyz, yaw=None):
        cur = self._q(ln)
        best, best_score = None, None
        base_x, base_y = ln.cell['arm']
        byaw = math.radians(ln.cell['arm_yaw'])
        az = math.atan2(xyz[1] - base_y, xyz[0] - base_x) - byaw
        seeds = [cur]
        for extra in (0.0, 2 * math.pi, -2 * math.pi):
            s = list(cur)
            s[0] = az + extra
            seeds.append(s)
        for seed in seeds:
            req = GetPositionIK.Request()
            r = req.ik_request
            r.group_name = ln.p
            r.ik_link_name = ln.tcp_link
            r.avoid_collisions = True
            r.timeout = Duration(sec=0, nanosec=200_000_000)
            r.robot_state.joint_state = JointState(name=ln.joints, position=[float(v) for v in seed])
            ps = PoseStamped()
            ps.header.frame_id = 'world'
            ps.pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
            ps.pose.orientation = quat_from_rpy(math.pi, 0.0, ln.tcp_yaw if yaw is None else yaw)
            r.pose_stamped = ps
            res = self._call(self.ik, req, 5.0)
            if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
                continue
            pos = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            sol = [pos[j] for j in ln.joints]
            sol = [min((q + k * 2 * math.pi for k in range(-2, 3) if -2 * math.pi <= q + k * 2 * math.pi <= 2 * math.pi),
                       key=lambda c, s0=c0: abs(c - s0) + 0.1 * abs(c)) for q, c0 in zip(sol, cur)]
            if sol[2] < 0:                                     # keep elbow up
                continue
            score = sum(abs(a - b) for a, b in zip(sol, cur))
            if best is None or score < best_score:
                best, best_score = sol, score
        if best is None:
            raise PlantError(f'{ln.name}: no collision-free IK for ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})')
        return best

    def _plan(self, ln, planner, goal, scale):
        req = GetMotionPlan.Request()
        m = req.motion_plan_request
        m.group_name = ln.p
        m.pipeline_id = PIPELINE
        m.planner_id = planner
        m.num_planning_attempts = 1
        m.allowed_planning_time = 5.0
        m.max_velocity_scaling_factor = scale
        m.max_acceleration_scaling_factor = scale
        m.start_state = RobotState(is_diff=True)
        m.goal_constraints = [goal]
        m.workspace_parameters.header.frame_id = 'world'
        m.workspace_parameters.min_corner = Vector3(x=-6.0, y=-8.0, z=-0.5)
        m.workspace_parameters.max_corner = Vector3(x=20.0, y=8.0, z=3.0)
        for _ in range(3):
            res = self._call(self.planner, req, 15.0)
            if res is None:
                raise PlantError(f'{ln.name}: planner timeout')
            code = res.motion_plan_response.error_code.val
            if code == MoveItErrorCodes.SUCCESS:
                return res.motion_plan_response.trajectory.joint_trajectory
            if code not in (MoveItErrorCodes.PLANNING_FAILED, MoveItErrorCodes.FAILURE) or \
                    m.max_velocity_scaling_factor <= 0.1:
                break
            # Pilz LIN near a wrist/shoulder turn can exceed a joint limit: retry slower
            m.max_velocity_scaling_factor = max(0.1, m.max_velocity_scaling_factor * 0.5)
            m.max_acceleration_scaling_factor = max(0.1, m.max_acceleration_scaling_factor * 0.5)
        raise PlantError(f'{ln.name}: {planner} planning failed ({ERRORS.get(code, code)})')

    def _execute(self, ln, traj):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.trajectory.header.stamp.sec = 0
        goal.trajectory.header.stamp.nanosec = 0
        send = self.fjt[ln.name].send_goal_async(goal)
        end = time.monotonic() + 30
        while not send.done():
            if time.monotonic() > end:
                raise PlantError(f'{ln.name}: controller did not accept the trajectory')
            time.sleep(0.01)
        h = send.result()
        if not h.accepted:
            raise PlantError(f'{ln.name}: trajectory rejected')
        res = h.get_result_async()
        dur = traj.points[-1].time_from_start.sec + traj.points[-1].time_from_start.nanosec * 1e-9 if traj.points else 0
        # timeout in simulation time (a slow or paused host must not trip it) + a wall-clock cap for a frozen sim
        sim_end = self.get_clock().now().nanoseconds * 1e-9 + dur * 2 + 5
        wall_end = time.monotonic() + max(300.0, dur * 20)
        while not res.done():
            if self.stop_evt.is_set():
                h.cancel_goal_async()
                raise Stopped()
            if self.get_clock().now().nanoseconds * 1e-9 > sim_end or time.monotonic() > wall_end:
                h.cancel_goal_async()
                raise PlantError(f'{ln.name}: motion timeout')
            time.sleep(0.02)
        r = res.result().result
        if r.error_code != 0:
            raise PlantError(f'{ln.name}: controller error {r.error_code} {r.error_string}')

    def move_ptp(self, ln, xyz=None, joints=None, speed=1.0, yaw=None):
        self._check()
        q = joints if joints is not None else self._ik(ln, xyz, yaw)
        c = Constraints()
        for n, v in zip(ln.joints, q):
            c.joint_constraints.append(JointConstraint(joint_name=n, position=float(v), tolerance_above=1e-4,
                                                       tolerance_below=1e-4, weight=1.0))
        t0 = self.now()
        self._execute(ln, self._plan(ln, 'PTP', c, max(0.05, min(1.0, self.arm_speed * speed))))
        ln.busy_time += self.now() - t0

    def move_lin(self, ln, xyz, speed=0.5, yaw=None):
        self._check()
        c = Constraints()
        pc = PositionConstraint(link_name=ln.tcp_link, weight=1.0)
        pc.header.frame_id = 'world'
        bv = BoundingVolume()
        bv.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[1e-4]))
        bv.primitive_poses.append(Pose(position=Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])),
                                       orientation=Quaternion(w=1.0)))
        pc.constraint_region = bv
        oc = OrientationConstraint(link_name=ln.tcp_link,
                                   orientation=quat_from_rpy(math.pi, 0.0, ln.tcp_yaw if yaw is None else yaw),
                                   absolute_x_axis_tolerance=1e-3, absolute_y_axis_tolerance=1e-3,
                                   absolute_z_axis_tolerance=1e-3, weight=1.0)
        oc.header.frame_id = 'world'
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        t0 = self.now()
        self._execute(ln, self._plan(ln, 'LIN', c, max(0.05, min(1.0, self.arm_speed * speed))))
        ln.busy_time += self.now() - t0

    # ================================================================ arm worker
    def _arm_loop(self, ln):
        nominal = self.pick_world(ln)
        pick_up = (nominal[0], nominal[1], nominal[2] + 0.18)
        try:
            ln.status, ln.task = 'MOVING', 'Home'
            self.move_ptp(ln, joints=self.homes[ln.name])
            while not self.stop_evt.is_set():
                # ---- 1. wait for a box at the end of the conveyor
                ln.status, ln.task = 'WAIT_BOX', 'Waiting for a box'
                while True:
                    self._check()
                    with self.lock:
                        b = self._at_pick(ln)
                    if b and not self.poses_moving(b):
                        break
                    time.sleep(0.1)
                # ---- 2. pick
                ln.status, ln.task = 'PICK', f'Picking {b}'
                with self.lock:
                    bx, by = self.poses[b][:2]          # photo-eye: centre the cup on the real box
                pick = (bx, by, nominal[2])
                pick_up = (bx, by, nominal[2] + 0.18)
                self.move_ptp(ln, pick_up)
                for attempt in (1, 2):                  # one automatic re-pick, like a real cell
                    ln.picking = True
                    try:
                        self.move_lin(ln, pick, 0.4)
                        ln.held = b
                        self.attach_pub[b].publish(Empty())
                        self.sim_sleep(0.3)
                        self.move_lin(ln, pick_up, 0.6)
                    finally:
                        ln.picking = False
                    self.fresh_poses()
                    bp = self.poses.get(b)
                    if bp is not None and bp[2] > pick[2] - 0.02:
                        break
                    self.detach_pub[b].publish(Empty())
                    ln.held = None
                    if attempt == 2:
                        raise PlantError(f'{ln.name}: vacuum lost {b} twice (missed pick)')
                    self.log('warn', f'{ln.name}: vacuum missed {b}, picking again')
                placed = False
                while not placed:
                    # ---- 3. wait until this arm may place on the pallet
                    ln.status, ln.task = 'WAIT_PALLET', 'Waiting for the pallet'
                    t_wait = self.now()
                    while not self._pallet_ready_for(ln):
                        self._check()
                        time.sleep(0.1)
                    self.arm_wait_pallet += self.now() - t_wait
                    # ---- 4. pallet zone interlock
                    ln.status, ln.task = 'WAIT_ZONE', 'Waiting for the pallet zone'
                    if not self._zone_enter(ln):              # pallet left meanwhile
                        continue
                    try:
                        self._place(ln, b, pick_up)
                        placed = True
                    finally:
                        self._zone_leave(ln)
                if self.pallet_cycle_start is None:
                    self.pallet_cycle_start = self.now()
        except Stopped:
            pass
        except PlantError as e:
            self._fail(str(e))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(traceback.format_exc())
            self._fail(f'{ln.name}: internal error {e}')
        finally:
            if ln.held:
                self.detach_pub[ln.held].publish(Empty())
                ln.held = None
            ln.status, ln.task = 'STOPPED', ''

    def _place(self, ln, b, pick_up):
        i = ln.placed
        slot = self.slot_world(ln.name, i)
        slot_up = (slot[0], slot[1], slot[2] + 0.15)
        ln.status, ln.task = 'PLACE', f'Placing {b} in slot {i + 1}/{len(self.slots[ln.name])}'
        self.move_ptp(ln, slot_up)
        self.move_lin(ln, slot, 0.35)
        self.detach_pub[b].publish(Empty())
        ln.held = None
        self.sim_sleep(0.25)
        self.weld(b)
        self.move_lin(ln, slot_up, 0.6)
        ln.placed += 1
        ln.palletized += 1
        self.move_ptp(ln, pick_up)                   # leave the zone before releasing it

    def _adjacent(self, a, b):
        qa, qb = self.lanes[a].quadrant, self.lanes[b].quadrant
        return (qa[0] != qb[0]) + (qa[1] != qb[1]) == 1

    def _zone_enter(self, ln):
        """Block until no neighbouring arm works above the pallet and no neighbour queued earlier.
        Returns False when the pallet is no longer available (checked atomically with the entry)."""
        with self.zone_cv:
            self.zone_ticket += 1
            self.zone_queue[ln.name] = self.zone_ticket
            try:
                while True:
                    if self.stop_evt.is_set():
                        raise Stopped()
                    if not self._pallet_ready_for(ln):
                        return False
                    mine = self.zone_queue[ln.name]
                    if not any(self._adjacent(ln.name, o) for o in self.zone_active) and \
                            not any(self._adjacent(ln.name, o) and t < mine for o, t in self.zone_queue.items()):
                        self.zone_active.add(ln.name)
                        return True
                    self.zone_cv.wait(0.1)
            finally:
                self.zone_queue.pop(ln.name, None)
                self.zone_cv.notify_all()

    def _zone_leave(self, ln):
        with self.zone_cv:
            self.zone_active.discard(ln.name)
            self.zone_cv.notify_all()

    def _zone_close(self):
        """Logistics takes the pallet: no new arm may enter, wait for the ones above it to leave."""
        with self.zone_cv:
            self.logistics = 'LOADING'
            while self.zone_active:
                self._check()
                self.zone_cv.wait(0.1)

    def poses_moving(self, b):
        """A box still sliding against the end stop is not ready to pick."""
        p0 = self.poses.get(b)
        self.fresh_poses()
        p1 = self.poses.get(b)
        return p0 is None or p1 is None or math.hypot(p1[0] - p0[0], p1[1] - p0[1]) > 0.002

    def _pallet_ready_for(self, ln):
        if ln.placed >= len(self.slots[ln.name]):
            return False
        if self.logistics not in ('IDLE', 'DISPATCHED'):
            return False
        amr = self.amr or {}
        if amr.get('status') == 'TRANSFER':
            return False
        return self.pallet_on_station()

    def _fail(self, msg):
        with self.lock:
            if self.status == 'ERROR':
                return
            self.status = 'ERROR'
            self.error = msg
        self.log('error', msg)
        self.stop_evt.set()

    # ================================================================ logistics
    def amr_send(self, cmd, **kw):
        self.amr_seq += 1
        rid = f'p{self.amr_seq}'
        self.amr_cmd.publish(String(data=json.dumps({'id': rid, 'cmd': cmd, **kw})))
        end = time.monotonic() + 30
        while rid not in self.amr_replies and time.monotonic() < end:
            time.sleep(0.02)
        r = self.amr_replies.pop(rid, {'ok': False, 'msg': 'AMR did not answer'})
        if not r['ok']:
            raise PlantError(f'AMR refused {cmd}: {r["msg"]}')

    def amr_wait_idle(self, what, timeout=300):
        # judge only fresh states: on a slow host the last one can predate the command just sent
        rx0 = self.amr_rx
        while self.amr_rx < rx0 + 2:
            self._check()
            time.sleep(0.05)
        end = self.now() + timeout
        while self.now() < end:
            self._check()
            st = (self.amr or {}).get('status')
            if st == 'IDLE' and not (self.amr or {}).get('queue'):
                return
            if st in ('ERROR', 'ESTOP'):
                raise PlantError(f'AMR {st} during "{what}": {(self.amr or {}).get("error")}')
            time.sleep(0.1)
        raise PlantError(f'AMR timeout during "{what}"')

    def _logistics_loop(self):
        total = sum(len(s) for s in self.slots.values())
        try:
            while not self.stop_evt.is_set():
                time.sleep(0.2)
                if not self.auto_logistics:
                    continue
                fill = sum(ln.placed for ln in self.lanes.values())
                amr = self.amr or {}
                if self.logistics == 'IDLE' and fill >= self.prod['dispatch_at'] and amr.get('at_goal') != 'PalletStation':
                    self.logistics = 'DISPATCHED'
                    self.log('info', f'Pallet at {fill}/{total}: dispatching the LD-60 to the station')
                    self.amr_send('goto', goal='PalletStation')
                    self.amr_wait_idle('drive to station')
                    self.agv_arrived = self.now()
                if self.logistics in ('IDLE', 'DISPATCHED') and fill >= total:
                    if (self.amr or {}).get('at_goal') != 'PalletStation':
                        self.logistics = 'DISPATCHED'
                        self.amr_send('goto', goal='PalletStation')
                        self.amr_wait_idle('drive to station')
                    self._zone_close()                    # last arms leave the pallet zone
                    if self.agv_arrived is not None:              # AMR waited for a full pallet
                        self.agv_wait_total += self.now() - self.agv_arrived
                        self.agv_arrived = None
                    self.amr_send('load')
                    self.amr_wait_idle('load pallet')
                    self.logistics = 'TO_TRUCK'
                    self.amr_send('goto', goal='TruckDock')
                    self.amr_wait_idle('drive to truck')
                    self.amr_send('unload')
                    self.amr_wait_idle('unload pallet')
                    self.logistics = 'UNLOADING'
                    self._unload_truck()
                    self.logistics = 'RETURNING'
                    self.amr_send('load')
                    self.amr_wait_idle('take empty pallet')
                    self.amr_send('goto', goal='PalletStation')
                    self.amr_wait_idle('return to station')
                    self.amr_send('unload')
                    self.amr_wait_idle('return pallet')
                    for ln in self.lanes.values():
                        ln.placed = 0
                    self.pallets_delivered += 1
                    if self.pallet_cycle_start is not None:
                        dt = self.now() - self.pallet_cycle_start
                        self.pallet_cycles.append(round(dt, 1))
                        self.pallet_cycles = self.pallet_cycles[-20:]
                    self.pallet_cycle_start = None
                    self.logistics = 'IDLE'
                    self.log('info', f'Pallet #{self.pallets_delivered} delivered ({total} boxes). Empty pallet back.')
                    battery = (self.amr or {}).get('battery', 100)
                    self.amr_send('charge' if battery < 50 else 'goto', **({} if battery < 50 else {'goal': 'Home'}))
        except Stopped:
            pass
        except PlantError as e:
            self._fail(str(e))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(traceback.format_exc())
            self._fail(f'logistics internal error {e}')

    # ================================================================ truck robot
    def _next_row(self):
        """Top layer first: the 4 boxes of one pallet row (same layer, same pallet-frame y)."""
        pl = self.pallet_pose()
        with self.lock:
            on = [b for b in self.box_lane if self._on_pallet(b)]
            if not on:
                return None, None
            ztop = max(self.poses[b][2] for b in on)
            top = [b for b in on if self.poses[b][2] > ztop - 0.03]
            c, s = math.cos(-pl[3]), math.sin(-pl[3])
            rows = {}
            for b in top:
                dx, dy = self.poses[b][0] - pl[0], self.poses[b][1] - pl[1]
                ly = s * dx + c * dy
                key = min(self.prod['pattern_y'], key=lambda v, ly=ly: abs(v - ly))
                rows.setdefault(key, []).append(b)
            key = max(rows, key=lambda k: (len(rows[k]), abs(k)))      # full rows, outer first
            row = rows[key]
            pts = [self.poses[b] for b in row]
        x = sum(p[0] for p in pts) / len(pts)
        y = sum(p[1] for p in pts) / len(pts)
        z = max(p[2] for p in pts) + self.prod['box']['size'][2] / 2 + 0.003
        yaw = math.atan2(math.sin(pl[3]), math.cos(pl[3]))                # bar along the pallet rows
        if yaw > math.pi / 2:
            yaw -= math.pi
        elif yaw <= -math.pi / 2:
            yaw += math.pi
        return row, (x, y, z, yaw)

    def _truck_ptp(self, t, xyz, yaw, speed=1.0):
        """PTP, through the home pose when the direct swing is not collision-free."""
        try:
            self.move_ptp(t, xyz, yaw=yaw, speed=speed)
        except PlantError as e:
            if 'planning failed' not in str(e) and 'IK' not in str(e):
                raise
            self.move_ptp(t, joints=self.homes[t.name], speed=speed)
            self.move_ptp(t, xyz, yaw=yaw, speed=speed)

    def _unload_truck(self):
        """Truck robot: pallet row by row into the truck stack (4 boxes per move)."""
        t = self.truck
        t0 = self.now()
        done = False
        try:
            if not t.homed:
                t.status, t.task = 'MOVING', 'Home'
                self.move_ptp(t, joints=self.homes[t.name])
                t.homed = True
            while True:
                self._check()
                row, tgt = self._next_row()
                if row is None:
                    break
                if t.truck_state != 'DOCKED' or len(t.rows) >= len(t.slots):
                    t.status, t.task = 'WAIT_TRUCK', 'Waiting for an empty truck'
                    while t.truck_state != 'DOCKED' or len(t.rows) >= len(t.slots):
                        self._check()
                        time.sleep(0.2)
                x, y, z, yaw = tgt
                up = (x, y, z + 0.15)
                t.status, t.task = 'PICK', f'Taking a row of {len(row)} off the pallet'
                self._truck_ptp(t, up, yaw)
                self.unweld(row)
                self.move_lin(t, (x, y, z), 0.5, yaw=yaw)
                t.held = list(row)
                for b in row:
                    self.tgrip_on[b].publish(Empty())
                self.sim_sleep(0.35)
                self.move_lin(t, up, 0.5, yaw=yaw)
                # home first: high above the pallet edge, so the held row never sweeps over the
                # remaining rows; from there the swing to the stack goes away from the pallet
                self.move_ptp(t, joints=self.homes[t.name], speed=0.7)
                self.fresh_poses()
                with self.lock:
                    lost = [b for b in row if self.poses.get(b, (0, 0, 0))[2] < z - 0.02]
                if lost:
                    raise PlantError(f'{t.name}: gripper lost {", ".join(lost)}')
                i = len(t.rows)
                sx, sy, sz = t.slots[i]
                syaw = yaw + math.pi / 2
                t.status, t.task = 'PLACE', f'Stacking row {i + 1}/{len(t.slots)} in the truck'
                self._truck_ptp(t, (sx, sy, 0.62), syaw, speed=0.7)
                self.move_lin(t, (sx, sy, sz), 0.5, yaw=syaw)
                t.rows.append(list(row))                 # in the truck before it leaves the gripper
                t.boxes_loaded += len(row)
                for b in row:
                    self.tgrip_off[b].publish(Empty())
                t.held = []
                self.sim_sleep(0.25)
                self.move_lin(t, (sx, sy, 0.62), 0.8, yaw=syaw)
                self._scene_rows(True, [(i, (sx, sy, sz))])
                if len(t.rows) >= len(t.slots):
                    threading.Thread(target=self._truck_depart, daemon=True).start()
            t.status, t.task = 'MOVING', 'Home'
            self.move_ptp(t, joints=self.homes[t.name])
            t.status, t.task = 'IDLE', ''
            done = True
        finally:
            if t.held:
                for b in t.held:
                    self.tgrip_off[b].publish(Empty())
                t.held = []
            if done:
                self.log('info', f'Truck robot: pallet unloaded in {self.now() - t0:.0f} s '
                                 f'(truck {len(t.rows)}/{len(t.slots)} rows)')

    def _truck_depart(self):
        """Full truck leaves (boxes go back to the machines), an empty one docks."""
        t = self.truck
        t.truck_state = 'DEPARTING'
        try:
            swap = float(self.cfg['truck_robot']['truck_swap_time'])
            boxes = [b for row in t.rows for b in row]
            self.log('info', f'Truck #{t.trucks_departed + 1} full ({len(boxes)} boxes): departing')
            self.sim_sleep(swap / 2)
            self._scene_rows(False, [(i, None) for i in range(len(t.rows))])
            self._recycle(boxes)                    # before forgetting the rows: else they look like strays
            t.rows = []
            t.trucks_departed += 1
            self.sim_sleep(swap / 2)
            t.truck_state = 'DOCKED'
            self.log('info', f'Empty truck #{t.trucks_departed + 1} docked')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(traceback.format_exc())
            self._fail(f'truck change failed: {e}')

    # ================================================================ commands
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
        self.reply_pub.publish(String(data=json.dumps({'id': rid, 'ok': ok, 'msg': text})))

    def _handle(self, cmd, req):
        if cmd == 'start':
            if self.status not in ('READY',):
                return False, f'Plant is {self.status}'
            if any(ln.thread and ln.thread.is_alive() for ln in self.lanes.values()):
                return False, 'Still stopping, try again'
            self.stop_evt.clear()
            with self.lock:
                self.status = 'RUNNING'
                if self.t_start is None:
                    self.t_start = self.now()
            for ln in self.lanes.values():
                ln.thread = threading.Thread(target=self._arm_loop, args=(ln,), daemon=True)
                ln.thread.start()
            threading.Thread(target=self._logistics_loop, daemon=True).start()
            self.log('info', 'Production started')
            return True, 'Production started'
        if cmd == 'stop':
            if self.status != 'RUNNING':
                return False, 'Not running'
            self.stop_evt.set()
            with self.lock:
                self.status = 'READY'
            self.log('info', 'Production stopped (arms hold, belts stop)')
            return True, 'Stopped'
        if cmd == 'reset':
            if self.status != 'ERROR':
                return False, 'Nothing to reset'
            self.stop_evt.set()
            with self.lock:
                self.status, self.error = 'READY', ''
            self.logistics = 'IDLE'
            self.log('info', 'Reset - check the cell, then Start again')
            return True, 'Reset'
        if cmd == 'clear':
            if self.status == 'RUNNING':
                return False, 'Stop production first'
            with self.lock:
                out = [b for b in self.box_lane if not self._in_storage(b)]
            for ln in self.lanes.values():
                if ln.held:
                    self.detach_pub[ln.held].publish(Empty())
                    ln.held = None
                ln.placed = 0
            t = self.truck
            for b in t.held:
                self.tgrip_off[b].publish(Empty())
            t.held = []
            self._scene_rows(False, [(i, None) for i in range(len(t.rows))])
            t.rows, t.truck_state = [], 'DOCKED'
            self.unweld(out)
            self._recycle(out)
            self.logistics = 'IDLE'
            self.pallet_cycle_start = None
            self.log('info', f'Cell cleared: {len(out)} boxes back to the machines, pallet count reset')
            return True, 'Cleared'
        if cmd == 'set_speed':
            v = float(req.get('value', 0))
            if not 0.1 <= v <= 1.0:
                return False, 'Arm speed must be 10..100 %'
            self.arm_speed = v
            return True, f'Arm speed {v * 100:.0f} %'
        if cmd == 'set_rate':
            v = float(req.get('value', 0))
            if not 2.0 <= v <= 60.0:
                return False, 'Machine period must be 2..60 s'
            self.prod['machine_period'] = v
            return True, f'Machines: 1 box every {v:.1f} s'
        if cmd == 'set_auto':
            self.auto_logistics = bool(req.get('value', True))
            return True, f'AMR logistics {"automatic" if self.auto_logistics else "manual"}'
        if cmd == 'fill_pallet':                    # test aid: a full pallet without waiting for the arms
            if self.status == 'RUNNING':
                return False, 'Stop production first'
            items = []
            for n, ln in self.lanes.items():
                with self.lock:
                    free = [b for b in ln.boxes if self._in_storage(b)]
                for i in range(ln.placed, len(self.slots[n])):
                    x, y, z = self.slot_world(n, i)
                    items.append((free.pop(0), (x, y, z - self.prod['box']['size'][2] / 2 - 0.002)))
                ln.placed = len(self.slots[n])
            self._set_poses(items)
            time.sleep(0.6)
            for b, _ in items:
                self.weld(b)
            return True, f'{len(items)} boxes put on the pallet'
        if cmd == 'get_logs':
            for e in list(self.logs)[-80:]:
                self.log_pub.publish(String(data=json.dumps(e)))
            return True, ''
        return False, f'Unknown command {cmd!r}'

    # ================================================================ state
    def _publish_state(self):
        now = self.now()
        elapsed = now - self.t_start if self.t_start else 0
        if self.last_state_t is not None and self.status == 'RUNNING' and \
                (self.amr or {}).get('status') not in (None, 'IDLE', 'CHARGING', 'ERROR', 'ESTOP'):
            self.amr_busy += now - self.last_state_t
        self.last_state_t = now
        t = self.truck
        with self.lock:
            boxes = []
            for b, ln in self.box_lane.items():
                p = self.poses.get(b)
                if p and p[0] > -20:
                    boxes.append([b, round(p[0], 3), round(p[1], 3), round(p[2], 3), ln.name])
            fill = sum(1 for b in self.box_lane if self._on_pallet(b))
        lanes = {}
        for n, ln in self.lanes.items():
            on_belt = sum(1 for b in ln.boxes if self._on_conveyor(b, ln) and b != ln.held)
            lanes[n] = {'status': ln.status, 'task': ln.task, 'placed': ln.placed, 'made': ln.made,
                        'palletized': ln.palletized, 'on_belt': on_belt, 'belt': ln.belt_running,
                        'held': ln.held, 'util': round(100 * ln.busy_time / elapsed, 1) if elapsed > 1 else 0.0,
                        'arm': ln.cell['arm'], 'pick': list(ln.geo['pick']),
                        'conveyor': [ln.cell['conveyor']['start'], ln.cell['conveyor']['end']], 'color': ln.cell['color']}
        st = {
            'status': self.status, 'error': self.error, 'logistics': self.logistics, 'auto': self.auto_logistics,
            'zone_owner': ', '.join(sorted(self.zone_active)) or None, 'arm_speed': self.arm_speed, 'machine_period': self.prod['machine_period'],
            'pallet_fill': fill, 'pallet_target': sum(len(s) for s in self.slots.values()),
            'placed_total': sum(ln.placed for ln in self.lanes.values()),
            'dispatch_at': self.prod['dispatch_at'],
            'pallets_delivered': self.pallets_delivered, 'pallet_cycles': self.pallet_cycles,
            'agv_wait_s': round(self.agv_wait_total, 1), 'arm_wait_pallet_s': round(self.arm_wait_pallet, 1),
            'boxes_made': sum(ln.made for ln in self.lanes.values()),
            'boxes_palletized': sum(ln.palletized for ln in self.lanes.values()),
            'throughput_h': round(sum(ln.palletized for ln in self.lanes.values()) / elapsed * 3600, 0) if elapsed > 30 else None,
            'pallets_h': round(self.pallets_delivered / elapsed * 3600, 1) if elapsed > 30 else None,
            'agv_util': round(100 * self.amr_busy / elapsed, 1) if elapsed > 1 else 0.0,
            'truck': {'name': t.name, 'status': t.status, 'task': t.task, 'held': t.held, 'arm': t.cell['arm'],
                      'color': t.cell['color'], 'rows': len(t.rows), 'rows_max': len(t.slots),
                      'boxes': sum(len(r) for r in t.rows), 'boxes_max': 4 * len(t.slots),
                      'boxes_loaded': t.boxes_loaded, 'trucks_departed': t.trucks_departed,
                      'truck_state': t.truck_state,
                      'util': round(100 * t.busy_time / elapsed, 1) if elapsed > 1 else 0.0},
            'elapsed': round(elapsed, 0), 'lanes': lanes, 'boxes': boxes,
        }
        self.state_pub.publish(String(data=json.dumps(st)))


def main():
    rclpy.init()
    node = PlantManager()
    ex = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stop_evt.set()
        node.shutdown_gz()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
