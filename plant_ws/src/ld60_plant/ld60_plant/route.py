"""Fixed-route navigation for the LD-60 (no ROS imports, unit-testable).

The route is a graph of goals (where the robot stops) and junction nodes. A trip is turned into
simple steps, the way an LD robot drives a taught path:
    rotate  - turn in place to a heading
    line    - drive a straight segment, forward or in reverse, with pure-pursuit steering
Dock goals are always entered and left straight (no turning at the dock itself); leaving a dock is
done in reverse when the robot faces the dock.
"""
import heapq
import math


def wrap(a):
    """Wrap an angle (rad) to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class RouteGraph:
    def __init__(self, cfg):
        self.goals = {k: dict(v) for k, v in cfg['goals'].items()}
        self.nodes = {k: dict(v) for k, v in cfg.get('nodes', {}).items()}
        self.points = {}
        for name, g in self.goals.items():
            self.points[name] = (float(g['x']), float(g['y']))
        for name, n in self.nodes.items():
            self.points[name] = (float(n['x']), float(n['y']))
        self.adj = {name: [] for name in self.points}
        for a, b in cfg['edges']:
            if a not in self.points or b not in self.points:
                raise ValueError(f'edge {a}-{b} uses an unknown point')
            d = self.dist(a, b)
            self.adj[a].append((b, d))
            self.adj[b].append((a, d))

    def dist(self, a, b):
        (ax, ay), (bx, by) = self.points[a], self.points[b]
        return math.hypot(bx - ax, by - ay)

    def is_dock(self, name):
        return bool(self.goals.get(name, {}).get('dock'))

    def goal_yaw(self, name):
        g = self.goals.get(name)
        return None if g is None or g.get('yaw') is None else math.radians(float(g['yaw']))

    def shortest_path(self, start, goal):
        """Dijkstra over the route graph. Returns the list of point names or None."""
        best = {start: 0.0}
        prev = {}
        pq = [(0.0, start)]
        while pq:
            d, u = heapq.heappop(pq)
            if u == goal:
                path = [u]
                while u in prev:
                    u = prev[u]
                    path.append(u)
                return path[::-1]
            if d > best.get(u, math.inf):
                continue
            for v, w in self.adj[u]:
                nd = d + w
                if nd < best.get(v, math.inf):
                    best[v], prev[v] = nd, u
                    heapq.heappush(pq, (nd, v))
        return None

    def nearest_point(self, x, y):
        return min(self.points, key=lambda n: math.hypot(self.points[n][0] - x, self.points[n][1] - y))

    def plan(self, pose, at_goal, target, params):
        """Steps to drive from pose (x, y, yaw[rad]) to the target goal.

        at_goal: the goal the robot is docked/parked at (exact), or None.
        """
        x, y, yaw = pose
        steps = []
        if at_goal is not None and at_goal in self.points:
            start = at_goal
        else:
            start = self.nearest_point(x, y)
            sx, sy = self.points[start]
            if math.hypot(sx - x, sy - y) > params['goal_distance_tol']:
                h = math.atan2(sy - y, sx - x)
                steps.append({'type': 'rotate', 'yaw': h})
                steps.append({'type': 'line', 'from': (x, y), 'to': (sx, sy), 'reverse': False,
                              'heading': h, 'dock': False, 'name': start})
                yaw = h
        if start == target:
            return steps + self._final_turn(target, yaw)
        path = self.shortest_path(start, target)
        if path is None:
            raise ValueError(f'no route from {start} to {target}')
        heading = yaw
        for a, b in zip(path, path[1:]):
            (ax, ay), (bx, by) = self.points[a], self.points[b]
            seg = math.atan2(by - ay, bx - ax)
            if self.is_dock(a) and a == path[0] and at_goal == a:
                # leaving a dock: keep the dock heading, back out if we face the dock
                h = self.goal_yaw(a)
                reverse = abs(wrap(seg - h)) > math.pi / 2
            elif self.is_dock(b) and b == target:
                h = self.goal_yaw(b)
                reverse = abs(wrap(seg - h)) > math.pi / 2
            else:
                h, reverse = seg, False
            if abs(wrap(h - heading)) > math.radians(params['goal_angle_tol']):
                steps.append({'type': 'rotate', 'yaw': h})
            steps.append({'type': 'line', 'from': (ax, ay), 'to': (bx, by), 'reverse': reverse,
                          'heading': h, 'dock': self.is_dock(b) and b == target, 'name': b})
            heading = h
        return steps + self._final_turn(target, heading)

    def _final_turn(self, target, heading):
        yaw = self.goal_yaw(target)
        if yaw is None or self.is_dock(target) or abs(wrap(yaw - heading)) < 1e-3:
            return []
        return [{'type': 'rotate', 'yaw': yaw}]


# --------------------------------------------------------------------------- controllers
def line_progress(pose, seg):
    """Distance travelled along the segment, distance remaining and signed cross-track error."""
    (sx, sy), (ex, ey) = seg['from'], seg['to']
    L = math.hypot(ex - sx, ey - sy)
    if L < 1e-9:
        return 0.0, 0.0, 0.0, 0.0
    ux, uy = (ex - sx) / L, (ey - sy) / L
    px, py = pose[0] - sx, pose[1] - sy
    along = px * ux + py * uy
    cross = ux * py - uy * px          # + = robot is left of the path
    return along, L - along, cross, L


def follow_line(pose, seg, v_prev, dt, p, vmax):
    """Pure pursuit along a straight segment. Returns (v, w, remaining, cross)."""
    along, remaining, cross, L = line_progress(pose, seg)
    (sx, sy), (ex, ey) = seg['from'], seg['to']
    ux, uy = (ex - sx) / max(L, 1e-9), (ey - sy) / max(L, 1e-9)
    if seg['reverse']:
        vmax = min(vmax, p['reverse_speed'])
    if seg['dock'] and remaining < p['dock_slow_distance']:
        vmax = min(vmax, p['dock_speed'])
    # speed profile: accelerate, cruise, decelerate to the end of the segment
    tol = p['dock_distance_tol'] if seg['dock'] else p['goal_distance_tol']
    v_stop = math.sqrt(max(0.0, 2.0 * p['decel'] * max(remaining - tol * 0.5, 0.0)))
    v = min(vmax, v_stop, abs(v_prev) + p['accel'] * dt)
    v = max(v, 0.02 if remaining > tol else 0.0)
    # look-ahead point on the (extended) line keeps steering stable right up to the stop
    ld = max(0.25, min(0.8, 0.6 * v + 0.25))
    tx, ty = sx + ux * (along + ld), sy + uy * (along + ld)
    virtual_yaw = pose[2] + (math.pi if seg['reverse'] else 0.0)
    alpha = wrap(math.atan2(ty - pose[1], tx - pose[0]) - virtual_yaw)
    w = 2.0 * max(v, 0.05) * math.sin(alpha) / ld
    w = max(-1.0, min(1.0, w))
    if seg['reverse']:
        v = -v
    return v, w, remaining, cross


def rotate_step(yaw, target, w_prev, dt, p):
    """Turn in place with acceleration limit. Returns (w, error)."""
    err = wrap(target - yaw)
    wmax = math.radians(p['rot_speed'])
    acc = math.radians(p['rot_accel'])
    w_stop = math.sqrt(2.0 * acc * abs(err))          # can still stop on target
    w = min(wmax, w_stop, abs(w_prev) + acc * dt)
    w = max(w, 0.08)                                   # creep so we never stall near the target
    return math.copysign(w, err), err
