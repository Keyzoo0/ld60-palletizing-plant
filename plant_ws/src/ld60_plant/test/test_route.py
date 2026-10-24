import math
import os

import yaml

from ld60_plant.route import RouteGraph, follow_line, line_progress, rotate_step, wrap

HERE = os.path.dirname(__file__)
with open(os.path.join(HERE, '..', 'config', 'plant.yaml')) as fh:
    CFG = yaml.safe_load(fh)
P = CFG['robot']


def graph():
    return RouteGraph(CFG)


def lines(steps):
    return [s for s in steps if s['type'] == 'line']


def test_home_to_station_reverses_nothing_and_docks_forward():
    g = graph()
    steps = g.plan((2.2, -3.0, math.radians(90)), 'Home', 'PalletStation', P)
    ls = lines(steps)
    assert [s['name'] for s in ls] == ['J_Home', 'PalletStation']
    assert ls[0]['reverse'] is False          # leaves home forward (faces +Y)
    assert ls[1]['reverse'] is False and ls[1]['dock'] is True
    assert math.isclose(math.degrees(ls[1]['heading']) % 360, 180, abs_tol=1e-6)


def test_station_to_truck_backs_out_then_docks_forward():
    g = graph()
    steps = g.plan((0.87, 0.0, math.pi), 'PalletStation', 'TruckDock', P)
    ls = lines(steps)
    assert [s['name'] for s in ls] == ['J_Home', 'J_Dock', 'TruckDock']
    assert ls[0]['reverse'] is True           # backs out of the station
    assert steps[0]['type'] == 'line'         # no turning while still in the dock
    assert ls[2]['dock'] is True and ls[2]['reverse'] is False


def test_back_home_reverses_into_charger():
    g = graph()
    steps = g.plan((12.33, 0.0, 0.0), 'TruckDock', 'Home', P)
    ls = lines(steps)
    assert ls[0]['reverse'] is True           # out of the truck
    assert ls[-1]['name'] == 'Home' and ls[-1]['reverse'] is True and ls[-1]['dock'] is True


def test_off_route_start_goes_to_nearest_point_first():
    g = graph()
    steps = g.plan((5.0, 0.5, 0.0), None, 'TruckDock', P)
    assert lines(steps)[0]['name'] in g.points


def test_line_progress_and_cross_track():
    seg = {'from': (0, 0), 'to': (10, 0)}
    along, rem, cross, L = line_progress((4.0, 0.2, 0.0), seg)
    assert math.isclose(along, 4) and math.isclose(rem, 6) and math.isclose(cross, 0.2)


def test_pure_pursuit_converges_to_line_and_stops_on_target():
    seg = {'from': (0, 0), 'to': (3, 0), 'reverse': False, 'dock': True}
    x, y, yaw, v = 0.0, 0.05, 0.1, 0.0
    dt = 0.05
    for _ in range(2000):
        v, w, rem, cross = follow_line((x, y, yaw), seg, v, dt, P, P['cruise_speed'])
        if rem <= P['dock_distance_tol']:
            break
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
    assert abs(rem) <= P['dock_distance_tol'] + 0.005
    assert abs(y) < 0.01 and abs(wrap(yaw)) < math.radians(1.5)


def test_reverse_following_moves_backwards():
    seg = {'from': (2, 0), 'to': (0, 0), 'reverse': True, 'dock': False}
    v, w, rem, cross = follow_line((2.0, 0.0, 0.0), seg, 0.2, 0.05, P, 1.0)
    assert v < 0 and abs(v) <= P['reverse_speed'] + 1e-9


def test_rotate_step_turns_the_short_way():
    w, err = rotate_step(math.radians(170), math.radians(-170), 0.0, 0.05, P)
    assert err > 0 and w > 0
