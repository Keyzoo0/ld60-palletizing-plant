# ld60_plant

Palletizing plant simulation: 4 machines and conveyors, 4 palletizing TM12S cells, an Omron LD-60 AMR
with a roller top, and a TM12S truck loader with a 4-cup gripper. Gazebo Jetty + MoveIt 2 (Pilz) +
ros2_control, with a MobilePlanner-style web dashboard on http://localhost:8081.

```bash
~/plant_ws/run_plant.sh [gz_gui:=false] [rviz:=true]
~/plant_ws/tools/soak.sh --fill --pallets 2        # end-to-end test (plant must be running)
python3 -m pytest -q test                          # route + AMR controller tests
```

All plant parameters live in `config/plant.yaml`. After changing the cells or the truck robot,
regenerate the arm model and MoveIt config with `tools/gen_arms.py`.

See the repository README for the full description, architecture, test results and lessons learned.
