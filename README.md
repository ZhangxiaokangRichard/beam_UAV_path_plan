# beam_UAV_path_plan

ROS1 (Noetic) workspace source for UAV path planning based on Beam Search + Dubins primitives.

## Packages

- `beam_dubins`: Core planner library, ROS messages/services, and planner server.
- `uav_guide_env`: Simulation loop, UAV/target dynamics, obstacle manager, and RViz publishers.

## Build

```bash
cd ~/beam_dubins_ws
catkin_make
```

## Run

```bash
source ~/beam_dubins_ws/devel/setup.bash
roslaunch uav_guide_env simulation.launch
```
