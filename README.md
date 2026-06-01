# Robotics Simulation with Python and PyBullet

This repository contains beginner-friendly robotics simulation projects built with Python and PyBullet.

The projects demonstrate core robotics and automation concepts such as robot loading, URDF models, inverse kinematics, pick-and-place motion, mobile manipulation, conveyor systems, virtual sensors, object handling, and simple industrial automation workflows.

## Project Overview

The repository includes several simulation examples:

| File | Description |
|---|---|
| `01_epson_pick_place.py` | Epson robot pick-and-place simulation between two containers |
| `02_conveyer_system.py` | Conveyor system with ray-based start/end sensors |
| `03_husky_kuka_loading.py` | Husky mobile robot carrying a KUKA arm |
| `04_husky_kuka_pick_and_place.py` | Husky-KUKA mobile manipulation and pick-and-place |
| `05_UR_pick_and_place.py` | UR3 robot pick-and-place between two trays |

## Main Topics Covered

- PyBullet simulation setup
- Loading URDF robot models
- Loading custom robot assets
- Industrial robot arm control
- Mobile robot simulation
- Inverse kinematics
- End-effector control
- Pick-and-place logic
- Fake gripper behavior using fixed constraints
- Conveyor belt simulation
- Ray-based virtual sensors
- Object spawning and movement
- Basic automation cycles
- Debug visualization
- Keyboard control
- Simple production-line behavior

## Requirements

This project uses Python with PyBullet.

Recommended setup:

```text
Python 3.11 or 3.12
PyBullet
pybullet_data
```

On Apple Silicon Macs, installing PyBullet with `conda-forge` is recommended.

## Environment Setup

Create a conda environment:

```bash
conda create -n robot python=3.11
conda activate robot
```

Install PyBullet:

```bash
conda install -c conda-forge pybullet
```

Test the installation:

```bash
python -c "import pybullet as p; print('pybullet works')"
```

## Running the Simulations

Run any script from the project folder:

```bash
python 01_epson_pick_place.py
python 02_conveyer_system.py
python 03_husky_kuka_loading.py
python 04_husky_kuka_pick_and_place.py
python 05_UR_pick_and_place.py
```

Some scripts require custom URDF folders to exist in the correct relative path.

Example expected structure:

```text
robotic_simulation/
├── Robot_Arm_project/
│   ├── 01_epson_pick_place.py
│   ├── 02_conveyer_system.py
│   ├── 03_husky_kuka_loading.py
│   ├── 04_husky_kuka_pick_and_place.py
│   └── 05_UR_pick_and_place.py
└── URDFS/
    ├── Robots/
    │   ├── EPSON_LS3_B401S/
    │   └── Universal_Robots_UR3/
    └── conveyor_belt.urdf
```

## 01 - Epson Pick and Place

File:

```text
01_epson_pick_place.py
```

This simulation loads an Epson robot arm and moves small cubes between two containers.

The robot performs a repeated pick-and-place sequence:

```text
source container → pick object → move → release object → target container
```

Then the pick and place positions are swapped, so the robot moves the objects back.

### Concepts Demonstrated

- Custom Epson robot URDF loading
- Robot base mounting
- Inverse kinematics
- End-effector control
- Pick-and-place sequence
- Object grasping with fixed constraints
- Container creation using box shapes
- Repeated automation cycle

### Important Notes

The Epson robot script expects this URDF path:

```text
../URDFS/Robots/EPSON_LS3_B401S/EPSON_LS3_B401S.urdf
```

The URDF must be present together with its mesh files.

If PyBullet cannot load the robot, check for missing mesh paths inside the URDF.

## 02 - Conveyor System with Sensors

File:

```text
02_conveyer_system.py
```

This simulation loads a conveyor belt and uses ray tests as virtual sensors.

The script includes:

```text
start sensor
end sensor
sensor beams
spawned boxes
keyboard control
conveyor start/stop logic
```

### Keyboard Controls

| Key | Action |
|---|---|
| `Space` | Start/stop conveyor |
| `O` | Spawn a box |
| `Q` | Quit simulation |

### Concepts Demonstrated

- Conveyor URDF loading
- Ray-based object detection
- Virtual photoelectric sensors
- Object spawning
- Velocity-based object movement
- Automatic stopping when the end sensor is blocked

### Important Notes

The script expects the conveyor URDF at:

```text
URDFS/conveyor_belt.urdf
```

If the file is not found, update the path or replace the conveyor with a generated box shape.

## 03 - Husky with KUKA Loading

File:

```text
03_husky_kuka_loading.py
```

This simulation loads a Husky mobile robot and mounts a KUKA iiwa robot arm on top of it.

The Husky can be controlled with keyboard input, while the KUKA arm follows an inverse-kinematics target.

### Concepts Demonstrated

- Mobile robot loading
- KUKA arm loading
- Fixed constraint between Husky and KUKA
- Keyboard-based wheel control
- Skid-steering motion
- Inverse kinematics
- End-effector trajectory visualization
- Debug trail lines

### Keyboard Controls

| Key | Action |
|---|---|
| Arrow Up | Drive forward |
| Arrow Down | Drive backward |
| Arrow Left | Turn left |
| Arrow Right | Turn right |
| `S` | Save world state |

## 04 - Husky-KUKA Pick and Place

File:

```text
04_husky_kuka_pick_and_place.py
```

This simulation combines mobile navigation with robotic manipulation.

A Husky mobile base drives between two stations. A KUKA arm mounted on the Husky picks objects from one tray and places them in another tray.

The cycle then reverses direction.

### Main Workflow

```text
drive to source tray
pick object
drive to target tray
place object
repeat for all objects
drive back
move objects back to source
```

### Concepts Demonstrated

- Mobile manipulation
- Husky-KUKA system
- Driving to target positions
- Arm movement using inverse kinematics
- Object grasping using constraints
- Object release
- Base locking during arm operation
- Tray-to-tray material transport
- Repeated industrial cycle

### Why Base Locking Is Used

The script freezes the Husky base while the KUKA arm performs pick-and-place.

This prevents the mobile base from drifting due to physics forces while the arm is moving.

## 05 - UR3 Pick and Place

File:

```text
05_UR_pick_and_place.py
```

This simulation loads a Universal Robots UR3 arm and moves three colored cubes between two trays.

The robot repeatedly performs:

```text
Tray A → Tray B
Tray B → Tray A
```

### Objects

The scene contains:

```text
one UR3 robot arm
two trays
three colored cubes
```

### Concepts Demonstrated

- UR3 robot loading from custom URDF
- Tray creation
- Cube creation
- Inverse kinematics
- End-effector control
- Pick-and-place cycle
- Fake gripper using fixed constraints
- Collision filtering during grasping

### Pick-and-Place Sequence

For each object, the robot performs:

```text
approach object
descend
attach object to end-effector
lift object
move to target tray
descend
release object
retract
return home
```

## Robotics Concepts Explained

### URDF

URDF stands for Unified Robot Description Format.

It defines:

```text
robot links
robot joints
visual geometry
collision geometry
mass and inertia
joint limits
```

The simulations use URDF files to load robot models into PyBullet.

### Inverse Kinematics

Inverse kinematics calculates the robot joint angles needed to move the end-effector to a desired position.

Example:

```text
target hand position → joint angles
```

In PyBullet, this is done with:

```python
pb.calculateInverseKinematics(...)
```

### End-Effector

The end-effector is the robot's working point.

In real robots, this could be:

```text
gripper
suction cup
welding tool
camera
screwdriver
```

In these simulations, the end-effector is used as the point that reaches objects and attaches to them.

### Fake Gripper

Most example robot models in this repository do not include a physical gripper.

Instead, object grasping is simulated using a fixed constraint:

```python
pb.createConstraint(...)
```

This attaches the object to the robot end-effector.

This is a simplified but useful way to learn pick-and-place logic before modeling a real gripper.

### Ray Sensors

The conveyor simulation uses ray tests to simulate sensors.

A ray sensor checks whether a straight line between two points is blocked by an object.

In industry, this is similar to a photoelectric sensor detecting whether a product is present on a conveyor.

### Mobile Manipulation

The Husky-KUKA simulations combine:

```text
mobile base movement
robot arm manipulation
object transport
```

This is useful for understanding warehouse robots, mobile manipulators, and autonomous industrial transport systems.

## Industrial Relevance

These simulations are simplified versions of real industrial automation tasks.

Similar concepts are used in:

```text
factory automation
robotic sorting
machine tending
tray loading
bin picking
warehouse robotics
mobile manipulation
production-line inspection
digital twins
offline robot programming
```

Real industrial systems add:

```text
PLC communication
real grippers
camera calibration
safety zones
collision checking
force sensors
conveyor tracking
cycle-time optimization
emergency stop logic
```

## Known Limitations

- The grippers are simplified using constraints.
- Some robots require external custom URDF and mesh files.
- Paths are relative and may need adjustment depending on folder structure.
- Collision checking is basic.
- No real camera-based object detection is implemented.
- No PLC or ROS integration is included.
- The simulations are educational, not production-ready.
- Some URDF files may require mesh path corrections before loading.

## Troubleshooting

### PyBullet cannot load URDF

Check that the URDF file exists:

```bash
find .. -name "robot_name.urdf"
```

Check that mesh files exist:

```bash
find .. -name "*.stl"
find .. -name "*.obj"
find .. -name "*.dae"
```

If the URDF contains paths like:

```text
/Mesh/Visual/Base.stl
```

change them to relative paths:

```text
Mesh/Visual/Base.stl
```

### PyBullet cannot find conveyor URDF

If this error appears:

```text
URDF file 'URDFS/conveyor_belt.urdf' not found
```

search for the file:

```bash
find .. -name "conveyor_belt.urdf"
```

Then update the path in the script.

### PyBullet does not install with pip on Apple Silicon

Use conda instead:

```bash
conda create -n robot python=3.11
conda activate robot
conda install -c conda-forge pybullet
```

### GUI opens but nothing moves

PyBullet requires simulation stepping:

```python
pb.stepSimulation()
```

The scripts already include this inside loops. If you write a new script, make sure the simulation loop contains it.

## Repository Structure

Suggested repository structure:

```text
robotics-simulation-pybullet/
├── README.md
├── requirements.txt
├── 01_epson_pick_place.py
├── 02_conveyer_system.py
├── 03_husky_kuka_loading.py
├── 04_husky_kuka_pick_and_place.py
├── 05_UR_pick_and_place.py
└── URDFS/
    ├── Robots/
    │   ├── EPSON_LS3_B401S/
    │   └── Universal_Robots_UR3/
    └── conveyor_belt.urdf
```

## Suggested `.gitignore`

```gitignore
# Python
__pycache__/
*.pyc
*.pyo
*.pyd

# Virtual environments
.venv/
venv/
env/
robot/

# macOS
.DS_Store

# VS Code
.vscode/

# Logs
*.log

# Build artifacts
build/
dist/
*.egg-info/
```

## Learning Outcomes

By working through these examples, I practiced:

```text
PyBullet simulation setup
URDF loading and debugging
robot joint control
inverse kinematics
pick-and-place automation
mobile robot control
sensor simulation using ray tests
object creation and manipulation
fixed constraints for simplified grasping
debug visualization
industrial automation logic
```

## Future Improvements

Possible next steps:

```text
add real gripper models
add collision-aware path planning
add camera-based object detection
add ROS 2 integration
add conveyor tracking
add production counters
add sensor dashboards
add safety zones
add multi-robot coordination
add more realistic robot controllers
add digital twin style factory layouts
```

## Author

Moawia Sardar Bagdash

## License

This project is for learning and portfolio demonstration purposes.