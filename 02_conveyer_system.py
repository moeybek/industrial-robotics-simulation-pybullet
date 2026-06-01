import pybullet as pb
import pybullet_data
import time

# Initialize
pb.connect(pb.GUI)
pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.setGravity(0, 0, -9.81)
pb.loadURDF("plane.urdf")

# Load conveyor
conveyor = pb.loadURDF("URDFS/conveyor_belt.urdf", [0, 0, 0.5], useFixedBase=True)

# Sensor link indices (from URDF joint order)
START_SENSOR = 0       # start_sensor_link (ray from)
START_TARGET = 1       # start_sensor_target_link (ray to)
END_SENSOR = 2         # end_sensor_link (ray from)  
END_TARGET = 3         # end_sensor_target_link (ray to)

# Debug lines for sensor beams
start_beam_id = None
end_beam_id = None

def get_link_pos(link_idx):
    """Get link world position."""
    return pb.getLinkState(conveyor, link_idx)[0]

def check_sensor(sensor_idx, target_idx):
    """Check if object blocks beam between sensor and target."""
    ray_from = get_link_pos(sensor_idx)
    ray_to = get_link_pos(target_idx)
    result = pb.rayTest(ray_from, ray_to)
    hit_id = result[0][0]
    return hit_id > 0 and hit_id != conveyor

def draw_beam(sensor_idx, target_idx, blocked, old_id):
    """Draw sensor beam (green=clear, red=blocked)."""
    if old_id is not None:
        pb.removeUserDebugItem(old_id)
    color = [1, 0, 0] if blocked else [0, 1, 0]
    return pb.addUserDebugLine(get_link_pos(sensor_idx), get_link_pos(target_idx), color, 2)

def spawn_box():
    """Spawn box at start of conveyor."""
    col = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
    vis = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05], rgbaColor=[1, 0, 0, 1])
    return pb.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                              basePosition=[-2.3, 0, 0.6])

# Settings
SPEED = 5.0
running = False
objects = []

# Camera
pb.resetDebugVisualizerCamera(5.0, 45, -30, [0, 0, 0.5])

# Main loop
try:
    while True:
        keys = pb.getKeyboardEvents()
        if ord('q') in keys and keys[ord('q')] & pb.KEY_WAS_TRIGGERED:
            break
        if ord(' ') in keys and keys[ord(' ')] & pb.KEY_WAS_TRIGGERED:
            running = not running
        if ord('o') in keys and keys[ord('o')] & pb.KEY_WAS_TRIGGERED:
            objects.append(spawn_box())

        # Check sensors
        start_blocked = check_sensor(START_SENSOR, START_TARGET)
        end_blocked = check_sensor(END_SENSOR, END_TARGET)
        
        # Draw beams
        start_beam_id = draw_beam(START_SENSOR, START_TARGET, start_blocked, start_beam_id)
        end_beam_id = draw_beam(END_SENSOR, END_TARGET, end_blocked, end_beam_id)
        
        # Stop when end sensor triggered
        if end_blocked:
            running = False
        
        # Move objects using velocity (physics-based, respects gravity)
        if running:
            for obj in objects:
                pb.resetBaseVelocity(obj, [SPEED, 0, 0])
        else:
            for obj in objects:
                vel = pb.getBaseVelocity(obj)[0]
                pb.resetBaseVelocity(obj, [0, vel[1], vel[2]])  # Stop X, keep Y/Z
        
        pb.stepSimulation()
        time.sleep(1/240)
except:
    pass
finally:
    try:
        pb.disconnect()
    except:
        pass