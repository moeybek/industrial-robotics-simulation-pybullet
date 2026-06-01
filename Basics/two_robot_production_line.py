import pybullet as p
import pybullet_data
import time
import math
import random

# Connect to PyBullet GUI
physics_client = p.connect(p.GUI)

# Basic setup
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

time_step = 1.0 / 240.0
p.setTimeStep(time_step)

# Camera
p.resetDebugVisualizerCamera(
    cameraDistance=5.5,
    cameraYaw=45,
    cameraPitch=-35,
    cameraTargetPosition=[0, 0, 0.6]
)

# Ground
plane_id = p.loadURDF("plane.urdf")

# Conveyor belt visual object
belt_collision = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[2.8, 0.35, 0.05]
)

belt_visual = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[2.8, 0.35, 0.05],
    rgbaColor=[0.2, 0.2, 0.2, 1]
)

belt_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=belt_collision,
    baseVisualShapeIndex=belt_visual,
    basePosition=[0, 0, 0.05]
)

# Load two KUKA robot arms
robot_1 = p.loadURDF(
    "kuka_iiwa/model.urdf",
    basePosition=[-1.2, -1.0, 0.0],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    useFixedBase=True
)

robot_2 = p.loadURDF(
    "kuka_iiwa/model.urdf",
    basePosition=[1.2, 1.0, 0.0],
    baseOrientation=p.getQuaternionFromEuler([0, 0, math.pi]),
    useFixedBase=True
)

# Labels
p.addUserDebugText(
    "Robot Arm 1",
    [-1.8, -1.3, 1.2],
    [0, 1, 0],
    1.2
)

p.addUserDebugText(
    "Robot Arm 2",
    [0.8, 1.3, 1.2],
    [0, 1, 1],
    1.2
)

p.addUserDebugText(
    "Production Line / Conveyor",
    [-1.0, -0.15, 0.25],
    [1, 1, 0],
    1.1
)

# Add coordinate axes
axis_length = 1.5
p.addUserDebugLine([0, 0, 0], [axis_length, 0, 0], [1, 0, 0], 3)
p.addUserDebugLine([0, 0, 0], [0, axis_length, 0], [0, 1, 0], 3)
p.addUserDebugLine([0, 0, 0], [0, 0, axis_length], [0, 0, 1], 3)
p.addUserDebugText("X", [axis_length, 0, 0], [1, 0, 0], 1.2)
p.addUserDebugText("Y", [0, axis_length, 0], [0, 1, 0], 1.2)
p.addUserDebugText("Z", [0, 0, axis_length], [0, 0, 1], 1.2)

# Get movable joints
def get_movable_joints(robot_id):
    joints = []
    for joint_index in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointInfo(robot_id, joint_index)
        joint_type = joint_info[2]

        if joint_type == p.JOINT_REVOLUTE:
            joints.append(joint_index)

    return joints


robot_1_joints = get_movable_joints(robot_1)
robot_2_joints = get_movable_joints(robot_2)

# Create boxes on conveyor
boxes = []

def create_box(x_position):
    size = 0.15

    box_collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[size, size, size]
    )

    box_visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[size, size, size],
        rgbaColor=[
            random.uniform(0.2, 1.0),
            random.uniform(0.2, 1.0),
            random.uniform(0.2, 1.0),
            1
        ]
    )

    box_id = p.createMultiBody(
        baseMass=0.2,
        baseCollisionShapeIndex=box_collision,
        baseVisualShapeIndex=box_visual,
        basePosition=[x_position, 0, 0.25]
    )

    return box_id


for x in [-2.2, -1.3, -0.4, 0.5, 1.4]:
    boxes.append(create_box(x))

# Simple robot animation
def animate_robot(robot_id, joints, t, phase_shift=0.0):
    joint_targets = [
        0.4 * math.sin(t + phase_shift),
        0.6 * math.sin(t * 0.7 + phase_shift),
        0.5 * math.cos(t * 0.8 + phase_shift),
        -1.2 + 0.3 * math.sin(t * 1.1 + phase_shift),
        0.4 * math.cos(t + phase_shift),
        1.0 + 0.4 * math.sin(t * 0.9 + phase_shift),
        0.3 * math.cos(t * 1.2 + phase_shift),
    ]

    for joint_index, target in zip(joints, joint_targets):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target,
            force=300,
            positionGain=0.05,
            velocityGain=1.0
        )


start_time = time.time()

try:
    while True:
        elapsed = time.time() - start_time

        # Animate both robots with different phase shifts
        animate_robot(robot_1, robot_1_joints, elapsed, phase_shift=0.0)
        animate_robot(robot_2, robot_2_joints, elapsed, phase_shift=math.pi)

        # Move boxes along the conveyor
        for box_id in boxes:
            position, orientation = p.getBasePositionAndOrientation(box_id)
            x, y, z = position

            new_x = x + 0.003

            if new_x > 2.6:
                new_x = -2.6

            p.resetBasePositionAndOrientation(
                box_id,
                [new_x, y, z],
                orientation
            )

        p.stepSimulation()
        time.sleep(time_step)

except KeyboardInterrupt:
    pass

finally:
    p.disconnect()