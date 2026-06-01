import pybullet as p
import pybullet_data
import time
import math


# ============================================================
# TWO ROBOT PICK AND PLACE SIMULATION
# ============================================================
# - Two KUKA iiwa robot arms
# - Two boxes
# - Two pick locations
# - Two place locations
# - Inverse kinematics
# - State machines for both robots
# - Fake gripper behavior
# - Visible pick and place motion
# ============================================================


# ----------------------------
# PyBullet setup
# ----------------------------
physics_client = p.connect(p.GUI)

p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

time_step = 1.0 / 240.0
p.setTimeStep(time_step)

p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

p.resetDebugVisualizerCamera(
    cameraDistance=5.0,
    cameraYaw=45,
    cameraPitch=-35,
    cameraTargetPosition=[0, 0, 0.7]
)


# ----------------------------
# Ground
# ----------------------------
plane_id = p.loadURDF("plane.urdf")


# ----------------------------
# Work table / conveyor
# ----------------------------
table_collision = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[2.2, 0.55, 0.05]
)

table_visual = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[2.2, 0.55, 0.05],
    rgbaColor=[0.20, 0.20, 0.20, 1]
)

table_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=table_collision,
    baseVisualShapeIndex=table_visual,
    basePosition=[0, 0, 0.05]
)


# ----------------------------
# Place bins
# ----------------------------
def create_bin(position, color):
    bin_collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[0.25, 0.25, 0.05]
    )

    bin_visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[0.25, 0.25, 0.05],
        rgbaColor=color
    )

    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=bin_collision,
        baseVisualShapeIndex=bin_visual,
        basePosition=position
    )


bin_1 = create_bin([0.95, -0.35, 0.16], [0, 0.7, 0, 1])
bin_2 = create_bin([-0.95, 0.35, 0.16], [0, 0.4, 1, 1])


# ----------------------------
# Load two KUKA robots
# ----------------------------
robot_1 = p.loadURDF(
    "kuka_iiwa/model.urdf",
    basePosition=[-1.35, -1.0, 0],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    useFixedBase=True
)

robot_2 = p.loadURDF(
    "kuka_iiwa/model.urdf",
    basePosition=[1.35, 1.0, 0],
    baseOrientation=p.getQuaternionFromEuler([0, 0, math.pi]),
    useFixedBase=True
)


# ----------------------------
# Joint helpers
# ----------------------------
def get_revolute_joints(robot_id):
    joints = []

    for joint_index in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointInfo(robot_id, joint_index)
        joint_type = joint_info[2]

        if joint_type == p.JOINT_REVOLUTE:
            joints.append(joint_index)

    return joints


robot_1_joints = get_revolute_joints(robot_1)
robot_2_joints = get_revolute_joints(robot_2)

end_effector_link_index = 6


# ----------------------------
# Create boxes
# ----------------------------
def create_box(position, color):
    box_size = 0.11

    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[box_size, box_size, box_size]
    )

    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[box_size, box_size, box_size],
        rgbaColor=color
    )

    box_id = p.createMultiBody(
        baseMass=0.2,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=position
    )

    p.changeDynamics(
        box_id,
        -1,
        lateralFriction=0.8,
        restitution=0.05
    )

    return box_id


box_1 = create_box([-0.35, -0.25, 0.28], [1, 0.1, 0.1, 1])
box_2 = create_box([0.35, 0.25, 0.28], [1, 0.8, 0.1, 1])


# ----------------------------
# Debug markers
# ----------------------------
def add_marker(position, color, label):
    visual = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=0.04,
        rgbaColor=color
    )

    marker = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=visual,
        basePosition=position
    )

    p.addUserDebugText(
        label,
        [position[0], position[1], position[2] + 0.12],
        color[:3],
        1.0
    )

    return marker


# Robot 1 targets
r1_pick = [-0.35, -0.25, 0.38]
r1_pick_above = [-0.35, -0.25, 0.78]
r1_place = [0.95, -0.35, 0.38]
r1_place_above = [0.95, -0.35, 0.78]
r1_home = [0.10, -0.45, 0.95]

# Robot 2 targets
r2_pick = [0.35, 0.25, 0.38]
r2_pick_above = [0.35, 0.25, 0.78]
r2_place = [-0.95, 0.35, 0.38]
r2_place_above = [-0.95, 0.35, 0.78]
r2_home = [-0.10, 0.45, 0.95]

add_marker(r1_pick, [1, 0, 0, 1], "R1 Pick")
add_marker(r1_place, [0, 1, 0, 1], "R1 Place")
add_marker(r2_pick, [1, 0.8, 0, 1], "R2 Pick")
add_marker(r2_place, [0, 0.5, 1, 1], "R2 Place")


# ----------------------------
# Coordinate axes
# ----------------------------
axis_length = 1.2

p.addUserDebugLine([0, 0, 0], [axis_length, 0, 0], [1, 0, 0], 3)
p.addUserDebugLine([0, 0, 0], [0, axis_length, 0], [0, 1, 0], 3)
p.addUserDebugLine([0, 0, 0], [0, 0, axis_length], [0, 0, 1], 3)

p.addUserDebugText("X", [axis_length, 0, 0], [1, 0, 0], 1.0)
p.addUserDebugText("Y", [0, axis_length, 0], [0, 1, 0], 1.0)
p.addUserDebugText("Z", [0, 0, axis_length], [0, 0, 1], 1.0)


# ----------------------------
# Math helpers
# ----------------------------
def distance(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )


def get_end_effector_position(robot_id):
    return p.getLinkState(robot_id, end_effector_link_index)[0]


def move_robot_to(robot_id, joints, target_position):
    target_orientation = p.getQuaternionFromEuler([0, math.pi, 0])

    joint_poses = p.calculateInverseKinematics(
        bodyUniqueId=robot_id,
        endEffectorLinkIndex=end_effector_link_index,
        targetPosition=target_position,
        targetOrientation=target_orientation,
        maxNumIterations=100,
        residualThreshold=0.001
    )

    for i, joint_index in enumerate(joints):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_poses[i],
            force=700,
            positionGain=0.10,
            velocityGain=1.0
        )


def hold_box_at_end_effector(robot_id, box_id):
    ee_position = get_end_effector_position(robot_id)

    box_position = [
        ee_position[0],
        ee_position[1],
        ee_position[2] - 0.12
    ]

    p.resetBasePositionAndOrientation(
        box_id,
        box_position,
        p.getQuaternionFromEuler([0, 0, 0])
    )

    p.resetBaseVelocity(
        box_id,
        linearVelocity=[0, 0, 0],
        angularVelocity=[0, 0, 0]
    )


# ----------------------------
# Robot pick-and-place controller
# ----------------------------
class PickPlaceRobot:
    def __init__(
        self,
        name,
        robot_id,
        joints,
        box_id,
        pick,
        pick_above,
        place,
        place_above,
        home,
        start_delay=0.0
    ):
        self.name = name
        self.robot_id = robot_id
        self.joints = joints
        self.box_id = box_id

        self.pick = pick
        self.pick_above = pick_above
        self.place = place
        self.place_above = place_above
        self.home = home

        self.start_delay = start_delay
        self.state = "WAIT"
        self.state_start_time = time.time()
        self.carrying = False
        self.finished_cycles = 0

    def set_state(self, new_state):
        self.state = new_state
        self.state_start_time = time.time()

    def update(self, global_elapsed):
        elapsed_in_state = time.time() - self.state_start_time
        ee_pos = get_end_effector_position(self.robot_id)

        if global_elapsed < self.start_delay:
            move_robot_to(self.robot_id, self.joints, self.home)
            return

        if self.state == "WAIT":
            self.set_state("MOVE_ABOVE_PICK")

        elif self.state == "MOVE_ABOVE_PICK":
            move_robot_to(self.robot_id, self.joints, self.pick_above)

            if distance(ee_pos, self.pick_above) < 0.10:
                self.set_state("MOVE_DOWN_TO_PICK")

        elif self.state == "MOVE_DOWN_TO_PICK":
            move_robot_to(self.robot_id, self.joints, self.pick)

            if distance(ee_pos, self.pick) < 0.10:
                self.set_state("GRASP")

        elif self.state == "GRASP":
            move_robot_to(self.robot_id, self.joints, self.pick)

            if elapsed_in_state > 0.4:
                self.carrying = True
                self.set_state("LIFT")

        elif self.state == "LIFT":
            move_robot_to(self.robot_id, self.joints, self.pick_above)

            if self.carrying:
                hold_box_at_end_effector(self.robot_id, self.box_id)

            if distance(ee_pos, self.pick_above) < 0.10:
                self.set_state("MOVE_ABOVE_PLACE")

        elif self.state == "MOVE_ABOVE_PLACE":
            move_robot_to(self.robot_id, self.joints, self.place_above)

            if self.carrying:
                hold_box_at_end_effector(self.robot_id, self.box_id)

            if distance(ee_pos, self.place_above) < 0.10:
                self.set_state("MOVE_DOWN_TO_PLACE")

        elif self.state == "MOVE_DOWN_TO_PLACE":
            move_robot_to(self.robot_id, self.joints, self.place)

            if self.carrying:
                hold_box_at_end_effector(self.robot_id, self.box_id)

            if distance(ee_pos, self.place) < 0.10:
                self.set_state("RELEASE")

        elif self.state == "RELEASE":
            move_robot_to(self.robot_id, self.joints, self.place)

            if elapsed_in_state > 0.4:
                self.carrying = False

                p.resetBasePositionAndOrientation(
                    self.box_id,
                    [self.place[0], self.place[1], 0.28],
                    p.getQuaternionFromEuler([0, 0, 0])
                )

                p.resetBaseVelocity(
                    self.box_id,
                    linearVelocity=[0, 0, 0],
                    angularVelocity=[0, 0, 0]
                )

                self.finished_cycles += 1
                self.set_state("RETURN_HOME")

        elif self.state == "RETURN_HOME":
            move_robot_to(self.robot_id, self.joints, self.home)

            if distance(ee_pos, self.home) < 0.12:
                self.set_state("RESET_BOX")

        elif self.state == "RESET_BOX":
            # repeat the pick-and-place cycle
            if self.name == "Robot 1":
                reset_position = [-0.35, -0.25, 0.28]
            else:
                reset_position = [0.35, 0.25, 0.28]

            p.resetBasePositionAndOrientation(
                self.box_id,
                reset_position,
                p.getQuaternionFromEuler([0, 0, 0])
            )

            p.resetBaseVelocity(
                self.box_id,
                linearVelocity=[0, 0, 0],
                angularVelocity=[0, 0, 0]
            )

            self.set_state("MOVE_ABOVE_PICK")


# ----------------------------
# Create two independent robot controllers
# ----------------------------
controller_1 = PickPlaceRobot(
    name="Robot 1",
    robot_id=robot_1,
    joints=robot_1_joints,
    box_id=box_1,
    pick=r1_pick,
    pick_above=r1_pick_above,
    place=r1_place,
    place_above=r1_place_above,
    home=r1_home,
    start_delay=0.0
)

controller_2 = PickPlaceRobot(
    name="Robot 2",
    robot_id=robot_2,
    joints=robot_2_joints,
    box_id=box_2,
    pick=r2_pick,
    pick_above=r2_pick_above,
    place=r2_place,
    place_above=r2_place_above,
    home=r2_home,
    start_delay=1.5
)


# ----------------------------
# Labels
# ----------------------------
p.addUserDebugText(
    "Two Robot Pick-and-Place Simulation",
    [-1.6, -1.4, 1.7],
    [1, 1, 1],
    1.3
)

p.addUserDebugText(
    "Press Q in the PyBullet window to quit",
    [-1.6, -1.4, 1.5],
    [1, 1, 0],
    1.1
)


# ----------------------------
# Main loop
# ----------------------------
start_time = time.time()
dashboard_text_id = None

try:
    while True:
        global_elapsed = time.time() - start_time

        controller_1.update(global_elapsed)
        controller_2.update(global_elapsed)

        if dashboard_text_id is not None:
            p.removeUserDebugItem(dashboard_text_id)

        dashboard_text = (
            f"Robot 1 state: {controller_1.state} | cycles: {controller_1.finished_cycles}\n"
            f"Robot 2 state: {controller_2.state} | cycles: {controller_2.finished_cycles}\n"
            f"Simulation time: {global_elapsed:.1f} s"
        )

        dashboard_text_id = p.addUserDebugText(
            dashboard_text,
            [-1.8, -1.4, 1.25],
            [1, 1, 1],
            1.0
        )

        keys = p.getKeyboardEvents()

        if ord("q") in keys and keys[ord("q")] & p.KEY_WAS_TRIGGERED:
            break

        p.stepSimulation()
        time.sleep(time_step)

except KeyboardInterrupt:
    pass

finally:
    p.disconnect()