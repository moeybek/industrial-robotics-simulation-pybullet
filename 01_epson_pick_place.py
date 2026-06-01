import pybullet as pb
import pybullet_data
import os
import time

class EpsonPickPlace:
    def __init__(self, base_position=(0, 0, 0.25), urdf_path=None):
        """
        Initialize the Epson pick and place system.
        
        Args:
            base_position: (x, y, z) tuple for robot base mounting position
            urdf_path: Path to URDF file (defaults to EPSON_LS3_B401S)
        """
        self.base_position = base_position
        self.urdf_path = urdf_path or os.path.join("..", "URDFS", "Robots", "EPSON_LS3_B401S", "EPSON_LS3_B401S.urdf")
        
        self.robot = None
        self.joints = []
        self.ee_link = None
        self.constraint_id = None

    def setup_simulation(self):
        """Initialize PyBullet simulation environment."""
        pb.connect(pb.GUI)
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)  # Disable extra GUI windows
        pb.resetSimulation()
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0, 0, -9.8)
        pb.setTimeStep(1. / 240.)
        pb.setPhysicsEngineParameter(numSolverIterations=150)
        pb.loadURDF("plane.urdf")
        
        pb.resetDebugVisualizerCamera(
            cameraDistance=1.2,
            cameraYaw=45,
            cameraPitch=-40,
            cameraTargetPosition=[0, 0, 0.25]
        )

    def load_robot(self):
        """Load and configure the Epson robot."""
        robot_folder = os.path.dirname(self.urdf_path)
        pb.setAdditionalSearchPath(robot_folder)

        print("Trying to load URDF from:", self.urdf_path)
        print("Exists:", os.path.exists(self.urdf_path))
        print("Robot folder:", robot_folder)

        self.robot = pb.loadURDF(
            self.urdf_path,
            self.base_position,
            useFixedBase=True
        )

        num_joints = pb.getNumJoints(self.robot)
        colors = [[1.0, 1.0, 1.0, 1.0], [0.3, 0.3, 0.3, 1.0]]

        for i in range(-1, num_joints):
            pb.changeDynamics(self.robot, i, linearDamping=0.5, angularDamping=0.5, jointDamping=1.0)
            pb.changeVisualShape(self.robot, i, rgbaColor=colors[i % 2])

        self.joints = [
            i for i in range(num_joints)
            if pb.getJointInfo(self.robot, i)[2] in (pb.JOINT_REVOLUTE, pb.JOINT_PRISMATIC)
        ]

        for i in range(num_joints):
            if pb.getJointInfo(self.robot, i)[12].decode() == "ee_link":
                self.ee_link = i
                break
    
    def move_to(self, position, wait_steps=60):
        """
        Move end effector to target position using IK.
        
        Args:
            position: (x, y, z) target position
            wait_steps: Number of simulation steps to wait
        """
        orn = pb.getQuaternionFromEuler([0, 0, 0])
        joint_positions = pb.calculateInverseKinematics(
            self.robot, self.ee_link, position, orn, maxNumIterations=200
        )
        
        for joint, target_pos in zip(self.joints, joint_positions):
            pb.setJointMotorControl2(self.robot, joint, pb.POSITION_CONTROL, target_pos, force=200)
        
        self._wait(wait_steps)
    
    def grasp(self, obj_id):
        """Attach object to end effector."""
        ee_pos, ee_orn = pb.getLinkState(self.robot, self.ee_link)[:2]
        obj_pos, obj_orn = pb.getBasePositionAndOrientation(obj_id)
        
        inv_pos, inv_orn = pb.invertTransform(ee_pos, ee_orn)
        rel_pos, rel_orn = pb.multiplyTransforms(inv_pos, inv_orn, obj_pos, obj_orn)
        
        self.constraint_id = pb.createConstraint(
            self.robot, self.ee_link, obj_id, -1,
            pb.JOINT_FIXED, [0, 0, 0],
            rel_pos, [0, 0, 0], rel_orn, [0, 0, 0, 1]
        )
        self._wait(20)
    
    def release(self):
        """Release currently grasped object."""
        if self.constraint_id is not None:
            pb.removeConstraint(self.constraint_id)
            self.constraint_id = None
        self._wait(40)

    def pick_and_place(self, object_ids, pick_positions, place_positions):
        """
        Execute pick and place operations with existing objects.
        
        Args:
            object_ids: List of PyBullet object IDs to manipulate
            pick_positions: List of (x, y, z) positions to pick from
            place_positions: List of (x, y, z) positions to place at
        """
        for obj_id, pick_pos, place_pos in zip(object_ids, pick_positions, place_positions):
            # Approach
            self.move_to([pick_pos[0], pick_pos[1], pick_pos[2] + 0.08], wait_steps=60)
            
            # Descend
            self.move_to([pick_pos[0], pick_pos[1], pick_pos[2] + 0.005], wait_steps=40)
            
            # Grasp
            self.grasp(obj_id)
            
            # Lift
            self.move_to([pick_pos[0], pick_pos[1], pick_pos[2] + 0.08], wait_steps=40)
            
            # Move to place location
            self.move_to([place_pos[0], place_pos[1], place_pos[2] + 0.08], wait_steps=60)
            
            # Release
            self.release()

    def create_objects(self, positions):
        """Create cube objects at specified positions."""
        return [self._create_cube(pos) for pos in positions]
    
    def _wait(self, steps):
        """Execute simulation steps with real-time delay."""
        for _ in range(steps):
            pb.stepSimulation()
            time.sleep(1. / 240.)
    
    def _create_cube(self, position, size=0.01, mass=0.05):
        """Create a small cube object."""
        col = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[size] * 3)
        vis = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[size] * 3,
                                   rgbaColor=[0.9, 0.6, 0.1, 1])
        return pb.createMultiBody(mass, col, vis, position)

def create_container(position, size=(0.1, 0.08, 0.04), color=(0.5, 0.5, 0.5, 1)):
    """Create a container box at specified position."""
    w, d, h = size
    thickness = 0.005
    
    def wall(half_extents, pos):
        col = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=half_extents)
        vis = pb.createVisualShape(pb.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
        pb.createMultiBody(0, col, vis, pos)
    
    # Bottom and four walls
    wall([w/2, d/2, thickness/2], position)
    wall([w/2, thickness/2, h/2], [position[0], position[1]-d/2, position[2]+h/2])
    wall([w/2, thickness/2, h/2], [position[0], position[1]+d/2, position[2]+h/2])
    wall([thickness/2, d/2, h/2], [position[0]-w/2, position[1], position[2]+h/2])
    wall([thickness/2, d/2, h/2], [position[0]+w/2, position[1], position[2]+h/2])


def create_mounting_base(position, size=0.25):
    """Create mounting cube for robot base."""
    half = size / 2
    col = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[half, half, half])
    vis = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[half, half, half],
                               rgbaColor=[0.3, 0.3, 0.35, 1])
    pb.createMultiBody(0, col, vis, position)

if __name__ == "__main__":
    table_height = 0.25
    base_pos= (0, 0, table_height)

    source_pos=(0.2,0.2,table_height)
    target_pos=(0.3,0,table_height)

    slot_offsets=[(-0.025,0),(0,0),(0.025,0)]

    #Initialize
    system = EpsonPickPlace(base_position=base_pos)
    system.setup_simulation()

    create_mounting_base([0, 0, table_height/2])
    system.load_robot()

    # Create container and objects
    create_container(target_pos,color=(0.2,0.5,0.8,1))
    create_container(source_pos,color=(0.8,0.5,0.2,1))

    pick_positions=[(source_pos[0]+offset[0], source_pos[1]+offset[1], source_pos[2]+0.025) for offset in slot_offsets]
    place_positions=[(target_pos[0]+offset[0], target_pos[1]+offset[1], target_pos[2]+0.025) for offset in slot_offsets]

    objects = system.create_objects(pick_positions)
    system._wait(200)

    while True:
        system.pick_and_place(objects, pick_positions, place_positions)
        pick_positions, place_positions = place_positions, pick_positions  # Swap for next cycle
        time.sleep(0.5)