
import pybullet as pb
import pybullet_data
import time
import math

class UR3:
    def __init__(self):
        self.id = pb.loadURDF("../URDFS/Robots/Universal_Robots_UR3/Universal_Robots_UR3.urdf", [0,0,0], useFixedBase=1)
        num_joints = pb.getNumJoints(self.id)
        ur_silver = [0.8, 0.8, 0.8, 1.0]
        ur_blue = [0.2, 0.5, 0.8, 1.0]
        for i in range(-1, num_joints):
            pb.changeDynamics(self.id, i, linearDamping=0.5, angularDamping=0.5, jointDamping=1.0)
            # Alternate colors for joints and links to mimic UR style
            color = ur_blue if i % 2 == 0 else ur_silver
            pb.changeVisualShape(self.id, i, rgbaColor=color)
        # Home: Upright and facing +Y slightly
        self.home_conf = [-1.57, -1.0, 1.2, -1.57, -1.57, 0]
        self.ll, self.ul = [-3.14]*6, [3.14]*6
        self.jr, self.rp = [6.28]*6, self.home_conf
        self.home()

    def home(self):
        for i, val in enumerate(self.home_conf):
            pb.setJointMotorControl2(self.id, i, pb.POSITION_CONTROL, val, force=200)
        for _ in range(60): pb.stepSimulation(); time.sleep(1/240)

    def move(self, pos):
        orn = pb.getQuaternionFromEuler([math.pi, 0, 0])
        j = pb.calculateInverseKinematics(self.id, 6, pos, orn, self.ll, self.ul, self.jr, self.rp)
        for i in range(6):
            pb.setJointMotorControl2(self.id, i, pb.POSITION_CONTROL, j[i], force=200, positionGain=0.3)
        for _ in range(120): pb.stepSimulation(); time.sleep(1/240)

    def pick(self, obj):

        p = pb.getBasePositionAndOrientation(obj)[0]
        print(f"Pick {p}")
        self.move([p[0], p[1]-0.02, p[2]+0.15]) # Appr
        self.move([p[0], p[1]-0.02, p[2]+0.01]) # Descend
        
        # Grasp
        inv = pb.invertTransform(*pb.getLinkState(self.id, 6)[0:2])
        rel = pb.multiplyTransforms(*inv, *pb.getBasePositionAndOrientation(obj))
        c = pb.createConstraint(self.id, 6, obj, -1, pb.JOINT_FIXED, [0,0,0], rel[0], [0,0,0], rel[1], [0,0,0,1])
        pb.changeConstraint(c, maxForce=5000)
        for i in range(6): pb.setCollisionFilterPair(self.id, obj, i, -1, 0)
        
        for _ in range(50): pb.stepSimulation(); time.sleep(1/240)
        self.move([p[0], p[1]-0.02, p[2]+0.15]) # Lift
        return c

    def place(self, c, p, obj):
        print(f"Place {p}")
        self.move([p[0], p[1], p[2]+0.15]) # Appr
        self.move([p[0], p[1], p[2]+0.03]) # Descend
        
        # Release
        pb.removeConstraint(c)
        for i in range(6): pb.setCollisionFilterPair(self.id, obj, i, -1, 1)
        
        for _ in range(50): pb.stepSimulation(); time.sleep(1/240)
        self.move([p[0], p[1], p[2]+0.15]) # Retract

def main():
    pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -9.81)
    pb.loadURDF("plane.urdf")
    pb.resetDebugVisualizerCamera(1.2, 0, -45, [0,0,0.4])
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

    def mk_box(p, c, ext):
        v = pb.createVisualShape(pb.GEOM_BOX, halfExtents=ext, rgbaColor=c)
        o = pb.createMultiBody(0.2 if ext[0]<0.05 else 0, pb.createCollisionShape(pb.GEOM_BOX, halfExtents=ext), v, p)
        pb.changeDynamics(o, -1, lateralFriction=1.0); return o

    # --- UPDATED LAYOUT: SAFELY POSITIVE Y ---
    mk_box([0.3, 0.25, 0.02], [0,1,0,1], [0.08, 0.08, 0.02])   # Tray A (Far Left)
    mk_box([-0.3, 0.25, 0.02], [1,0,0,1], [0.08, 0.08, 0.02])   # Tray B (Near Left)
    
    # Explicit Slots (Y-distributed on X=0.3 line)
    # Tray A (Center 0.25): 0.20, 0.25, 0.30
    POS_A = [[0.3, 0.18, 0.065], [0.3, 0.24, 0.065], [0.3, 0.29, 0.065]]
    
    # Tray B (Center 0.25): 0.20, 0.25, 0.30
    POS_B = [[-0.3, 0.2, 0.065], [-0.3, 0.25, 0.065], [-0.3, 0.3, 0.065]]
    
    colors = [[0.9,0.2,0.2,1], [0.2,0.8,0.2,1], [0.2,0.2,0.9,1]]
    objs = [mk_box(p, c, [0.025]*3) for p, c in zip(POS_A, colors)]

    bot = UR3()
    while 1:
        print("\n>>> A -> B")
        for i in range(3):
            c = bot.pick(objs[i]); bot.home()
            bot.place(c, POS_B[i], objs[i]); bot.home()
            
        print("\n>>> B -> A")
        for i in range(3):
            c = bot.pick(objs[i]); bot.home()
            bot.place(c, POS_A[i], objs[i]); bot.home()

if __name__ == "__main__": main()