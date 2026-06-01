import pybullet as pb
import pybullet_data
import time
import math

class HuskyManipulator:
    def __init__(self, base_pos=[0, 0, 0]):
        self.husky = pb.loadURDF("husky/husky.urdf", basePosition=base_pos)
        self.kuka = pb.loadURDF("kuka_iiwa/model_free_base.urdf", basePosition=[base_pos[0], base_pos[1], base_pos[2]+0.43])
        self.ee = 6
        self.wheels = [2, 3, 4, 5]
        
        # Stability: Heavy base, High friction
        pb.changeDynamics(self.husky, -1, mass=200.0)
        for w in self.wheels:
            pb.changeDynamics(self.husky, w, lateralFriction=15.0)
        
        # Mount Arm & Filter Collisions
        pb.createConstraint(self.husky, -1, self.kuka, -1, pb.JOINT_FIXED, [0,0,0], [0,0,0.43], [0,0,0])
        for i in range(-1, pb.getNumJoints(self.husky)):
            for j in range(-1, pb.getNumJoints(self.kuka)):
                pb.setCollisionFilterPair(self.husky, self.kuka, i, j, 0)
        
        # IK Constants
        self.ll, self.ul = [-2.96]*7, [2.96]*7
        self.jr, self.rp = [5.92]*7, [0]*7
        self.home_arm()

    def _step(self):
        pb.stepSimulation()
        time.sleep(1/240)

    def drive(self, target_x, timeout=60):  # FIX 1: longer timeout for slow PCs
        print(f"Drive -> X:{target_x:.1f}")
        t0 = time.time()
        settled = 0  # FIX 2: require N consecutive frames inside tolerance before returning

        while time.time() - t0 < timeout:
            pos, orn = pb.getBasePositionAndOrientation(self.husky)
            dx = target_x - pos[0]

            if abs(dx) < 0.05:
                settled += 1
                if settled >= 10:  # ~0.04s of stable arrival before we declare success
                    self.brake()
                    self._wait(30)  # FIX 3: let physics settle before arm moves
                    return True
            else:
                settled = 0

            # Rail Logic: Forward/Back + Heading Correction
            speed = min(abs(dx), 0.8)  # FIX 4: cap speed, smoother on slow PCs
            vel = (1 if dx > 0 else -1) * max(speed, 0.15)
            yaw = pb.getEulerFromQuaternion(orn)[2]
            ang = -2.0 * yaw

            # Skid Steer Mixer
            vl = (vel - ang * 0.256) / 0.17
            vr = (vel + ang * 0.256) / 0.17
            for i, w in enumerate(self.wheels):
                pb.setJointMotorControl2(
                    self.husky, w, pb.VELOCITY_CONTROL,
                    targetVelocity=[vl, vr, vl, vr][i],
                    force=2000  # FIX 5: more torque for heavy base on slow physics
                )
            self._step()

        self.brake()
        return False
    
    def brake(self):
        for w in self.wheels:
            pb.setJointMotorControl2(
                self.husky, w, pb.VELOCITY_CONTROL,
                targetVelocity=0, force=2000  # match drive force
            )

    def _lock_base(self):
        """
        FIX 6: Freeze the base in place while the arm operates.
        Prevents physics drift from moving the Husky during arm motion,
        which was the cause of the 'drives back before placing' bug.
        """
        pos, orn = pb.getBasePositionAndOrientation(self.husky)
        pb.resetBaseVelocity(self.husky, [0, 0, 0], [0, 0, 0])
        return pb.createConstraint(
            self.husky, -1, -1, -1,
            pb.JOINT_FIXED, [0, 0, 0], [0, 0, 0], pos, [0, 0, 0, 1], orn
        )

    def _unlock_base(self, lock_constraint):
        pb.removeConstraint(lock_constraint)

    def move_arm(self, target_xyz, duration=1.0):
        start_xyz = pb.getLinkState(self.kuka, self.ee)[0]
        steps = int(duration * 240)
        orn = pb.getQuaternionFromEuler([math.pi, 0, 0])

        for t in range(steps):
            alpha = t / steps
            curr = [s + (e - s) * alpha for s, e in zip(start_xyz, target_xyz)]
            poses = pb.calculateInverseKinematics(
                self.kuka, self.ee, curr, orn,
                lowerLimits=self.ll, upperLimits=self.ul,
                jointRanges=self.jr, restPoses=self.rp
            )
            for i in range(7):
                pb.setJointMotorControl2(
                    self.kuka, i, pb.POSITION_CONTROL,
                    targetPosition=poses[i], force=500, positionGain=0.3
                )
            self._step()

    def pick(self, obj):
        pos = pb.getBasePositionAndOrientation(obj)[0]
        print(f"Pick [{pos[0]:.2f}]")

        lock = self._lock_base()  # freeze base during arm op
        self.move_arm([pos[0], pos[1], pos[2]+0.2])
        self.move_arm([pos[0], pos[1], pos[2]+0.1])
        c = self._grasp(obj)
        self._wait(50)
        self.move_arm([pos[0], pos[1], pos[2]+0.15])
        self.home_arm()
        self._unlock_base(lock)

        return c

    def place(self, c, pos, obj):
        print(f"Place [{pos[0]:.2f}]")

        lock = self._lock_base()  # freeze base during arm op
        self.move_arm([pos[0], pos[1], pos[2]+0.2])
        self.move_arm([pos[0], pos[1], pos[2]+0.1])
        self._release(c, obj)
        self._wait(50)
        self.move_arm([pos[0], pos[1], pos[2]+0.15])
        self.home_arm()
        self._unlock_base(lock)

    def home_arm(self):
        pos = pb.getBasePositionAndOrientation(self.husky)[0]
        self.move_arm([pos[0], pos[1], pos[2]+0.8], duration=1.5)

    def _grasp(self, obj):
        ee_pos, ee_orn = pb.getLinkState(self.kuka, self.ee)[0:2]
        obj_pos, obj_orn = pb.getBasePositionAndOrientation(obj)
        inv = pb.invertTransform(ee_pos, ee_orn)
        rel = pb.multiplyTransforms(inv[0], inv[1], obj_pos, obj_orn)
        c = pb.createConstraint(
            self.kuka, self.ee, obj, -1,
            pb.JOINT_FIXED, [0, 0, 0], rel[0], [0, 0, 0], rel[1], [0, 0, 0, 1]
        )
        pb.changeConstraint(c, maxForce=10000)
        for i in range(8):
            pb.setCollisionFilterPair(self.kuka, obj, i, -1, 0)
        return c

    def _release(self, c, obj):
        pb.removeConstraint(c)
        for i in range(8):
            pb.setCollisionFilterPair(self.kuka, obj, i, -1, 1)

    def _wait(self, steps):
        for _ in range(steps):
            self._step()

def main():
    pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -9.81)
    pb.loadURDF("plane.urdf")
    pb.resetDebugVisualizerCamera(6.0, 0, -45, [3.0, 0, 0.3])
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

    def mk_tray(x, c):
        b = pb.createMultiBody(
            0,
            pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[0.15, 0.15, 0.25]),
            pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.15, 0.15, 0.25], rgbaColor=c),
            [x, 0.7, 0.25]
        )
        pb.changeDynamics(b, -1, lateralFriction=1.0)

    mk_tray(0.5, [0.8, 0.8, 0.8, 1])
    mk_tray(5.5, [0.3, 0.3, 0.3, 1])

    dobjs = []
    offsets = [-0.1, 0, 0.1]
    for i, c in enumerate([[0.9, 0.3, 0.1, 1], [0.1, 0.7, 0.3, 1], [0.1, 0.3, 0.9, 1]]):
        o = pb.createMultiBody(
            0.5,
            pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[0.025]*3),
            pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.025]*3, rgbaColor=c),
            [0.5, 0.7+offsets[i], 0.525]
        )
        pb.changeDynamics(o, -1, lateralFriction=1.0, rollingFriction=0.01, spinningFriction=0.01)
        dobjs.append(o)

    bot=HuskyManipulator()

    while True:
        print("\n>>> CYCLE: Source -> Sink")
        for i in range(3):
            if not bot.drive(0.5): break
            c = bot.pick(dobjs[i])
            if not bot.drive(5.5): break
            bot.place(c, [5.5, 0.7+offsets[i], 0.525], dobjs[i])

        print("\n>>> CYCLE: Sink -> Source")
        for i in range(3):
            if not bot.drive(5.5): break
            c = bot.pick(dobjs[i])
            if not bot.drive(0.5): break
            bot.place(c, [0.5, 0.7+offsets[i], 0.525], dobjs[i])

if __name__ == "__main__":
    main()