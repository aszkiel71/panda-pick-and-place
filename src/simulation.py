import pybullet as p
import pybullet_data

import time
import random

import config
from src.types import Position


class SimulationEnv:
    def __init__(self):
        self.client_id = None
        self.box_ids = []
        self.robot_id = None

    def setup(self) -> int:
        # cfg
        self.client_id = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, config.GRAVITY)

        # floor
        p.loadURDF("plane.urdf")

        """
        cinema mode:

        """

        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

        # robot
        self.robot_id = p.loadURDF(
            fileName="franka_panda/panda.urdf",
            basePosition=config.ROBOT_BASE_POS,
            useFixedBase=True,
            globalScaling=1.0,
        )
        p.changeDynamics(
            self.robot_id,
            9,
            lateralFriction=2.0,
            spinningFriction=0.1,
            rollingFriction=0.1,
        )
        p.changeDynamics(
            self.robot_id,
            10,
            lateralFriction=2.0,
            spinningFriction=0.1,
            rollingFriction=0.1,
        )

        # camera
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=50,
            cameraPitch=-35,
            cameraTargetPosition=[0, 0, 0],
        )

        return self.robot_id

    def step(self) -> None:
        p.stepSimulation()
        time.sleep(config.SIMULATION_STEP)

    def spawn_n_cubes(self, n: int = 5) -> None:
        for b_id in self.box_ids:
            p.removeBody(b_id)  # if they already existed
        self.box_ids = []
        for _ in range(n):
            x, y, z = (
                random.uniform(0.3, 0.6),
                random.uniform(-0.2, 0.2),
                random.uniform(0.2, 0.5),
            )
            box_id = p.loadURDF("cube_small.urdf", basePosition=[x, y, z])
            p.changeDynamics(
                box_id,
                -1,
                mass=0.5,
                lateralFriction=1.0,
                spinningFriction=0.001,
                rollingFriction=0.001,
            )
            self.box_ids.append(box_id)

    def get_object_position(self, body_id: int) -> Position:
        pos, _ = p.getBasePositionAndOrientation(body_id)
        return pos
