import pybullet as p

from src.robot import PandaRobot
from src.simulation import SimulationEnv
import config
import random


random.seed(600)

import math

PI = math.pi


class Main:
    def __init__(self):
        self.env = SimulationEnv()
        robot_id = self.env.setup()
        self.robot = PandaRobot(robot_id)

    def calculate_grasp_z(self, box_id: int) -> float:
        aabb_min, aabb_max = p.getAABB(box_id)

        min_z = aabb_min[2]
        max_z = aabb_max[2]

        delta = 0.03
        return ((min_z + max_z) / 2.0) - delta

    def normalize_angel(self, angle: float) -> float:
        while angle > PI / 4:
            angle -= math.PI / 2
        while angle < -PI / 4:
            angle += math.pi / 2
        return angle

    def run(self):
        def wait(seconds: float) -> None:
            steps = int(seconds / config.SIMULATION_STEP)
            for _ in range(steps):
                self.env.step()

        self.env.spawn_n_cubes(config.N)
        wait(2.0)

        targets_info = []
        for box_id in self.env.box_ids:
            pos = self.env.get_object_position(box_id)
            targets_info.append((box_id, pos[2]))

        targets_info.sort(key=lambda x: x[1], reverse=True)

        targets = [info[0] for info in targets_info]

        for box_id in targets:
            # wait(2.0)
            self.robot.go_home()
            self.robot.open_gripper(width=0.04)
            wait(1.0)

            pos, quat = p.getBasePositionAndOrientation(box_id)

            rot_x, rot_y, rot_z = p.getEulerFromQuaternion(quat)

            cube_angle = self.normalize_angel(rot_z)

            # hover
            self.robot.move_to((pos[0], pos[1], 0.2), target_yaw=cube_angle, speed=2.5)
            wait(0.6)

            # for _ in range(50): self.env.step()

            # revision
            pos_v2, _ = p.getBasePositionAndOrientation(box_id)
            self.robot.move_to(
                (pos_v2[0], pos_v2[1], 0.1), target_yaw=cube_angle, speed=1.0
            )
            wait(0.6)

            # final pos - double check
            final_pos, _ = p.getBasePositionAndOrientation(box_id)

            # grasp
            grasp_height = self.calculate_grasp_z(box_id)  # final_pos[2] - 0.008

            self.robot.move_to(
                (final_pos[0], final_pos[1], grasp_height),
                target_yaw=cube_angle,
                speed=0.5,
            )
            wait(0.8)

            self.robot.close_gripper()
            wait(0.4)

            self.robot.move_to((final_pos[0], final_pos[1], 0.3), target_yaw=cube_angle)
            wait(0.4)

            self.robot.move_to(config.DROP_ZONE_POS, target_yaw=0.0)
            wait(0.6)

            self.robot.open_gripper(width=0.06)
            wait(1.1)

        while True:
            self.env.step()


if __name__ == "__main__":
    app = Main()
    app.run()
