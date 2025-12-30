import pybullet as p

from src.robot import PandaRobot
from src.simulation import SimulationEnv
import config


class Main:
    def __init__(self):
        self.env = SimulationEnv()
        robot_id = self.env.setup()
        self.robot = PandaRobot(robot_id)

    def run(self):
        def wait(seconds: float) -> None:
            steps = int(seconds / config.SIMULATION_STEP)
            for _ in range(steps):
                self.env.step()

        self.env.spawn_n_cubes(4)
        wait(2.0)

        targets_info = []
        for box_id in self.env.box_ids:
            pos = self.env.get_object_position(box_id)
            targets_info.append((box_id, pos[2]))

        targets_info.sort(key=lambda x: x[1], reverse=True)

        targets = [info[0] for info in targets_info]

        for box_id in targets:
            wait(1.0)
            self.robot.go_home()
            self.robot.open_gripper()
            wait(1.0)

            pos, quat = p.getBasePositionAndOrientation(box_id)

            rot_x, rot_y, rot_z = p.getEulerFromQuaternion(quat)

            cube_angle = rot_z

            # hover
            self.robot.move_to((pos[0], pos[1], 0.05), target_yaw=cube_angle)
            wait(1.0)

            for _ in range(50):
                self.env.step()

            # revision
            final_pos, _ = p.getBasePositionAndOrientation(box_id)

            # grasp
            grasp_height = final_pos[2] - 0.008

            self.robot.move_to(
                (final_pos[0], final_pos[1], grasp_height), target_yaw=cube_angle
            )

            wait(1.5)
            self.robot.close_gripper()

            wait(0.5)
            self.robot.move_to((final_pos[0], final_pos[1], 0.3), target_yaw=cube_angle)

            wait(2.5)
            self.robot.move_to(config.DROP_ZONE_POS, target_yaw=0.0)

            wait(0.5)
            self.robot.open_gripper()

            wait(0.5)

        while True:
            self.env.step()


if __name__ == "__main__":
    app = Main()
    app.run()
