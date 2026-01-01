import pybullet as p
from src.types import Position
import math

PI = math.pi


class PandaRobot:
    def __init__(
        self,
        robot_id: int,
        end_effector_index: int = 11,
        gripper_indices: list[int] = None,
    ):
        self.robot_id = robot_id
        self.end_effector_index = end_effector_index  # gripper
        self.gripper_indices = gripper_indices  # gripper
        if gripper_indices is None:
            self.gripper_indices = [9, 10]  # default

        self.ll = [-2.96, -1.83, -2.96, -3.09, -2.96, -0.08, -2.96]  # lower limits
        self.ul = [2.96, 1.83, 2.96, -0.07, 2.96, 3.82, 2.96]  # upper limits
        self.jr = [5.92, 3.66, 5.92, 3.02, 5.92, 3.9, 5.92]  # joint ranges
        self.rp = [0, -0.215, 0, -2.57, 0, 2.356, 2.356]  # rest poses

        for i in range(7):
            p.resetJointState(self.robot_id, i, self.rp[i])

    def go_home(self) -> None:
        for i in range(7):
            p.setJointMotorControl2(
                self.robot_id,
                i,
                p.POSITION_CONTROL,
                targetPosition=self.rp[i],
                force=500,
            )

    def get_end_effector_pos(self) -> Position:
        state = p.getLinkState(self.robot_id, self.end_effector_index)
        actual_pos = state[4]
        return actual_pos

    def move_to(
        self, target_pos: Position, target_yaw: float = 0.5, speed: float = 2.0
    ) -> None:
        orn = p.getQuaternionFromEuler([PI, 0, target_yaw])
        joint_poses = p.calculateInverseKinematics(
            self.robot_id,
            self.end_effector_index,
            target_pos,
            targetOrientation=orn,
            lowerLimits=self.ll,
            upperLimits=self.ul,
            jointRanges=self.jr,
            restPoses=self.rp,
        )

        for i in range(7):
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_poses[i],
                force=500,
                maxVelocity=speed,
            )

    def open_gripper(self, width=0.08) -> None:
        for i in self.gripper_indices:
            p.setJointMotorControl2(
                self.robot_id, i, p.POSITION_CONTROL, targetPosition=width, force=500
            )

    def close_gripper(self) -> None:
        for i in self.gripper_indices:
            p.setJointMotorControl2(
                self.robot_id, i, p.POSITION_CONTROL, targetPosition=0.02, force=1000
            )
