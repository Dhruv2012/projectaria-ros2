import collections
import time

from eve.constants import (
    DT,
    FOLLOWER_GRIPPER_JOINT_CLOSE,
    FOLLOWER_GRIPPER_JOINT_OPEN,
    FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN,
    FOLLOWER_GRIPPER_POSITION_NORMALIZE_FN,
    FOLLOWER_GRIPPER_VELOCITY_NORMALIZE_FN,
    IS_MOBILE,
    LEADER_GRIPPER_JOINT_NORMALIZE_FN,
    START_ARM_POSE,
)
from eve.robot_utils import (
    ImageRecorder,
    move_arms,
    move_grippers,
    Recorder,
    setup_follower_bot,
    setup_leader_bot,
)
import dm_env
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    get_interbotix_global_node,
    InterbotixRobotNode,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
# from interbotix_xs_modules.xs_robot.slate import InterbotixSlate
from interbotix_xs_msgs.msg import JointSingleCommand
import matplotlib.pyplot as plt
import numpy as np


class RealEnv:
    """
    Environment for real robot bi-manual manipulation.

    Action space: [
        left_arm_qpos (6),             # absolute joint position
        left_gripper_positions (1),    # normalized gripper position (0: close, 1: open)
        right_arm_qpos (6),            # absolute joint position
        right_gripper_positions (1),   # normalized gripper position (0: close, 1: open)
    ]

    Observation space: {
        "qpos": Concat[
            left_arm_qpos (6),          # absolute joint position
            left_gripper_position (1),  # normalized gripper position (0: close, 1: open)
            right_arm_qpos (6),         # absolute joint position
            right_gripper_qpos (1)      # normalized gripper position (0: close, 1: open)
        ]
        "qvel": Concat[
            left_arm_qvel (6),          # absolute joint velocity (rad)
            left_gripper_velocity (1),  # normalized gripper velocity (pos: opening, neg: closing)
            right_arm_qvel (6),         # absolute joint velocity (rad)
            right_gripper_qvel (1)      # normalized gripper velocity (pos: opening, neg: closing)
        ]
        "images": {
            "cam_high": (480x640x3),        # h, w, c, dtype='uint8'
            "cam_low": (480x640x3),         # h, w, c, dtype='uint8'
            "cam_left_wrist": (480x640x3),  # h, w, c, dtype='uint8'
            "cam_right_wrist": (480x640x3)  # h, w, c, dtype='uint8'
        }
    """

    def __init__(
        self,
        node: InterbotixRobotNode,
        active_arms: str = "both",
        setup_robots: bool = True,
        setup_base: bool = False,
        is_mobile: bool = IS_MOBILE,
        torque_base: bool = False,
    ):
        """Initialize the Real Robot Environment

        :param node: The InterbotixRobotNode to build the Interbotix API on
        :param setup_robots: True to run through the arm setup process on init, defaults to True
        :param setup_base: True to run through the base setup process on init, defaults to False
        :param is_mobile: True to initialize the Mobile ALOHA environment, False for the Stationary
            ALOHA environment, defaults to IS_MOBILE
        :param torque_base: True to torque the base on after setup, False otherwise, defaults to
            True. Only applies when IS_MOBILE is True
        :raises ValueError: On providing False for setup_base but the robot is not mobile
        """
        self.active_arms = active_arms
        self.follower_bot_left = InterbotixManipulatorXS(
            robot_model='vx300s',
            group_name='arm',
            gripper_name='gripper',
            robot_name='follower_left',
            node=node,
            iterative_update_fk=False,
        ) if active_arms in ["left", "both"] else None

        self.follower_bot_right = InterbotixManipulatorXS(
            robot_model='vx300s',
            group_name='arm',
            gripper_name='gripper',
            robot_name='follower_right',
            node=node,
            iterative_update_fk=False,
        ) if active_arms in ["right", "both"] else None

        self.recorder_left = Recorder('left', node=node) if active_arms in ["left", "both"] else None
        self.recorder_right = Recorder('right', node=node) if active_arms in ["right", "both"] else None
        self.image_recorder = ImageRecorder(node=node, is_mobile=IS_MOBILE)
        self.gripper_command = JointSingleCommand(name='gripper')

        if setup_robots:
            self.setup_robots()

        if setup_base:
            if is_mobile:
                self.setup_base(node, torque_base)
            else:
                raise ValueError((
                    'Requested to set up base but robot is not mobile. '
                    "Hint: check the 'IS_MOBILE' constant."
                ))

    def setup_base(self, node: InterbotixRobotNode, torque_enable: bool = False):
        """Create and configure the SLATE base node

        :param node: The InterbotixRobotNode to build the SLATE base module on
        :param torque_enable: True to torque the base on setup, defaults to False
        """
        self.base = InterbotixSlate(
            'eve',
            node=node,
        )
        self.base.base.set_motor_torque(torque_enable)

    def setup_robots(self):
        if self.active_arms in ["left", "both"]:
            setup_follower_bot(self.follower_bot_left)
        if self.active_arms in ["right", "both"]:
            setup_follower_bot(self.follower_bot_right)

    def get_qpos(self):
        left_qpos_raw = self.recorder_left.qpos if self.recorder_left else [0] * 9
        right_qpos_raw = self.recorder_right.qpos if self.recorder_right else [0] * 9
        left_arm_qpos = left_qpos_raw[:6]
        right_arm_qpos = right_qpos_raw[:6]
        left_gripper_qpos = [FOLLOWER_GRIPPER_POSITION_NORMALIZE_FN(left_qpos_raw[7])]
        right_gripper_qpos = [FOLLOWER_GRIPPER_POSITION_NORMALIZE_FN(right_qpos_raw[7])]
        return np.concatenate(
            [left_arm_qpos, left_gripper_qpos, right_arm_qpos, right_gripper_qpos]
        )

    def get_qvel(self):
        left_qvel_raw = self.recorder_left.qvel if self.recorder_left else [0] * 9
        right_qvel_raw = self.recorder_right.qvel if self.recorder_right else [0] * 9
        left_arm_qvel = left_qvel_raw[:6]
        right_arm_qvel = right_qvel_raw[:6]
        left_gripper_qvel = [FOLLOWER_GRIPPER_VELOCITY_NORMALIZE_FN(left_qvel_raw[7])]
        right_gripper_qvel = [FOLLOWER_GRIPPER_VELOCITY_NORMALIZE_FN(right_qvel_raw[7])]
        return np.concatenate(
            [left_arm_qvel, left_gripper_qvel, right_arm_qvel, right_gripper_qvel]
        )

    def get_effort(self):
        left_effort_raw = self.recorder_left.effort if self.recorder_left else [0] * 9
        right_effort_raw = self.recorder_right.effort if self.recorder_right else [0] * 9
        left_robot_effort = left_effort_raw[:7]
        right_robot_effort = right_effort_raw[:7]
        return np.concatenate([left_robot_effort, right_robot_effort])

    def get_images(self):
        return self.image_recorder.get_images()

    def get_base_vel(self):
        linear_vel = self.base.base.get_linear_velocity().x
        angular_vel = self.base.base.get_angular_velocity().z
        return np.array([linear_vel, angular_vel])

    def set_gripper_pose(
        self,
        left_gripper_desired_pos_normalized=None,
        right_gripper_desired_pos_normalized=None
    ):
        if self.active_arms in ["left", "both"] and left_gripper_desired_pos_normalized is not None:
            left_gripper_desired_joint = FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN(
                left_gripper_desired_pos_normalized
            )
            self.gripper_command.cmd = left_gripper_desired_joint
            self.follower_bot_left.gripper.core.pub_single.publish(self.gripper_command)

        if self.active_arms in ["right", "both"] and right_gripper_desired_pos_normalized is not None:
            right_gripper_desired_joint = FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN(
                right_gripper_desired_pos_normalized
            )
            self.gripper_command.cmd = right_gripper_desired_joint
            self.follower_bot_right.gripper.core.pub_single.publish(self.gripper_command)

    def _reset_joints(self):
        reset_position = START_ARM_POSE[:6]
        if self.active_arms in ["left", "both"]:
            move_arms([self.follower_bot_left], [reset_position], moving_time=2.0)
        if self.active_arms in ["right", "both"]:
            move_arms([self.follower_bot_right], [reset_position], moving_time=2.0)

    def _reset_gripper(self):
        if self.active_arms in ["left", "both"]:
            move_grippers([self.follower_bot_left], [FOLLOWER_GRIPPER_JOINT_OPEN], moving_time=1.0)
            move_grippers([self.follower_bot_left], [FOLLOWER_GRIPPER_JOINT_CLOSE], moving_time=1.0)
        if self.active_arms in ["right", "both"]:
            move_grippers([self.follower_bot_right], [FOLLOWER_GRIPPER_JOINT_OPEN], moving_time=1.0)
            move_grippers([self.follower_bot_right], [FOLLOWER_GRIPPER_JOINT_CLOSE], moving_time=1.0)

    def get_observation(self, get_base_vel=False):
        obs = collections.OrderedDict()
        obs['qpos'] = self.get_qpos()
        obs['qvel'] = self.get_qvel()
        obs['effort'] = self.get_effort()
        obs['images'] = self.get_images()
        if get_base_vel:
            obs['base_vel'] = self.get_base_vel()
        return obs

    def get_reward(self):
        return 0

    def reset(self, fake=False):
        if not fake:
            if self.active_arms in ["left", "both"]:
                self.follower_bot_left.core.robot_reboot_motors('single', 'gripper', True)
            if self.active_arms in ["right", "both"]:
                self.follower_bot_right.core.robot_reboot_motors('single', 'gripper', True)
            self._reset_joints()
            self._reset_gripper()
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation(),
        )

    def step(self, action, base_action=None, get_base_vel=False, get_obs=True):
        if self.active_arms in ["left", "both"]:
            left_action = action[:7]
            self.follower_bot_left.arm.set_joint_positions(left_action[:6], blocking=False)
            self.set_gripper_pose(left_action[-1], None)
        if self.active_arms in ["right", "both"]:
            right_action = action[-7:]
            self.follower_bot_right.arm.set_joint_positions(right_action[:6], blocking=False)
            self.set_gripper_pose(None, right_action[-1])
        if base_action is not None:
            base_action_linear, base_action_angular = base_action
            self.base.base.command_velocity_xyaw(x=base_action_linear, yaw=base_action_angular)
        if get_obs:
            obs = self.get_observation(get_base_vel)
        else:
            obs = None
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=self.get_reward(),
            discount=None,
            observation=obs)


def get_action(
    leader_bot_left: InterbotixManipulatorXS,
    leader_bot_right: InterbotixManipulatorXS,
    active_arms: str
):
    action = np.zeros(14)  # 6 joint + 1 gripper, for two arms
    if active_arms in ["left", "both"]:
        action[:6] = leader_bot_left.core.joint_states.position[:6]
        action[6] = LEADER_GRIPPER_JOINT_NORMALIZE_FN(leader_bot_left.core.joint_states.position[6])
    if active_arms in ["right", "both"]:
        action[7:7+6] = leader_bot_right.core.joint_states.position[:6]
        action[13] = LEADER_GRIPPER_JOINT_NORMALIZE_FN(leader_bot_right.core.joint_states.position[6])
    return action


def make_real_env(
    node: InterbotixRobotNode = None,
    active_arms: str = "both",
    setup_robots: bool = True,
    setup_base: bool = False,
    torque_base: bool = False,
):
    if node is None:
        node = get_interbotix_global_node()
        if node is None:
            node = create_interbotix_global_node('eve')
    env = RealEnv(
        node=node,
        active_arms=active_arms,
        setup_robots=setup_robots,
        setup_base=setup_base,
        is_mobile=IS_MOBILE,
        torque_base=torque_base,
    )
    return env


def test_real_teleop(active_arms="both"):
    """
    Test bimanual teleoperation and show image observations onscreen.

    It first reads joint poses from both leader arms.
    Then use it as actions to step the environment.
    The environment returns full observations including images.

    An alternative approach is to have separate scripts for teleop and observation recording.
    This script will result in higher fidelity (obs, action) pairs
    """
    onscreen_render = True
    render_cam = 'cam_left_wrist'

    node = get_interbotix_global_node()

    # source of data
    leader_bot_left = InterbotixManipulatorXS(
        robot_model='wx250s',
        robot_name='leader_left',
        node=node,
    ) if active_arms in ["left", "both"] else None

    leader_bot_right = InterbotixManipulatorXS(
        robot_model='wx250s',
        robot_name='leader_right',
        node=node,
    ) if active_arms in ["right", "both"] else None

    if leader_bot_left:
        setup_leader_bot(leader_bot_left)
    if leader_bot_right:
        setup_leader_bot(leader_bot_right)

    # environment setup
    env = make_real_env(node=node, active_arms=active_arms)
    ts = env.reset(fake=True)
    episode = [ts]
    # visualization setup
    if onscreen_render:
        ax = plt.subplot()
        plt_img = ax.imshow(ts.observation['images'][render_cam])
        plt.ion()

    for _ in range(1000):
        action = get_action(leader_bot_left, leader_bot_right, active_arms)
        ts = env.step(action)
        episode.append(ts)

        if onscreen_render:
            plt_img.set_data(ts.observation['images'][render_cam])
            plt.pause(DT)
        else:
            time.sleep(DT)


if __name__ == '__main__':
    test_real_teleop(active_arms="both")  # change "both" to "left" or "right" to test individual arms
