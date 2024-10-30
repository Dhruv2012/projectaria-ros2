#!/usr/bin/env python3

from eve.constants import (
    DT_DURATION,
    FOLLOWER_GRIPPER_JOINT_CLOSE,
    LEADER2FOLLOWER_JOINT_FN,
    LEADER_GRIPPER_CLOSE_THRESH,
    LEADER_GRIPPER_JOINT_MID,
    START_ARM_POSE,
)
from eve.robot_utils import (
    get_arm_gripper_positions,
    move_arms,
    move_grippers,
    torque_off,
    torque_on,
)
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    get_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
import rclpy
import argparse


def opening_ceremony(
    leader_bot_left: InterbotixManipulatorXS,
    leader_bot_right: InterbotixManipulatorXS,
    follower_bot_left: InterbotixManipulatorXS,
    follower_bot_right: InterbotixManipulatorXS,
    active_arms: str,
) -> None:
    """Move the active robots to a pose where it is easy to start demonstration."""
    if active_arms in ["left", "both"]:
        # reboot gripper motors, and set operating modes for left motors
        follower_bot_left.core.robot_reboot_motors('single', 'gripper', True)
        follower_bot_left.core.robot_set_operating_modes('group', 'arm', 'position')
        follower_bot_left.core.robot_set_operating_modes('single', 'gripper', 'current_based_position')
        leader_bot_left.core.robot_set_operating_modes('group', 'arm', 'position')
        leader_bot_left.core.robot_set_operating_modes('single', 'gripper', 'position')
        follower_bot_left.core.robot_set_motor_registers('single', 'gripper', 'current_limit', 200)
        torque_on(follower_bot_left)
        torque_on(leader_bot_left)

    if active_arms in ["right", "both"]:
        # reboot gripper motors, and set operating modes for right motors
        follower_bot_right.core.robot_reboot_motors('single', 'gripper', True)
        follower_bot_right.core.robot_set_operating_modes('group', 'arm', 'position')
        follower_bot_right.core.robot_set_operating_modes('single', 'gripper', 'current_based_position')
        leader_bot_right.core.robot_set_operating_modes('group', 'arm', 'position')
        leader_bot_right.core.robot_set_operating_modes('single', 'gripper', 'position')
        follower_bot_right.core.robot_set_motor_registers('single', 'gripper', 'current_limit', 200)
        torque_on(follower_bot_right)
        torque_on(leader_bot_right)

    # move arms to starting position
    start_arm_qpos = START_ARM_POSE[:6]
    if active_arms == "left":
        move_arms(
            [leader_bot_left, follower_bot_left],
            [start_arm_qpos] * 2,
            moving_time=4.0,
        )
        move_grippers(
            [leader_bot_left, follower_bot_left],
            [LEADER_GRIPPER_JOINT_MID, FOLLOWER_GRIPPER_JOINT_CLOSE],
            moving_time=0.5
        )
    elif active_arms == "right":
        move_arms(
            [leader_bot_right, follower_bot_right],
            [start_arm_qpos] * 2,
            moving_time=4.0,
        )
        move_grippers(
            [leader_bot_right, follower_bot_right],
            [LEADER_GRIPPER_JOINT_MID, FOLLOWER_GRIPPER_JOINT_CLOSE],
            moving_time=0.5
        )
    elif active_arms == "both":
        move_arms(
            [leader_bot_left, follower_bot_left, leader_bot_right, follower_bot_right],
            [start_arm_qpos] * 4,
            moving_time=4.0,
        )
        move_grippers(
            [leader_bot_left, follower_bot_left, leader_bot_right, follower_bot_right],
            [LEADER_GRIPPER_JOINT_MID, FOLLOWER_GRIPPER_JOINT_CLOSE] * 2,
            moving_time=0.5
        )


def press_to_start(
    leader_bot_left: InterbotixManipulatorXS,
    leader_bot_right: InterbotixManipulatorXS,
    active_arms: str,
) -> None:
    # press gripper to start teleop
    # disable torque for only gripper joint of leader robot to allow user movement
    if active_arms in ["left", "both"]:
        leader_bot_left.core.robot_torque_enable('single', 'gripper', False)
    if active_arms in ["right", "both"]:
        leader_bot_right.core.robot_torque_enable('single', 'gripper', False)
    print('Close the grippers to start')
    pressed = False
    while rclpy.ok() and not pressed:
        if active_arms == "left":
            pressed = (get_arm_gripper_positions(leader_bot_left) < LEADER_GRIPPER_CLOSE_THRESH)
        elif active_arms == "right":
            pressed = (get_arm_gripper_positions(leader_bot_right) < LEADER_GRIPPER_CLOSE_THRESH)
        elif active_arms == "both":
            pressed = (
                (get_arm_gripper_positions(leader_bot_left) < LEADER_GRIPPER_CLOSE_THRESH) and
                (get_arm_gripper_positions(leader_bot_right) < LEADER_GRIPPER_CLOSE_THRESH)
            )
        get_interbotix_global_node().get_clock().sleep_for(DT_DURATION)
    if active_arms in ["left", "both"]:
        torque_off(leader_bot_left)
    if active_arms in ["right", "both"]:
        torque_off(leader_bot_right)
    print('Started!')


def main(args) -> None:
    node = create_interbotix_global_node('eve')
    follower_bot_right = None
    leader_bot_right = None
    follower_bot_left = None
    leader_bot_left = None

    if args.arm in ["left", "both"]:
        follower_bot_left = InterbotixManipulatorXS(
            robot_model='vx300s',
            robot_name='follower_left',
            node=node,
            iterative_update_fk=False,
        )

        leader_bot_left = InterbotixManipulatorXS(
            robot_model='wx250s',
            robot_name='leader_left',
            node=node,
            iterative_update_fk=False,
        )

    
    if args.arm in ["right", "both"]:
        follower_bot_right = InterbotixManipulatorXS(
            robot_model='vx300s',
            robot_name='follower_right',
            node=node,
            iterative_update_fk=False,
        )
        
        leader_bot_right = InterbotixManipulatorXS(
            robot_model='wx250s',
            robot_name='leader_right',
            node=node,
            iterative_update_fk=False,
        )


    active_arms = args.arm

    robot_startup(node)

    opening_ceremony(
        leader_bot_left,
        leader_bot_right,
        follower_bot_left,
        follower_bot_right,
        active_arms,
    )

    press_to_start(leader_bot_left, leader_bot_right, active_arms)

    # Teleoperation loop
    gripper_left_command = JointSingleCommand(name='gripper')
    gripper_right_command = JointSingleCommand(name='gripper')
    while rclpy.ok():
        # sync joint positions
        if active_arms in ["left", "both"]:
            leader_left_state_joints = leader_bot_left.core.joint_states.position[:6]
            follower_bot_left.arm.set_joint_positions(leader_left_state_joints, blocking=False)
            # sync left gripper position
            gripper_left_command.cmd = LEADER2FOLLOWER_JOINT_FN(
                leader_bot_left.core.joint_states.position[6]
            )
            follower_bot_left.gripper.core.pub_single.publish(gripper_left_command)

        if active_arms in ["right", "both"]:
            leader_right_state_joints = leader_bot_right.core.joint_states.position[:6]
            follower_bot_right.arm.set_joint_positions(leader_right_state_joints, blocking=False)
            # sync right gripper position
            gripper_right_command.cmd = LEADER2FOLLOWER_JOINT_FN(
                leader_bot_right.core.joint_states.position[6]
            )
            follower_bot_right.gripper.core.pub_single.publish(gripper_right_command)

        # print(
        #     f"Left leader: {leader_bot_left.core.joint_states.position[6]}" if active_arms in ["left", "both"] else "",
        #     f"Right Leader: {leader_bot_right.core.joint_states.position[6]}" if active_arms in ["right", "both"] else "",
        #     f"Left Follower: {follower_bot_left.core.joint_states.position[6]}" if active_arms in ["left", "both"] else "",
        #     f"Right Follower: {follower_bot_right.core.joint_states.position[6]}" if active_arms in ["right", "both"] else ""
        # )

        # sleep DT
        get_interbotix_global_node().get_clock().sleep_for(DT_DURATION)

        if active_arms == "right" and follower_bot_right.core.joint_states.position[0] > 1.5:
            break
        elif active_arms == "left" and follower_bot_left.core.joint_states.position[0] < -1.5:
            break
        elif active_arms == "both" and (follower_bot_right.core.joint_states.position[0] > 1.5 or follower_bot_left.core.joint_states.position[0] < -1.5):
            break

    robot_shutdown(node)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--arm',
        type=str,
        default="both"
    )
    args = parser.parse_args()
    main(args)
