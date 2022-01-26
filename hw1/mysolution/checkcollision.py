import numpy as np
from pybullet_tools.more_utils import load_env, get_collision_fn_PR2, execute_trajectory
from pybullet_tools.utils import connect,  wait_for_user, wait_if_gui, disconnect, joint_from_name, set_pose

"""ROB 520 HW1 - Han Zheng - Jan 24 2022"""

joint_names =('l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint',
              'l_forearm_roll_joint','l_wrist_flex_joint')

def main(screenshot=False):

    connect(use_gui=True)
    robots, obstacles = load_env('scenes/pr2doorway.json')
    wait_for_user()
    PR2 = robots['pr2']

    joint_names =('l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint',
                  'l_forearm_roll_joint','l_wrist_flex_joint')
    joint_idx = [joint_from_name(robots['pr2'], jn) for jn in joint_names]

    # set up collision function
    collision_fn = get_collision_fn_PR2(robots['pr2'], joint_idx, list(obstacles.values()))

    # set PR2 close to the table
    PR2_pose_n = ((1.2, 0.3, 0), (0.0, 0.0, 0.0, 1))
    set_pose(PR2, PR2_pose_n)
    wait_for_user()

    # move left arm to collision
    robot_config_collide = (1.2, 0.2, 1.5, -1.5, -1.320, -0.193)
    print("Robot in collision? ", "yes" if collision_fn(robot_config_collide) else "no")

    execute_trajectory(PR2, joint_idx, [robot_config_collide], sleep=0.1)

    wait_if_gui()
    disconnect()

    # print("checkpoint#1")

if __name__ == '__main__':
    main()