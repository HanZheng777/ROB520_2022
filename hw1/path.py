from pybullet_tools.more_utils import load_env, get_collision_fn_PR2, execute_trajectory, draw_line
from pybullet_tools.utils import connect,  wait_for_user, wait_if_gui, disconnect, joint_from_name, get_link_pose, link_from_name, set_pose, set_joint_position

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
    PR2_pose_n = ((1, 0.3, 0), (0.0, 0.0, 0.0, 1))
    set_pose(PR2, PR2_pose_n)

    # non-collision initial joint config
    torso_idx = joint_from_name(PR2, "torso_lift_joint")
    set_joint_position(PR2, torso_idx, 0.2)
    wait_for_user()

    # find link to be tracked
    target_link = "l_gripper_tool_frame"
    target_link_idx = link_from_name(PR2, target_link)
    link_path = []
    line_width = 12
    line_color = (1, 0, 0)  # R, G, B

    # move the left arm and draw the track path
    config_0 = [1.2, 1.3, -1, -0.2, -0.5, -0.5]
    execute_trajectory(PR2, joint_idx, [config_0], sleep=0.1)
    print("Robot in collision? ", "yes" if collision_fn(config_0) else "no")
    link_path.append(get_link_pose(PR2, target_link_idx)[0])
    wait_for_user()

    config_rest = [[1.2, 1.3, -1, 1.58, -0.5, -0.5], [0.4, 1.3, -1, 1.58, -0.5, -0.5], [0.4, -0.1, -1, 1.58, -0.5, -0.5],
                 [1.2, -0.1, -1, 1.58, -0.5, -0.5], [1.2, -0.1, -0.2, 1.58, -0.5, -0.5]]

    for config in config_rest:
        execute_trajectory(PR2, joint_idx, [config], sleep=0.1)
        print("Robot in collision? ", "yes" if collision_fn(config) else "no")
        link_path.append(get_link_pose(PR2, target_link_idx)[0])
        draw_line(link_path[-2], link_path[-1], line_width, line_color)
        wait_for_user()


    wait_if_gui()
    disconnect()

    # print("checkpoint#1")

if __name__ == '__main__':
    main()