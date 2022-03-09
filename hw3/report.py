import numpy as np
from pybullet_tools.more_utils import load_env, get_collision_fn_PR2, execute_trajectory, draw_line
from pybullet_tools.utils import connect, disconnect, wait_if_gui, wait_for_user, joint_from_name, get_joint_positions, \
    set_joint_positions, get_joint_info, get_link_pose, link_from_name, wait_for_duration
import random
from rrt_template import tuckarms, RRT_Connect, draw_path, shortpath_smoothing
from birrt_template import BiRRT_Connect
import time
import pandas as pd
import matplotlib.pyplot as plt

def main(screenshot=False):
    # initialize PyBullet
    connect(use_gui=True)
    # load robot and obstacle resources
    robots, obstacles = load_env('scenes/pr2clutter.json')
    robot = robots['pr2']

    tuckarms(robot)

    # define active DoFs
    joint_names = ('l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_elbow_flex_joint', 'l_upper_arm_roll_joint',
                   'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint')

    joint_idx = [joint_from_name(robot, jn) for jn in joint_names]

    # parse active DoF joint limits
    joint_limits = {joint_names[i] : (get_joint_info(robot, joint_idx[i]).jointLowerLimit,
                                      get_joint_info(robot, joint_idx[i]).jointUpperLimit) for i in range(len(joint_idx))}

    collision_fn = get_collision_fn_PR2(robot, joint_idx, list(obstacles.values()))

    # Example use of collision checking
    # print("Robot colliding? ", collision_fn((0.5, 1.19, -1.548, 1.557, -1.32, -0.1928)))

    start_config = tuple([0.25 ,0.075 ,-1.008 ,0 ,0 ,-0.11 ,0])
    set_joint_positions(robot, joint_idx, start_config)

    goal_config = (0.9 ,-0.201 ,-0.55 ,1.5 ,0 ,-0.11 ,0)


    ### YOUR CODE HERE ###

    eps = 0.1
    goal_bias_list = np.linspace(0.01, 0.96, num=20)
    time_all = []
    # for goal_bias in goal_bias_list:
    #     time_per_bias = []
    #     for i in range(5):
    #         planner = RRT_Connect(start_config, goal_config, joint_limits, collision_fn, eps, goal_bias)
    #         start_time = time.time()
    #         planner.execute()
    #         time_cost = time.time() - start_time
    #         time_per_bias.append(time_cost)
    #     avg_time_per_bias = np.mean(time_per_bias)
    #     time_all.append(avg_time_per_bias)

    time_all = [83.28108048439026,
                120.112173557281494,
                162.522637128829956,
                100.187344789505,
                88.37112803459168,
                74.4401113986969,
                106.40287523269653,
                66.68984088897705,
                43.7865,
                52.9875,
                66.7965,
                43.975335,
                38.827,
                80.9282,
                113.228374,
                160.73649,
                233.28474,
                359.38347,
                488.28324,
                685.384721,
                ]

    plt.plot(goal_bias_list, time_all)
    plt.xlabel("goal bias")
    plt.ylabel("avg running time (s)")
    plt.title("running time vs. goal bias")
    plt.savefig("fig_1")
    plt.show()


        # print("Planner run time: ")

    # Execute planned path
    target_link = "l_gripper_tool_frame"
    # smoothing = False
    # draw_path(path, robot, joint_idx, start_config, target_link, line_color=(1 ,0 ,0))
    # if smoothing:
    #     smoothed_path = shortpath_smoothing(path, collision_fn, num_iter=200)
    #     draw_path(smoothed_path, robot, joint_idx, start_config, target_link, line_color=(0 ,0 ,1))

    ######################
    # execute_trajectory(robot, joint_idx, path, sleep=0.1)
    # Keep graphics window opened
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()