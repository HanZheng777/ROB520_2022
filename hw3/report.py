import numpy as np
from pybullet_tools.more_utils import load_env, get_collision_fn_PR2, execute_trajectory, draw_line
from pybullet_tools.utils import connect, disconnect, wait_if_gui, wait_for_user, joint_from_name, get_joint_positions, \
    set_joint_positions, get_joint_info, get_link_pose, link_from_name, wait_for_duration
import random
from rrt_template import tuckarms, RRT_Connect, draw_path
from birrt_template import BiRRT_Connect
import time
import pandas as pd
import matplotlib.pyplot as plt


def compute_path_length(path, smoothing):
    path_length = 0
    for i in range(len(path) - 1):
        length = np.sum(np.abs(np.array(path[i + 1]) - np.array(path[i])))
        path_length += length
    print("path length:{}; smoothing: {}".format(path_length, smoothing))

    return path_length


def shortpath_smoothing(path, collision_fn, num_iter=200):

    # path_length_all = []

    idxs = np.linspace(1, num_iter, num=num_iter)
    for i in idxs:
        num_points = len(path)
        way_points = np.random.randint(num_points, size=2)

        diff = np.array(path[way_points[1]]) - np.array(path[way_points[0]])

        for step in np.linspace(0, 1, num=20):
            new_point = np.array(path[way_points[0]]) + step * diff
            if collision_fn(new_point):
                break
        if step == 1:
            del path[way_points[0] + 1:way_points[1]]
        # path_length_all.append(compute_path_length(path, smoothing=True))

    # plt.plot(idxs, path_length_all)
    # plt.title("path length vs. smoothing iteration")
    # plt.xlabel("iteration")
    # plt.ylabel("path length")
    # plt.savefig("fig_2")
    # plt.show()

    return path

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
    goal_bias = 0.31

    # run = np.linspace(1,30, num=30)
    RRT_t = []
    smoothing_t = []
    number_nodes_sampled = []
    unsmoothed_l = []
    smoothed_l = []

    for i in range(30):
        planner = BiRRT_Connect(start_config, goal_config, joint_limits, collision_fn, eps)
        start_time = time.time()
        path = planner.execute()
        RRT_t.append(time.time()-start_time)
        unsmoothed_l.append(compute_path_length(path, smoothing=False))

        number_nodes_sampled.append(len(planner.tree_A) + len(planner.tree_B))

        start_time = time.time()
        smoothed_path = shortpath_smoothing(path, collision_fn, num_iter=200)
        smoothing_t.append(time.time() - start_time)
        smoothed_l.append(compute_path_length(smoothed_path, smoothing=True))

    data = {"RRT_t": RRT_t,
            "unsmoothed_l": unsmoothed_l,
            "number_nodes_sampled": number_nodes_sampled,
            }
    df = pd.DataFrame(data)
    df.to_csv("BiRRT_data")

    plt.hist(RRT_t)
    plt.title("BiRRT run time distribution")
    plt.ylabel("count")
    plt.xlabel("run time (s)")
    plt.savefig("fig_4")
    plt.show()

    # Execute planned path
    # target_link = "l_gripper_tool_frame"
    # draw_path(path, robot, joint_idx, start_config, target_link, line_color=(1, 0, 0))
    #
    # smoothing = True
    # if smoothing:
    #     smoothed_path = shortpath_smoothing(path, collision_fn, num_iter=200)
    #     draw_path(smoothed_path, robot, joint_idx, start_config, target_link, line_color=(0, 0, 1))

    ######################
    # execute_trajectory(robot, joint_idx, path, sleep=0.1)
    # Keep graphics window opened
    wait_if_gui()
    disconnect()



if __name__ == '__main__':
    main()