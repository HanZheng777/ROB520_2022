import numpy as np
from pybullet_tools.more_utils import get_collision_fn_PR2, load_env, execute_trajectory, draw_sphere_marker
from pybullet_tools.utils import connect, disconnect, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, wait_for_duration, get_pose, set_pose
from pybullet_tools.pr2_utils import PR2_GROUPS
import time
from queue import PriorityQueue
### YOUR IMPORTS HERE ###

#########################

def compute_h(current_config, goal_config, variant=2):

    diff = np.array(current_config) - np.array(goal_config)
    if np.abs(diff[2]) > np.pi:
        diff[2] = 2*np.pi - np.abs(diff[2])
    diff[2] = diff[2] * 0.65
    if variant == 1 or variant == 3:
        h = np.sum(np.abs(diff))
    else:
        h = np.linalg.norm(diff)
    return h

def get_neighbor_weight(variant=2):

    if variant == 1 or variant == 2:
        weight = np.row_stack([np.eye(3), -np.eye(3)])
    else:
        weight = np.array([0, 0, 0])
        for i in [1, -1, 0]:
            for j in [1, -1, 0]:
                for k in [1, -1, 0]:
                    row = np.array([i, j, k])
                    weight = np.row_stack([weight, row])
        weight = np.delete(weight, 0, 0)
        weight = np.delete(weight, -1, 0)

    return weight

def update_config(current_config, weight_row, d_total):

    increment = np.array(d_total) * weight_row
    config = current_config + increment
    if abs(config[2]) >= np.pi :
        config[2] = (config[2] + np.pi) % (2 * np.pi) - np.pi

    config = (round(config[0], 2), round(config[1], 2), config[2])
    return config

def compute_neighbor_dist(neighbor, d_total, variant=2):

    dist = np.zeros(neighbor.shape[0])
    for i in range(len(dist)):
        value = neighbor[i, :] * np.array(d_total)
        value[2] = value[2] * 0.65
        if variant == 1 or variant == 3:
            dist[i] = np.sum(np.abs(value))
        else:
            dist[i] = np.linalg.norm(value)
    return dist


def reconstruct_path(came_from, start_config, goal_config, body, joints):

    path = []
    config = goal_config
    while config != start_config:
        path.append(config)
        parent_config = came_from[config]
        config = parent_config

    path.append(start_config)
    path.reverse()

    for config in path:
        set_joint_positions(body, joints, config)
        draw_position = tuple(config[:2]) + (0,)
        draw_sphere_marker(draw_position, 0.05, (0, 0, 0, 1))
        wait_for_duration(0.1)
    return path


def main(screenshot=False):
    # initialize PyBullet
    connect(use_gui=True)
    # load robot and obstacle resources
    robots, obstacles = load_env('scenes/pr2doorway.json')

    # define active DoFs
    base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]

    collision_fn = get_collision_fn_PR2(robots['pr2'], base_joints, list(obstacles.values()))


    table_poses = []
    for i in range(6):
        table_poses.append(get_pose(obstacles["ikeatable{}".format(i + 1)]))
    # assign new poses
    set_pose(obstacles["ikeatable1"], ((-2, -1.2, 0.74), (0.0, 0.0, 0.4, 0.707)))
    set_pose(obstacles["ikeatable2"], ((-3, 0.5, 0.74), (0.0, 0.0, -0.3, 0.707)))

    set_pose(obstacles["ikeatable3"], ((0.1, -0.8, 0.74), (0.0, 0.0, 0.1, 0.707)))
    set_pose(obstacles["ikeatable4"], ((-1.3, 0.6, 0.74), (0.0, 0.0, -0.2, 0.707)))

    set_pose(obstacles["ikeatable5"], ((1.8, -0.5, 0.74), (0.0, 0.0, 0.5, 0.707)))
    set_pose(obstacles["ikeatable6"], ((2.5, 1.2, 0.74), (0.0, 0.0, -0.4, 0.707)))

    
    start_config = tuple(get_joint_positions(robots['pr2'], base_joints))
    # start_ponint = list(start_config[:2])
    # start_ponint.append(0)
    # draw_sphere_marker(tuple(start_ponint), 0.1, (1, 0, 0, 1))

    goal_config = (2.6, -1.3, -np.pi/2)
    path = []
    start_time = time.time()
    ### YOUR CODE HERE ###

    # start_node = (0, start_config)
    dx = 0.1
    dy = 0.1
    d_theta = np.pi/2
    d_total = [dx, dy, d_theta]
    variant = 2

    # priority = -1
    open_set = PriorityQueue()
    close_set = []
    open_set.put((compute_h(start_config, goal_config, variant), start_config))

    came_from = {start_config: start_config}
    config_table = {0: start_config}
    iter_idx = 0
    g_score = {start_config: 0}
    f_score = {start_config: compute_h(start_config, goal_config, variant)}

    neighbor_weight = get_neighbor_weight(variant)
    neighbor_dist = compute_neighbor_dist(neighbor_weight, d_total)
    num_neighbor = len(neighbor_dist)


    while not open_set.empty():

        # priority -= 1
        # current_idx = open_set.get()[2]
        # current_config = config_table[current_idx]
        current_node = open_set.get()
        current_config = current_node[1]
        # current_idx = current_node[1]
        # close_set[current_idx] = current_config

        if current_config == goal_config:
            # path = reconstruct_path(came_from, current_config, config_table)
            print("Reach Goal")
            # collision_fn(current_config)
            reconstruct_path(came_from, start_config, goal_config, robots['pr2'], base_joints)
            total_cost = g_score[goal_config]
            print("Total cost: {}".format(total_cost))
            break

        for i in range(num_neighbor):

            neighbor_config = update_config(current_config, neighbor_weight[i, :], d_total)

            tentative_g = g_score[current_config] + neighbor_dist[i]

            # if goal_config in g_score.keys():
            #     print("goal is found")
            draw_position = tuple(neighbor_config[:2]) + (0,)

            if neighbor_config not in g_score.keys() or tentative_g < g_score[neighbor_config]:
                if not collision_fn(neighbor_config):
                    # draw_sphere_marker(draw_position, 0.1, (0, 0, 1, 1))
                    iter_idx += 1
                    came_from[neighbor_config] = current_config
                    g_score[neighbor_config] = tentative_g
                    h_score = compute_h(neighbor_config, goal_config, variant)
                    f_score[neighbor_config] = tentative_g + h_score

                    if all(False for items in open_set.queue if items[1] == neighbor_config):
                        open_set.put((tentative_g + h_score, neighbor_config))
                    # print(open_set.qsize())
                # else:
                #     draw_sphere_marker(draw_position, 0.1, (1, 0, 0, 1))


    ######################
    print("Planner run time: ", time.time() - start_time)
    # Execute planned path
    execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)
    # Keep graphics window opened
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()
