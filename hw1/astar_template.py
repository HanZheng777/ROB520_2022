import numpy as np
from pybullet_tools.more_utils import get_collision_fn_PR2, load_env, execute_trajectory, draw_sphere_marker
from pybullet_tools.utils import connect, disconnect, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, get_link_pose, link_from_name
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

    return tuple(config)

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


def reconstruct_path(came_from, current_config, close_set):

    path = []
    idx = came_from.indx(current_config)
    while idx != 0:
        pre_idx = came_from[idx]
        pre_config = close_set[pre_idx]
        path.insert(0, pre_config)
        idx = pre_idx

    return path


def main(screenshot=False):
    # initialize PyBullet
    connect(use_gui=True)
    # load robot and obstacle resources
    robots, obstacles = load_env('scenes/pr2doorway.json')

    # define active DoFs
    base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]

    collision_fn = get_collision_fn_PR2(robots['pr2'], base_joints, list(obstacles.values()))
    # Example use of collision checking
    # print("Robot colliding? ", collision_fn((0.5, -1.3, -np.pi/2)))

    # Example use of setting body poses
    # set_pose(obstacles['ikeatable6'], ((0, 0, 0), (1, 0, 0, 0)))

    # Example of draw 
    # draw_sphere_marker((0, 0, 1), 0.1, (1, 0, 0, 1))

    
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
            break

        for i in range(num_neighbor):

            neighbor_config = update_config(current_config, neighbor_weight[i, :], d_total)
            # if neighbor_config not in f_score.keys():
            # if not any(True for v in config_table.values() if v==neighbor_config):

            # draw_position = tuple(neighbor_config[:2]) + (0,)
            # if collision_fn(tuple(neighbor_config)):
            #     # draw_sphere_marker(draw_position, 0.1, (1, 0, 0, 1))
            #     continue
            # else:
            #     draw_sphere_marker(draw_position, 0.1, (0, 0, 1, 1))
                # config_table[node_idx] = tuple(neighbor_config)
            # if
            #     g_score[neighbor_config] = np.inf


                # if not any(True f/or v in config_table.values() if v == neighbor_config):
                #
                # config_table[node_idx] = tuple(neighbor_config)
                # g_score[node_idx] = np.inf

                # if not any(True for node in open_set.queue if node[2] == neighbor_config):
                #     node_idx += 1
                #     g_score[node_idx] = np.inf

            # if
            #     g_score[neighbor_config] = np.inf

            tentative_g = g_score[current_config] + neighbor_dist[i]

            if goal_config in g_score.keys():
                print("fuck, goal is found")

            if neighbor_config not in g_score.keys() or tentative_g < g_score[neighbor_config]:
                if not collision_fn(neighbor_config):
                    iter_idx += 1
                    came_from[neighbor_config] = current_config
                    g_score[neighbor_config] = tentative_g
                    h_score = compute_h(neighbor_config, goal_config, variant)
                    f_score[neighbor_config] = tentative_g + h_score

                    # if all(False for items in open_set.queue if items[2] == neighbor_config):
                    open_set.put((tentative_g + h_score, neighbor_config))
                    # print(open_set.qsize())




    ######################
    print("Planner run time: ", time.time() - start_time)
    # Execute planned path
    execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)
    # Keep graphics window opened
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()
