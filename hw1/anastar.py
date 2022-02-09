import numpy as np
from pybullet_tools.more_utils import get_collision_fn_PR2, load_env, execute_trajectory, draw_sphere_marker
from pybullet_tools.utils import connect, disconnect, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, wait_for_duration
from pybullet_tools.pr2_utils import PR2_GROUPS
import time
from queue import PriorityQueue
### YOUR IMPORTS HERE ###

#########################

class ReversePriorityQueue(PriorityQueue):

    def put(self, tup):
        newtup = tup[0] * -1, tup[1], tup[2]
        PriorityQueue.put(self, newtup)

    def get(self):
        tup = PriorityQueue.get(self)
        newtup = tup[0] * -1, tup[1], tup[2]
        return newtup


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


def compute_e(G_global, g, h):

    e = (G_global - g) / (h + 1e-15)
    return e


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


def prune(open_set, G_global, g_score, goal_config, variant):

    update_open_set = ReversePriorityQueue(0)

    while not open_set.empty():

        current_node = open_set.get()
        current_config = current_node[1]
        g_curr = g_score[current_node]
        h_curr = compute_h(current_config, goal_config, variant)

        if g_curr + h_curr < G_global:
            new_e_curr = compute_e(G_global, g_curr, h_curr)
            update_open_set.put((new_e_curr, current_config))

    return update_open_set


def main(screenshot=False):
    # initialize PyBullet
    connect(use_gui=True)
    # load robot and obstacle resources
    robots, obstacles = load_env('scenes/pr2doorway.json')

    # define active DoFs
    base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]

    collision_fn = get_collision_fn_PR2(robots['pr2'], base_joints, list(obstacles.values()))
    start_config = tuple(get_joint_positions(robots['pr2'], base_joints))

    goal_config = (2.6, -1.3, -np.pi/2)
    path = []
    start_time = time.time()

    ### YOUR CODE HERE ###

    dx = 0.1
    dy = 0.1
    d_theta = np.pi/2
    d_total = [dx, dy, d_theta]
    variant = 4

    open_set = ReversePriorityQueue(0)
    open_set.put((compute_h(start_config, goal_config, variant), start_config))

    came_from = {start_config: start_config}
    g_score = {start_config: 0}

    neighbor_weight = get_neighbor_weight(variant)
    neighbor_dist = compute_neighbor_dist(neighbor_weight, d_total)
    num_neighbor = len(neighbor_dist)

    E_global = np.inf
    G_global = np.inf

    while not open_set.empty():

        while not open_set.empty():

            current_node = open_set.get()
            e_curr = current_node[0]
            g_curr = g_score[current_node]
            current_config = current_node[1]

            if e_curr < E_global:
                E_global = e_curr

            if current_config == goal_config:
                G_global = g_curr
                break

            for i in range(num_neighbor):

                neighbor_config = update_config(current_config, neighbor_weight[i, :], d_total)

                tentative_g = g_score[current_config] + neighbor_dist[i]

                draw_position = tuple(neighbor_config[:2]) + (0,)

                if neighbor_config not in g_score.keys() or tentative_g < g_score[neighbor_config]:
                    if not collision_fn(neighbor_config):
                        # draw_sphere_marker(draw_position, 0.1, (0, 0, 1, 1))

                        came_from[neighbor_config] = current_config
                        g_score[neighbor_config] = tentative_g
                        h_neighbor = compute_h(neighbor_config, goal_config, variant)

                        if tentative_g + h_neighbor < G_global:
                            e_neighbor = compute_e(G_global, tentative_g, h_neighbor)
                            open_set.put((e_neighbor, neighbor_config))
                        # print(open_set.qsize())
                    # else:
                    #     draw_sphere_marker(draw_position, 0.1, (1, 0, 0, 1))

        open_set = prune(open_set, G_global, g_score, goal_config, variant)

        reconstruct_path(came_from, start_config, goal_config, robots['pr2'], base_joints)

    ######################
    print("Planner run time: ", time.time() - start_time)
    # Execute planned path
    execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)
    # Keep graphics window opened
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()
