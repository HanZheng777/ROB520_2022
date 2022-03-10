import numpy as np
from pybullet_tools.more_utils import load_env, get_collision_fn_PR2, execute_trajectory, draw_line
from pybullet_tools.utils import connect, disconnect, wait_if_gui, wait_for_user, joint_from_name, get_joint_positions, \
    set_joint_positions, get_joint_info, get_link_pose, link_from_name, wait_for_duration
from rrt_template import RRT_Connect, draw_path, tuckarms, shortpath_smoothing
import random
import time
import copy


class BiRRT_Connect(RRT_Connect):

    def __init__(self, start_config, goal_config, joint_limits, collision_fn, eps, goal_bias=0):

        self.start_config = start_config
        self.goal_config = goal_config

        self.joint_limits = joint_limits
        self.collision_fn = collision_fn

        self.eps = eps
        self.goal_bias = goal_bias

        self.tree_A = {start_config: None}
        self.tree_B = {goal_config: None}


    def swap_trees(self):

        tree_A_copy = copy.deepcopy(self.tree_A)
        self.tree_A = copy.deepcopy(self.tree_B)
        self.tree_B = copy.deepcopy(tree_A_copy)


    def find_nearest(self, tree, q):

        nodes = np.array(list(tree.keys()))
        diffs = nodes - np.array(q)
        distance = np.linalg.norm(diffs, axis=1)
        indx = np.argmin(distance)
        q_near = tuple(nodes[indx])

        return q_near


    def connect_tree(self, tree, q):

        q_near = self.find_nearest(tree, q)

        while True:

            direction = np.array(q) - np.array(q_near)
            direction_length = np.linalg.norm(direction)
            if direction_length <= self.eps:
                step = direction
            else:
                direction_unit = direction / direction_length
                step = self.eps * direction_unit

            q_new = np.around(q_near + step, decimals=3)
            q_new = tuple(q_new)
            if self.collision_fn(q_new):
                flag = "Trapped"
            else:
                tree[q_new] = q_near
                if q_new == q:
                    flag = "Reached"
                else:
                    flag = "Advanced"
            q_near = q_new

            if flag != "Advanced":
                return flag, q_new


    def reconstruct_path(self, connection_point):

        path_A = []
        path_B = []

        if not self.goal_config in self.tree_B:
            self.swap_trees()

        config = connection_point
        while config != self.goal_config:
            path_B.append(config)
            parent_config = self.tree_B[config]
            config = parent_config
        path_B.append(self.goal_config)

        config = connection_point
        while config != self.start_config:
            path_A.append(config)
            parent_config = self.tree_A[config]
            config = parent_config
        path_A.append(self.start_config)
        path_A.reverse()

        path = path_A + path_B

        return path


    def execute(self):

        while True:
            q_rand = self.sample()
            flag_A, q_new_A = self.connect_tree(self.tree_A, q_rand)

            if flag_A != "Trapped":
                flag_B, q_new_B = self.connect_tree(self.tree_B, q_new_A)

                if flag_B == "Reached":
                    print("Find Path!")
                    path = self.reconstruct_path(q_new_B)
                    return path

                self.swap_trees()


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

    start_config = tuple([0.25,0.075,-1.008,0,0,-0.11,0])
    set_joint_positions(robot, joint_idx, start_config)
    
    goal_config = (0.9,-0.201,-0.55,1.5,0,-0.11,0)

    start_time = time.time()
    ### YOUR CODE HERE ###

    eps = 0.1
    planner = BiRRT_Connect(start_config, goal_config, joint_limits, collision_fn, eps)
    path = planner.execute()
    smoothing = True

    print("Planner run time: ", time.time() - start_time)

    # Execute planned path
    target_link = "l_gripper_tool_frame"

    draw_path(path, robot, joint_idx, start_config, target_link, line_color=(1,0,0))
    if smoothing:
        smoothed_path = shortpath_smoothing(path, collision_fn, num_iter=200)
        draw_path(smoothed_path, robot, joint_idx, start_config, target_link, line_color=(0,0,1))

    ######################
    # execute_trajectory(robot, joint_idx, path, sleep=0.1)
    # Keep graphics window opened
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()
