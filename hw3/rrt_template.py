import numpy as np
from pybullet_tools.more_utils import load_env, get_collision_fn_PR2, execute_trajectory, draw_line
from pybullet_tools.utils import connect, disconnect, wait_if_gui, wait_for_user, joint_from_name, get_joint_positions, \
    set_joint_positions, get_joint_info, get_link_pose, link_from_name, wait_for_duration
import random
import time

class RRT_Connect(object):

    def __init__(self, start_config, goal_config, joint_limits, collision_fn,  eps, goal_bias):

        self.start_config = start_config
        self.goal_config = goal_config

        self.joint_limits = joint_limits
        self.collision_fn = collision_fn

        self.eps = eps
        self.goal_bias = goal_bias


        self.tree = {start_config: start_config}


    def find_nearest(self, q_rand):

        nodes = np.array(list(self.tree.keys()))
        diffs = nodes - np.array(q_rand)
        distance = np.linalg.norm(diffs, axis=1)
        indx = np.argmin(distance)
        q_near = tuple(nodes[indx])

        return q_near


    def connect_tree(self, q_rand):

        q_near = self.find_nearest(q_rand)

        while True:

            direction = np.array(q_rand) - np.array(q_near)
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
                self.tree[q_new] = q_near
                if q_new == q_rand:
                    flag = "Reached"
                else:
                    flag = "Advanced"
            q_near = q_new

            if flag != "Advanced":
                return q_new


    def sample(self):

        mask = np.random.rand()

        if mask <= self.goal_bias:
            q_rand = self.goal_config
        else:
            q_rand = []
            for i in self.joint_limits:
                joint_s = random.uniform(self.joint_limits[i][0], self.joint_limits[i][1])
                q_rand.append(round(joint_s, 3))

            q_rand = np.around(q_rand, decimals=3)
            q_rand = tuple(q_rand)

        return q_rand


    def reconstruct_path(self):

        path = []
        config = self.goal_config
        while config != self.start_config:
            path.append(config)
            parent_config = self.tree[config]
            config = parent_config

        path.append(self.start_config)
        path.reverse()

        return path


    def execute(self):

        while True:
            q_rand = self.sample()
            q_new = self.connect_tree(q_rand)

            if q_new == self.goal_config:
                print("Reach Goal!")
                path = self.reconstruct_path()

                return path


def draw_path(path, robot, joint_idx, start_config, target_link, line_width=25, line_color=(1,0,0)):

    target_link_idx = link_from_name(robot, target_link)

    set_joint_positions(robot, joint_idx, start_config)
    end_effector_path = [get_link_pose(robot, target_link_idx)[0]]

    for config in path:
        set_joint_positions(robot, joint_idx, config)
        wait_for_duration(0.1)
        end_effector_path.append(get_link_pose(robot, target_link_idx)[0])
        draw_line(end_effector_path[-2], end_effector_path[-1], line_width, line_color)


def shortpath_smoothing(path, collision_fn, num_iter=200):

    for i in range(num_iter):
        num_points = len(path)
        way_points = np.random.randint(num_points, size=2)

        diff = np.array(path[way_points[1]]) - np.array(path[way_points[0]])

        for step in np.linspace(0, 1, num=20):
            new_point = np.array(path[way_points[0]]) + step * diff
            if collision_fn(new_point):
                break
        if step == 1:
            del path[way_points[0]+1:way_points[1]]

    return path

def tuckarms(robot):
    _joint_names = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint',
                    'r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
    joint_idx = [joint_from_name(robot, jn) for jn in _joint_names]
    set_joint_positions(robot, joint_idx, [1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996])


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


    ### YOUR CODE HERE ###

    eps = 0.1
    goal_bias = 0.31
    planner = RRT_Connect(start_config, goal_config, joint_limits, collision_fn, eps, goal_bias)

    start_time = time.time()
    path = planner.execute()
    print("Planner run time: ", time.time() - start_time)

    # Execute planned path
    target_link = "l_gripper_tool_frame"
    draw_path(path, robot, joint_idx, start_config, target_link, line_color=(1,0,0))

    smoothing = True
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
