import json
from pybullet_tools.parse_json import parse_robot, parse_body
from pybullet_tools.utils import set_joint_positions, \
    wait_if_gui, wait_for_duration, get_collision_fn, \
    joint_from_name, link_from_name, get_joint_info, HideOutput, \
    get_com_pose, wait_for_duration
from pybullet_tools.pr2_utils import get_disabled_collisions
from pybullet_tools.transformations import quaternion_matrix
import pybullet as p
import numpy as np

def load_env(env_file):
    # load robot and obstacles defined in a json file
    with open(env_file, 'r') as f:
        env_json = json.loads(f.read())
    robots = {robot['name']: parse_robot(robot) for robot in env_json['robots']}
    bodies = {body['name']: parse_body(body) for body in env_json['bodies']}
    return robots, bodies

def get_collision_fn_PR2(robot, joints, obstacles):
    # check robot collision with environment
    disabled_collisions = get_disabled_collisions(robot)
    return get_collision_fn(robot, joints, obstacles=obstacles, attachments=[], \
        self_collisions=True, disabled_collisions=disabled_collisions)

def execute_trajectory(robot, joints, path, sleep=None):
    # Move the robot according to a given path
    if path is None:
        print('Path is empty')
        return
    print('Executing trajectory')
    for bq in path:
        set_joint_positions(robot, joints, bq)
        if sleep is None:
            wait_if_gui('Continue?')
        else:
            wait_for_duration(sleep)
    print('Finished')

def draw_sphere_marker(position, radius, color):
   vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
   marker_id = p.createMultiBody(basePosition=position, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vs_id)
   return marker_id


def draw_line(start, end, width, color):
    line_id = p.addUserDebugLine(start, end, color, width)
    return line_id
    
def get_joint_axis(robot, joint_idx):
    # returns joint axis in the world frame
    j_info = get_joint_info(robot, joint_idx)
    jt_local_pos, jt_local_orn = j_info.parentFramePos, j_info.parentFrameOrn
    H_L_J = quaternion_matrix(jt_local_orn) # joint transform in parent link CoM frame
    H_L_J[:3, 3] = jt_local_pos
    parent_link_world_pos, parent_link_world_orn = get_com_pose(robot, j_info.parentIndex)
    H_W_L = quaternion_matrix(parent_link_world_orn) # parent link CoM transform in world frame
    H_W_L[:3, 3] = parent_link_world_pos
    H_W_J = np.dot(H_W_L, H_L_J)
    R_W_J = H_W_J[:3, :3]
    joint_axis_local = np.array(j_info.jointAxis)
    joint_axis_world = np.dot(R_W_J, joint_axis_local)
    return joint_axis_world

def get_joint_position(robot, joint_idx):
    # returns joint position in the world frame
    j_info = get_joint_info(robot, joint_idx)
    jt_local_pos, jt_local_orn = j_info.parentFramePos, j_info.parentFrameOrn
    H_L_J = quaternion_matrix(jt_local_orn) # joint transform in parent link CoM frame
    H_L_J[:3, 3] = jt_local_pos
    parent_link_world_pos, parent_link_world_orn = get_com_pose(robot, j_info.parentIndex)
    H_W_L = quaternion_matrix(parent_link_world_orn) # parent link CoM transform in world frame
    H_W_L[:3, 3] = parent_link_world_pos
    H_W_J = np.dot(H_W_L, H_L_J)
    j_world_posi = H_W_J[:3, 3]
    return j_world_posi    
    
