from pybullet_tools.more_utils import load_env
from pybullet_tools.utils import connect, disconnect, wait_if_gui, wait_for_user,  get_pose, set_pose

"""ROB 520 HW1 - Han Zheng - Jan 24 2022"""

def main(screenshot=False):

    connect(use_gui=True)
    robots, obstacles = load_env('scenes/pr2doorway.json')
    wait_for_user()
    # get poses of the table
    table_poses = []
    for i in range(6):
        table_poses.append(get_pose(obstacles["ikeatable{}".format(i+1)]))
    # assign new poses

#     set_pose(obstacles["ikeatable1"], ((-1.9, 0.1, 0.74), (0.0, 0.0, 0.7, 0.707)))
#     set_pose(obstacles["ikeatable2"], ((-3.5, 0.8, 0.74), (0.0, 0.0, 0.0, 0.707)))

#     set_pose(obstacles["ikeatable3"], ((0.2, -0.8, 0.74), (0.0, 0.0, 0.1, 0.707)))
#     set_pose(obstacles["ikeatable4"], ((-1.7, -0.5, 0.74), (0.0, 0.0, 0.7, 0.707)))

#     set_pose(obstacles["ikeatable5"], ((1.6, -0.5, 0.74), (0.0, 0.0, 0.0, 0.707)))
#     set_pose(obstacles["ikeatable6"], ((3.5, -1.2, 0.74), (0.0, 0.0, 0.0, 0.707)))


    wait_if_gui()
    disconnect()

    #print("checkpoint#1")

if __name__ == '__main__':
    main()
