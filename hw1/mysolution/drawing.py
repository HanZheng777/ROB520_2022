from pybullet_tools.more_utils import load_env, draw_sphere_marker, draw_line
from pybullet_tools.utils import connect, disconnect, wait_if_gui, wait_for_user, get_aabb, get_aabb_edges, get_circle_vertices

"""ROB 520 HW1 - Han Zheng - Jan 24 2022"""

def main(screenshot=False):

    connect(use_gui=True)
    robots, obstacles = load_env('scenes/pr2doorway.json')
    wait_for_user()

    # draw bounding rectangles above each table
    for i in range(6):
        aabb = get_aabb(obstacles["ikeatable{}".format(i+1)])
        edge = get_aabb_edges(aabb)

        upper_edges = [edge[3], edge[4], edge[7], edge[10]]

        line_width = 12
        line_color = (1, 0, 0)
        for p1, p2 in upper_edges:
            draw_line(p1, p2, line_width, line_color)

    wait_for_user()

    # draw 35 blue points in a circle which encompasses the env
    circle_vertices = get_circle_vertices(center=(0, 0, 1.5), radius=5, n=35)
    for point in circle_vertices:
        draw_sphere_marker(point, radius=0.03, color=(0, 0, 1, 1))

    wait_if_gui()
    disconnect()

    # print("checkpoint#1")

if __name__ == '__main__':
    main()