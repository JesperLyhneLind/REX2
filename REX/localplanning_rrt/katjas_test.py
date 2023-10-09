import numpy as np
from time import sleep
from enum import Enum
import grid_occ as g
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
import robot_models, rrt
import os


class Direction(Enum):
    Left = 1
    Right = 2


#go to box but instead it goes to a point
def go_to_point(start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]   
    vec = np.array([dx, dy])
    z_vector = np.array([0, 1])

    dist = np.linalg.norm(vec)  # distance to the box
    dot = np.dot((vec / dist), z_vector)
    angle = np.degrees(np.arccos(dot))
    angle_sign = np.sign(vec[0])  # 1 is right, -1 is left
    print("angle", angle)
    print("anlgesign", angle_sign)

    # dot = np.dot((vec / dist), z_vector)
    # angle = np.degrees(np.arccos(dot))
    # angle_sign = np.sign(vec[0])  # 1 is right, -1 is left

    if angle_sign == -1:
        # print("angle: ", angle)
        # print("angle_sign: ", angle_sign)
        print("turning left with " + str(angle) + " degrees")
    elif angle_sign == 1:
        # print("angle: ", angle)
        # print("angle_sign: ", angle_sign)
        print("turning right with " + str(angle) + " degrees")
    else:
        print("going straight for" + str(dist) + "unit")
        # print("angle_sign: ", angle_sign)
    
path_res = 1
map = g.GridOccupancyMap(low=(-20, 0), high=(20, 20), res=path_res)
map.populate()

robot = robot_models.PointMassModel(ctrl_range=[-path_res, path_res])   
rrt = rrt.RRT(
start=[0, 0],
goal=[0, 19],
robot_model=robot,
map=map,
expand_dis=5,
path_resolution=path_res,
)

show_animation = True
metadata = dict(title="RRT Test")
writer = FFMpegWriter(fps=2, metadata=metadata)
fig = plt.figure()

with writer.saving(fig, os.getcwd(), 100):
    path = rrt.planning(animation=show_animation, writer=writer)

if path is None:
    print("Cannot find path")
else:
    print("found path!!")
    print("FINAL PATH:", path)
    # Draw final path
    if show_animation:
        rrt.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()
        writer.grab_frame()

    path.reverse() 
    print("path flipped:", path)
    for i in range(len(path)-1):
        print("NOW GOIING")
        go_to_point(path[i], path[i+1])
    
    
    

# go_to_box(angle_sign[0], angle, dist, ids[maxvecidx])

