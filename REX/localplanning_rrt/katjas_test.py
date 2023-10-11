import numpy as np
from time import sleep
from enum import Enum
import grid_occ as g
import matplotlib
import matplotlib.pyplot as plt

from matplotlib.animation import FFMpegWriter

import robot_models, rrt
import os


class Direction(Enum):
    Left = 1
    Right = 2


#go to box but instead it goes to a point
def go_to_point(old, start, end):
    new_vec = np.array([end[0] - start[0], end[1] - start[1]])
    old_vec =  np.array([start[0] - old[0], start[1] - old[1]])


    new_dist = np.linalg.norm(new_vec)  
    old_dist = np.linalg.norm(old_vec)  
    dot = np.dot(new_vec , old_vec)
    angle = np.degrees(np.arccos(dot/(new_dist*old_dist)))

    angle_sign = np.sign(new_vec[0])  # 1 is right, -1 is left
    print("angle", angle)
    print("anlgesign", angle_sign)

    # dot = np.dot((vec / dist), z_vector)
    # angle = np.degrees(np.arccos(dot))
    # angle_sign = np.sign(vec[0])  # 1 is right, -1 is left

    if angle_sign == -1:
        print("turning left with " + str(angle) + " degrees")
    elif angle_sign == 1:
        print("turning right with " + str(angle) + " degrees")
    else:
        print("going straight ")

    
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

with writer.saving(fig, "rrt_test.mp4", 100):
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
        if i == 0:
            go_to_point([0,0], [0,1], path[i+1])
        else:
            go_to_point(path[i-1], path[i], path[i+1])
    
    
    

# go_to_box(angle_sign[0], angle, dist, ids[maxvecidx])

