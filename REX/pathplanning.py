import grids as g
import matplotlib.pyplot as plt

map = g.GridOccupancyMap()
map.populate()
plt.clf()
map.draw_map()
plt.show() 
