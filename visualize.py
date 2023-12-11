import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math
import csv

if __name__ == '__main__':
    workspace1 = [
        "POLYGON((1.000000 1.000000,2.000000 1.000000,2.000000 5.000000,1.000000 5.000000,1.000000 1.000000))",
        "POLYGON((3.000000 3.000000,4.000000 3.000000,4.000000 12.000000,3.000000 12.000000,3.000000 3.000000))",
        "POLYGON((3.000000 12.000000,12.000000 12.000000,12.000000 13.000000,3.000000 13.000000,3.000000 12.000000))",
        "POLYGON((12.000000 5.000000,13.000000 5.000000,13.000000 13.000000,12.000000 13.000000,12.000000 5.000000))",
        "POLYGON((6.000000 5.000000,12.000000 5.000000,12.000000 6.000000,6.000000 6.000000,6.000000 5.000000))"
    ]

    workspace2 = [
        "POLYGON((-6.000000 -6.000000,25.000000 -6.000000,25.000000 -5.000000,-6.000000 -5.000000,-6.000000 -6.000000))",
        "POLYGON((-6.000000 5.000000,30.000000 5.000000,30.000000 6.000000,-6.000000 6.000000,-6.000000 5.000000))",
        "POLYGON((-6.000000 -5.000000,-5.000000 -5.000000,-5.000000 5.000000,-6.000000 5.000000,-6.000000 -5.000000))",
        "POLYGON((4.000000 -5.000000,5.000000 -5.000000,5.000000 1.000000,4.000000 1.000000,4.000000 -5.000000))",
        "POLYGON((9.000000 0.000000,10.000000 0.000000,10.000000 5.000000,9.000000 5.000000,9.000000 0.000000))",
        "POLYGON((14.000000 -5.000000,15.000000 -5.000000,15.000000 1.000000,14.000000 1.000000,14.000000 -5.000000))",
        "POLYGON((19.000000 0.000000,20.000000 0.000000,20.000000 5.000000,19.000000 5.000000,19.000000 0.000000))",
        "POLYGON((24.000000 -5.000000,25.000000 -5.000000,25.000000 1.000000,24.000000 1.000000,24.000000 -5.000000))",
        "POLYGON((29.000000 0.000000,30.000000 0.000000,30.000000 5.000000,29.000000 5.000000,29.000000 0.000000))",
    ]

    # Convert polygon strings to lists of obstacles
    obstacles = []
    for poly_str in workspace1:
        coords = poly_str.split('((')[1].split('))')[0].split(',')
        polygon_points = [tuple(map(float, coord.split())) for coord in coords]
        obstacles.append(polygon_points)

    plt.figure()
    ax = plt.gca() 
    ax.set_aspect('equal', adjustable='box')
    for obstacle in obstacles:
        patch = Polygon(obstacle, facecolor="indianred", edgecolor="black", lw=1)
        ax.add_patch(patch)


    ugv_data = []
    with open('solutions/Car_plan.txt', 'r') as file:
        for line in file:
            row = [float(val) for val in line.split()]
            ugv_data.append(row)
    uav_data = []
    with open('solutions/UAV_plan.txt', 'r') as file:
        for line in file:
            row = [float(val) for val in line.split()]
            uav_data.append(row)
    ugv_data.pop()
    uav_data.pop()

    half_w = 1.0 / 2
    half_l = 0.5 / 2
    x_pts = []
    y_pts = []
    for state in ugv_data:
        x_pts.append(ugv_data[0])
        y_pts.append(ugv_data[1])
        vertices = [
            (state[0] + half_w * math.cos(state[4]) - half_l * math.sin(state[4]),
            state[1] + half_w * math.sin(state[4]) + half_l * math.cos(state[4])),

            (state[0] - half_w * math.cos(state[4]) - half_l * math.sin(state[4]),
            state[1] - half_w * math.sin(state[4]) + half_l * math.cos(state[4])),

            (state[0] - half_w * math.cos(state[4]) + half_l * math.sin(state[4]),
            state[1] - half_w * math.sin(state[4]) - half_l * math.cos(state[4])),

            (state[0] + half_w * math.cos(state[4]) + half_l * math.sin(state[4]),
            state[1] + half_w * math.sin(state[4]) - half_l * math.cos(state[4]))
        ]
        patch = Polygon(vertices, facecolor="seagreen", edgecolor="black", lw=0.5)
        ax.add_patch(patch)

    half_w = 0.5 / 2
    half_l = 0.5 / 2
    x_pts = []
    y_pts = []
    for state in uav_data:
        vertices = [
            (state[0] + half_w * math.cos(0) - half_l * math.sin(0),
            state[1] + half_w * math.sin(0) + half_l * math.cos(0)),

            (state[0] - half_w * math.cos(0) - half_l * math.sin(0),
            state[1] - half_w * math.sin(0) + half_l * math.cos(0)),

            (state[0] - half_w * math.cos(0) + half_l * math.sin(0),
            state[1] - half_w * math.sin(0) - half_l * math.cos(0)),

            (state[0] + half_w * math.cos(0) + half_l * math.sin(0),
            state[1] + half_w * math.sin(0) - half_l * math.cos(0))
        ]
        patch = Polygon(vertices, facecolor="cornflowerblue", edgecolor="black", lw=0.5)
        ax.add_patch(patch)
    ax.plot(x_pts, y_pts)
    plt.show()
