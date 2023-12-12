import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math
import numpy as np
import csv
import os

def plot_RRT(ax, fileName):
    nodes = {}
    if os.path.exists('solutions/' + fileName + '_nodes.csv'):
        with open('solutions/' + fileName + '_nodes.csv', 'r') as csvfile: 
            reader = csv.reader(csvfile)
            for row in reader:
                node_id = int(row[0])
                x, y = float(row[1]), float(row[2])
                nodes[node_id] = (x, y)

        # Read connections from the CSV file
        connections = []
        with open('solutions/' + fileName + '_connections.csv', 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                node1, node2 = int(row[0]), int(row[1])
                connections.append((node1, node2))

        frontier_nodes = []
        with open('solutions/frontier_nodes.csv', 'r') as csvfile:  
            reader = csv.reader(csvfile)
            for row in reader:
                x, y = float(row[0]), float(row[1])
                frontier_nodes.append((x, y))

        for node1, node2 in connections:
            x1, y1 = nodes[node1]
            x2, y2 = nodes[node2]
            ax.plot([x1, x2], [y1, y2], color="goldenrod", zorder=0)  # Plot connections as lines

        for node_id, (x, y) in nodes.items():
            ax.scatter(x, y, color="darkgoldenrod", s=10,zorder=0)

        for node in frontier_nodes:
            ax.scatter(node[0], node[1], color="teal", s=10, zorder=0)


def plot_UGV(ax):
    if os.path.exists('solutions/UGV_plan.txt'):
        ugv_data = []
        with open('solutions/UGV_plan.txt', 'r') as file:
            for line in file:
                row = [float(val) for val in line.split()]
                if row: ugv_data.append(row)

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

def draw_uav_icon(ax, x, y):
    # Create a figure and axis


    # Lines representing the arms of the UAV
    arm_length = 0.4 # Length of the arms
    arm1 = plt.Line2D([x - arm_length / 2, x + arm_length / 2], [y - arm_length / 2, y + arm_length / 2], color='mediumpurple', alpha=0.75)
    arm2 = plt.Line2D([x + arm_length / 2, x - arm_length / 2], [y - arm_length / 2, y + arm_length / 2], color='mediumpurple', alpha=0.75)
    ax.add_line(arm1)
    ax.add_line(arm2)


    # Square representing the body of the UAV
    body_size = 0.16  # Size of the square
    body = plt.Rectangle((x - body_size / 2, y - body_size / 2), body_size, body_size, color='mediumpurple', alpha=0.75, zorder=10)
    ax.add_patch(body)

    # Circles represeting the rotors
    rotor_radius = 0.12  # Radius of the rotors
    rotor1 = plt.Circle((x - arm_length / 2, y - arm_length / 2), rotor_radius, color='mediumpurple', zorder=10, alpha=0.75)
    rotor2 = plt.Circle((x + arm_length / 2, y - arm_length / 2), rotor_radius, color='mediumpurple', zorder=10, alpha=0.75)
    rotor3 = plt.Circle((x - arm_length / 2, y + arm_length / 2), rotor_radius, color='mediumpurple', zorder=10, alpha=0.75)
    rotor4 = plt.Circle((x + arm_length / 2, y + arm_length / 2), rotor_radius, color='mediumpurple', zorder=10, alpha=0.75)
    ax.add_patch(rotor1)
    ax.add_patch(rotor2)
    ax.add_patch(rotor3)
    ax.add_patch(rotor4)

# Test the function by drawing the UAV icon centered at (0, 0)

def plot_UAV(ax):
    uav_data = []
    with open('solutions/UAV_plan.txt', 'r') as file:
        for line in file:
            row = [float(val) for val in line.split()]
            if row: uav_data.append(row)

    scope = 4.1 / 2
    x_pts = []
    y_pts = []
    for state in uav_data:
        draw_uav_icon(ax, state[0], state[1])
        vertices = [
            (state[0] + scope,
            state[1] + scope),

            (state[0] - scope,
            state[1] + scope),

            (state[0] - scope,
            state[1] - scope),

            (state[0] + scope ,
            state[1] - scope)
        ]
        patch = Polygon(vertices, facecolor="oldlace", zorder=0)
        ax.add_patch(patch)

def plot_obstacles():
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
    for obstacle in obstacles:
        patch = Polygon(obstacle, facecolor="indianred", edgecolor="black", lw=1, zorder=0)
        ax.add_patch(patch)

if __name__ == '__main__':
    limits = [(-1, -1), (-1, 15), (15, 15), (15, -1)]

    plt.figure()
    ax = plt.gca() 

    ax.set_aspect('equal', adjustable='box')
    plt.xlim(limits[0][0], limits[2][0])  # Set x-axis limits from 0 to 6
    plt.ylim(limits[0][1], limits[2][1])

    patch = Polygon(limits, facecolor="darkgrey", zorder=0, alpha=0.75)
    ax.add_patch(patch)
    ax.scatter(0, 0, color="blue", s=15, zorder=0, marker='*')
    ax.scatter(10, 10, color="crimson", s=15, zorder=0, marker='*')

    plot_UGV(ax)
    plot_UAV(ax)

    plot_obstacles()
    # plot_RRT(ax, 'start')
    # plot_RRT(ax, 'goal')
    plt.show()

