#!/usr/bin/env python
import matplotlib
import matplotlib.pyplot as pyplot
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib.patches import Polygon
import matplotlib.cm as cmx
import matplotlib.colors as colors

from math import pi as PI
from math import atan2, sin, cos, sqrt, asin

from RVO import distance

ROB_RAD = 0.4

def visualize_traj_dynamic(ws_model, X, U, goal, time = None, name=None):
    figure = pyplot.figure()
    ax = figure.add_subplot(1,1,1)
    cmap = get_cmap(len(X))

    # ---plot traj---
    for i in range(0, len(X)):
        #-------plot car
        robot = matplotlib.patches.Circle(
            (X[i][0], X[i][1]),
            radius=ws_model['robot_radius'],
            facecolor=cmap(i),
            edgecolor='black',
            linewidth=1.0,
            ls='solid',
            alpha=1,
            zorder=2)
        ax.add_patch(robot)
        #----------plot velocity
        ax.arrow(X[i][0], X[i][1], U[i][0], U[i][1], head_width=0.05,
                 head_length=0.1, fc=cmap(i), ec=cmap(i))
        ax.text(X[i][0]-0.1, X[i][1]-0.1, r'$%s$' %
                i, fontsize=15, fontweight='bold', zorder=3)
        ax.plot([goal[i][0]], [goal[i][1]], '*',
                color=cmap(i), markersize=15, linewidth=3.0)

    # plot obstacles
    for i in range(len(ws_model['circular_obstacles'])):
        hole = ws_model['circular_obstacles'][i]
        pB = hole[0:2] #obstruct 
        vB = ws_model['obstacles_vel'][i]
        # print(V)
        # input()
        # srec = matplotlib.patches.Rectangle(
        srec = matplotlib.patches.Circle(
                (pB[0], pB[1]),
                radius=hole[2],
                facecolor= 'black',
                fill = True,
                alpha=1)
        ax.add_patch(srec)

        #----------plot cone
        for i in range(0, len(X)):
            pA = X[i] # robot
            vA = U[i]
            dist_BA = distance(pA, pB)
            theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])

            OVER_APPROX_C2S = 1.5
            rad = hole[2]*OVER_APPROX_C2S
            if (rad+ROB_RAD) > dist_BA:
                dist_BA = rad+ROB_RAD
            # half of cone's degree
            theta_BAort = asin((rad+ROB_RAD)/dist_BA)
            theta_ort_left = theta_BA+theta_BAort
            theta_ort_right = theta_BA-theta_BAort
            left = [pA[0]+dist_BA*cos(theta_ort_left), pA[1]+dist_BA*sin(theta_ort_left)]
            right = [pA[0]+dist_BA*cos(theta_ort_right), pA[1]+dist_BA*sin(theta_ort_right)]
            heading = [pA[0]+dist_BA*cos(theta_BA), pA[1]+dist_BA*sin(theta_BA)]

            # ax.plot([left[0], pA[0]+vB[0]], [left[1], pA[1]+vB[1]], '-', color=cmap(i))
            # ax.plot([right[0], pA[0]+vB[0]], [right[1], pA[1]+vB[1]], '-', color=cmap(i))
            ax.plot([left[0], pA[0], right[0], pA[0]], [left[1], pA[1], right[1], pA[1]], 
                    '-', color='blue')
            # ax.plot([heading[0], pA[0]], [heading[1], pA[1]], '-', color='blue')
            # print('\n')
            # print(pA, pB, theta_BA, heading)
            # input()

    if time:
        ax.text(2,5.5,'$t=%.1f s$' %time,
                fontsize=20, fontweight ='bold')                
    # ---set axes ---
    ax.set_aspect('equal')
    ax.set_xlim(-1.0, 6.0)
    ax.set_ylim(-1.0, 6.0)
    ax.set_xlabel(r'$x (m)$')
    ax.set_ylabel(r'$y (m)$')
    ax.grid(True)
    if name:
        pyplot.savefig(name, dpi = 200)
        # pyplot.show()
        #pyplot.savefig(name,bbox_inches='tight')
    pyplot.cla()
    pyplot.close(figure)
    return figure

def get_cmap(N):
    '''Returns a function that maps each index in 0, 1, ... N-1 to a distinct RGB color.'''
    color_norm  = colors.Normalize(vmin=0, vmax=N-1)
    scalar_map = cmx.ScalarMappable(norm=color_norm, cmap='hsv') 
    def map_index_to_rgb_color(index):
        return scalar_map.to_rgba(index)
    return map_index_to_rgb_color    
