import sys
import time 
from copy import deepcopy

from RVO import RVO_update, reach, compute_V_des, reach
from vis import visualize_traj_dynamic

from progress.bar import ChargingBar


#------------------------------
#define workspace model
ws_model = dict()
#robot radius
ws_model['robot_radius'] = 0.2
#circular obstacles, format [x,y,rad]
# no obstacles
# ws_model['circular_obstacles'] = []
# with obstacles
ws_model['circular_obstacles'] = [[-0.3, 1.5, 0.3], [-0.3, 3.5, 0.3],
                                  [1.6, 1.5, 0.3], [1.6, 3.5, 0.3], [3.3, 1.5, 0.3], [3.3, 3.5, 0.3]]
# ws_model['circular_obstacles'] = [[-0.3, 1.5, 0.3]]

# obstacles'velocity
ws_model['obstacles_vel'] = []
for i in range(len(ws_model['circular_obstacles'])):
    ws_model['obstacles_vel'].append([0, 0])

#rectangular boundary, format [x,y,width/2,heigth/2]
# ws_model['boundary'] = [10, 10, 10, 10]

#------------------------------
#initialization for robot 
# position of [x,y]
X = [[-0.5+1.0*i, 0.0] for i in range(7)] + [[-0.5+1.0*i, 5.0] for i in range(7)]
# velocity of [vx,vy]
V = [[0,0] for i in range(len(X))]
# maximal velocity norm
V_max = [2.0 for i in range(len(X))]
# V_max = [1+i*0.2 for i in range(len(X))]
# goal of [x,y]
goal = [[5.5-1.0*i, 5.0] for i in range(7)] + [[5.5-1.0*i, 0.0] for i in range(7)]

#------------------------------
#simulation setup
# total simulation time (s)
total_time = 12
# simulation step
step = 0.01

#------------------------------
#simulation starts
t = 0
# flag to control obstacle's moving（0:static 1:to right -1:to left）
isReverse = 0
# save obstacles'position last time
# last_obstacles = ws_model['circular_obstacles']
# configure progress bar
bar = ChargingBar('Processing', max=total_time*10, suffix='%(percent)d%%')

while t*step < total_time:
    # compute desired vel to goal
    V_des = compute_V_des(X, goal, V_max)
    # compute the optimal vel to avoid collision
    V = RVO_update(X, V_des, V, ws_model)
    # update position
    for i in range(len(X)):
        X[i][0] += V[i][0]*step
        X[i][1] += V[i][1]*step
    #----------------------------------------
    # visualization
    if t%10 == 0:
        visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='data/snap%s.png'%str(t/10))
        #visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='data/snap%s.png'%str(t/10))
        bar.next()

    # Moving obstacles
    if isReverse:
        if t > 0:
            # co = ws_model['circular_obstacles']
            last_obstacles = deepcopy(ws_model['circular_obstacles'])
            for i in range(len(last_obstacles)):
                # update obstacles's pos
                ws_model['circular_obstacles'][i][0] = last_obstacles[i][0] + isReverse * 0.01
                # calculate obstacles' vel
                ws_model['obstacles_vel'][i][0] = ws_model['circular_obstacles'][i][0] - last_obstacles[i][0]
                ws_model['obstacles_vel'][i][1] = ws_model['circular_obstacles'][i][1] - last_obstacles[i][1]
                # print("o:{}, l:{}, d:{}".format(ws_model['circular_obstacles'][i], 
                #                     last_obstacles[i], ws_model['circular_obstacles'][i][0]-last_obstacles[i][0]))

            if t % 300 == 0:
                isReverse = -isReverse
    t += 1

bar.finish()
    
