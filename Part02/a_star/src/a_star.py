#!/usr/bin/env python3

#importing libraries
import rospy
from geometry_msgs.msg import Twist

import math
import heapdict
import numpy as np
import time
import vidmaker
from sortedcollections import OrderedSet
import pygame


def round_theta(theta):                             
    if theta >= 360:
        theta = theta % 360
    elif theta <= -360:
        theta = theta % 360 + 360
    return theta


def check_obstacles(x, y, robot_size = 10.5):
    buffer_val = obstacle_buffer + robot_size
    rec1 = [(0, 250 - buffer_val), (0, 265 + buffer_val), (0, 125 + buffer_val)]
    rec2 = [(0, 150 - buffer_val), (0, 165 + buffer_val), (0, 75 - buffer_val)]
    bounds = [(0, buffer_val), (0, 600 - buffer_val), (0, buffer_val), (0, 200 - buffer_val)]
    a1_circ, b1_circ, c1_circ = 400, 110, 50 + obstacle_buffer + robot_size

    if (rec1[0][1] <= x <= rec1[1][1] and 0 <= y <= rec1[2][1]) or \
       (rec2[0][1] <= x <= rec2[1][1] and rec2[2][1] <= y <= 200) or \
       (x <= bounds[0][1] or x >= bounds[1][1] or y <= bounds[2][1] or y >= bounds[3][1]) or \
       ((x - a1_circ) ** 2 + (y - b1_circ) ** 2 <= c1_circ ** 2):
        return False
    else:
        return True

# Visited nodes in an array size 600x200x360
def visited_nodes_threshold_check(x, y, theta, threshold=5):
    for dx in range(-threshold, threshold + 1):
        for dy in range(-threshold, threshold + 1):
            if visited_nodes[int(x + dx)][int(y + dy)][int(theta)]:
                return False
    return True

def check_new_node(x, y, theta, total_cost, cost_to_go, cost_to_come,interim_points,interim_velocity):
    x = np.round(x, 1)
    y = np.round(y, 1)
    theta = round_theta(np.round(theta,2))
    if visited_nodes_threshold_check(x, y, theta):
        if visited_nodes[int(x)][int(y)][int(theta)] == 0:
            if (x, y, theta) in explored_nodes:
                if explored_nodes[(x, y, theta)][0] >= total_cost:
                    explored_nodes[(x, y, theta)] = total_cost, cost_to_go, cost_to_come
                    node_records[(x, y, theta)] = (pop[0][0], pop[0][1], pop[0][2]), interim_points
                    visited_nodes_track.add((x, y, theta))
                    velocity_track[(x, y, theta)] = (pop[0][0], pop[0][1], pop[0][2]), interim_velocity
                    return None
                else:
                    return None
            explored_nodes[(x, y, theta)] = total_cost, cost_to_go, cost_to_come
            node_records[(x, y, theta)] = (pop[0][0], pop[0][1], pop[0][2]), interim_points
            explored_mapping.append((x, y))
            visited_nodes_track.add((x, y, theta))
            velocity_track[(x, y, theta)] = (pop[0][0], pop[0][1], pop[0][2]), interim_velocity

# Non holonomic constraints
def action(RPM_L,RPM_R,x, y, theta):
    t, dt, R, L = 0, 0.1, 3.3, 17.8
    interim_points = OrderedSet()
    x_new, y_new = x, y
    theta_new = np.deg2rad(theta)
    interim_points.add((trans((x_new,y_new),200)))

    while t < 1:
        theta_new += (R/L)*(RPM_R-RPM_L)*dt*2*math.pi/60
        x_new += ((R/2)*(RPM_L+RPM_R)*np.cos((theta_new))/2*dt)
        y_new += ((R/2)*(RPM_L+RPM_R)*np.sin((theta_new))/2*dt)

        temp_obs = check_obstacles(x_new,y_new)
        if not temp_obs:  break
        interim_points.add((trans((x_new,y_new),200)))
        t = t + dt
    obs = check_obstacles(x_new, y_new)
    if obs:
        new_cost_to_go = 1.2 * np.sqrt(((x_new - x_f) ** 2) + ((y_new - y_f) ** 2))
        new_cost_to_come = np.sqrt(((x_new - x_s) ** 2) + ((y_new - y_s) ** 2))
        new_total_cost = new_cost_to_go + new_cost_to_come
        check_new_node(x_new, y_new, round_theta(np.rad2deg(theta_new)), new_total_cost, new_cost_to_go, new_cost_to_come,interim_points,[])

# Backtracking
def backtracking(x, y, theta):
    backtrack.append((x, y, theta))
    key = node_records[(x, y, theta)][0]
    backtrack.append(key)
    while key != init_pos:
        key = node_records[key][0]
        backtrack.append(key)
    return backtrack[::-1]

def vel_backtracking(x, y, theta):
    vel_backtrack.append((0,0,0))
    key = velocity_track[(x, y, theta)]
    vel_backtrack.extend(key[1])
    while key[0] != init_pos:
        key = velocity_track[key[0]]
        vel_backtrack.extend(key[1])
    return vel_backtrack[::-1]

def trans(coords, height):
    return coords[0], height - coords[1]

def rec_pygame(coords, height, obj_height):
    return coords[0], height - coords[1] - obj_height

obstacle_buffer = 5

RPM1 = 20
RPM2 = 30

x_s = 50
y_s = 100
theta_s = 0
init_pos = (x_s,y_s,theta_s)

x_f = 550
y_f = 50
goal_pos = (x_f,y_f)

#Initialising variables 
explored_nodes = heapdict.heapdict()
explored_mapping = []
visited_nodes = np.zeros((600, 200, 360))
visited_nodes_track = OrderedSet()
backtrack = []
vel_backtrack = []
node_records = {}
velocity_track = {}
pop = []
the_path = []
index = 0

def move(vel_path):
    # Initializing a new node
    rospy.init_node('a_star', anonymous=False)
    
    #Creating a publisher that publishes velocity commands to /cmd_vel topic
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    #Creating message of type Twist()
    vel_msg = Twist()

    print("Moving Robot with A-star path!")
    rate = rospy.Rate(4.5)
    
    for i in vel_path:
        start = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - start) < 1:
            vel_msg.linear.x = math.sqrt(i[0]**2 + i[1]**2)
            vel_msg.angular.z = i[2]
            print("x_dot, y_dot, theta_dot: ", i)
            vel_pub.publish(vel_msg)
            rate.sleep()


# A* algorithm
if __name__ == '__main__':
    start = time.time()

    if check_obstacles(x_s, y_s) and check_obstacles(x_f, y_f):
        print('Finding path...')
        init_cost_to_go = round(np.sqrt(((x_s - x_f) ** 2) + ((y_s - y_f) ** 2)), 1)
        init_cost_to_come = 0
        init_total_cost = init_cost_to_come + init_cost_to_go
        explored_nodes[(x_s, y_s, theta_s)] = init_total_cost, init_cost_to_go, init_cost_to_come
        explored_mapping.append((x_s, y_s))

        while len(explored_nodes):
            pop = explored_nodes.popitem()
            index += 1
            x, y, theta = pop[0]

            if x_f - 2 < x < x_f + 2 and y_f - 2 < y < y_f + 2:
                print('Goal Node Reached!')
                print('List of Nodes Explored:', list(explored_nodes.keys()))
                print('Ultimate Path:', pop)
                the_path = backtracking(x, y, theta)
                print('Backtracking:', the_path)
                end = time.time()
                print('Time Taken:', round((end - start), 2), 's')
                print('Number of Iterations:', index)
                print('Backtracking: ', the_path)
                rpms = vel_backtracking(x, y, theta)
                move(rpms)
                break

            if visited_nodes[int(x)][int(y)][int(theta)] == 0:
                visited_nodes[int(x)][int(y)][int(theta)] = 1
                for rpm1, rpm2 in [(0, RPM1), (RPM1, 0), (RPM1, RPM1), (0, RPM2), (RPM2, 0), (RPM2, RPM2), (RPM1, RPM2), (RPM2, RPM1)]:
                    action(rpm1, rpm2, x, y, theta)

        if not len(explored_nodes):
            print('Path not found!')
            print('List of Nodes Explored:', list(explored_nodes.keys()))
            print('Ultimate Path:', pop)
            end = time.time()
            print('Time Taken:', round((end - start), 2), 's')
            print('Number of Iterations:', index)

    elif not check_obstacles(x_s, y_s):
        print('Start node is in obstacle space. Cannot run A*')
    elif not check_obstacles(x_f, y_f):
        print(' node is in obstacle space. Cannot run A*')
