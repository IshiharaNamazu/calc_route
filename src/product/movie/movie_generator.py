#!/usr/bin/python3.6

import os.path

target = str(os.path.abspath(os.path.dirname(__file__))) + '/pyroute.py'

route = []
for line in open(target,"r"):
    e = line.split()
    if len(e) < 9:
        continue
    else:
        route.append([float(x) for x in e[1:9:2]])

import matplotlib.pyplot as plt

import numpy as np
import matplotlib.animation as animation
import matplotlib.image as mpimg
import numpy.linalg as LA
import math
import matplotlib.patches as patches
from basic import *
from torobo_r2_params import *

fig = plt.figure()
plt.axes().set_aspect('equal', 'datalim')

dt = 0.01
t = 0.0
index = 0

def draw_circle(center, radius):
    c = np.array(center)
    pos = [c + np.array([radius,0])]
    N = 40
    for i in range(1,N):
        theta = 2.0 * math.pi * i / N
        pos.append(c + radius * np.array([math.cos(theta), math.sin(theta)]))
    pos.append(c+np.array([radius,0]))
    x,y = np.array(pos).transpose()
    plt.plot(x,y)

def draw_machine(r):
    x,y,th = r
    center = np.array([x,y])
    R = np.array([[math.cos(th), -math.sin(th)],[math.sin(th), math.cos(th)]])
    edges = [center + R @ m for m in machine_size]
    edges.append(edges[0])
    X,Y = np.array(edges).transpose()
    plt.plot(X,Y)

def draw_machine_circle(r):
    # draw_circle(c)
    x,y,th = r
    center = np.array([x,y])
    pos = [center]
    N = 20
    for i in range(0,N):
        theta = 2.0 * math.pi * i / N
        pos.append(center + machine_size[0] * np.array([math.cos(theta+th), math.sin(theta+th)]))
    x,y = np.array(pos).transpose()
    plt.plot(x,y)

def update(i):
    plt.cla()
    #ax = fig.add_subplot(111)
    global index,t
    while route[index][0] < t and index >= 0:
        index += 1
        if index == len(route):
            print("out ouf index")
            index = -1
            break
    machine = route[index][1:]
    t += dt
    
    x,y = np.array(fence).transpose()#フェンスの描画
    plt.plot(x,y)
    for table in table_array:#ポールの描画?
        x, y = np.array(table).transpose()
        plt.plot(x,y)

    x,y,th = machine
    plt.plot(x,y,marker="x")
    if(machine_size.size == 3):
        draw_machine_circle(machine)
    else:
        draw_machine(machine)

T = route[-1][0] - route[0][0]
ani = animation.FuncAnimation(fig, update, interval = int(dt * 1e3),  frames=int(T/dt))
ani.save(str(os.path.abspath(os.path.dirname(__file__))) + '/route.mp4',writer='ffmpeg')


