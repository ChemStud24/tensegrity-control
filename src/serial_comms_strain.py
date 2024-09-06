# import async libraries
import asyncio
from asyncio import get_event_loop

# import vpython libraries
from vpython import *

# async pyserial
from serial_asyncio import open_serial_connection

# library for async equivalent of input()
from aioconsole import ainput

import sys

import csv

import time

import math

import re

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd
import numpy as np
  


async def run_connection(action_generator, device_name):
    # open serial connection through the Streams API and get reader and writer objects
    reader, writer = await open_serial_connection(url=device_name, baudrate=115200)

    # relay status to terminal
    print("Opened port. Press reset on Central to initialize echoing. Ctrl-C to stop.")

    # execute reader and writer coroutines concurrently
    await asyncio.gather(read(reader),write(writer, action_generator))

# Reads from serial, converts all received messages to strings, and prints to terminal
async def read(reader):
    filename1 = f"strain-output-1.csv"
    filename2 = f"strain-output-2.csv"
    filename3 = f"strain-output-3.csv"
    plt.ion()
    plt.style.use('seaborn')
    plt.style.use('fast')

    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)
    plt.show()
    time_list = []
    strain1_list = []
    capdac = 11
    c_off = capdac * 3.125
    start_time = time.time()
    scene.range = 20
    scene.background = color.yellow
    toRad = 2*np.pi/360
    toDeg = 1/toRad
    scene.forward=vector(-1,-1,-1)

    #scene.width = 1200
    #scene.height = 1080
    scene.width = 600
    scene.height = 480
            
    xarrow = arrow(length=2, shaftwidth=.1, color=color.red, axis=vector(1,0,0), pos=vector(0,0,10))
    yarrow = arrow(length=2, shaftwidth=.1, color=color.green, axis=vector(0,1,0), pos=vector(0,0,10))
    zarrow = arrow(length=4, shaftwidth=.1, color=color.blue, axis=vector(0,0,1), pos=vector(0,0,10))

    xarrow1 = arrow(length=2, shaftwidth=.1, color=color.red, axis=vector(1,0,0), pos=vector(10,0,0))
    yarrow1 = arrow(length=2, shaftwidth=.1, color=color.green, axis=vector(0,1,0), pos=vector(10,0,0))
    zarrow1 = arrow(length=4, shaftwidth=.1, color=color.blue, axis=vector(0,0,1), pos=vector(10,0,0))

    frontArrow=arrow(length=4,shaftwidth=.1,color=color.purple,axis=vector(1,0,0), pos=vector(0,0,10))
    upArrow=arrow(length=1,shaftwidth=.1,color=color.magenta,axis=vector(0,1,0), pos=vector(0,0,10))
    sideArrow=arrow(length=2,shaftwidth=.1,color=color.orange,axis=vector(0,0,1), pos=vector(0,0,10))

    frontArrow1=arrow(length=4,shaftwidth=.1,color=color.purple,axis=vector(1,0,0), pos=vector(10,0,0))
    upArrow1=arrow(length=1,shaftwidth=.1,color=color.magenta,axis=vector(0,1,0), pos=vector(10,0,0))
    sideArrow1=arrow(length=2,shaftwidth=.1,color=color.orange,axis=vector(0,0,1), pos=vector(10,0,0))
    
    #bBoard = box(length=6,width=2,height=.2,opacity=.8,pos=vector(0,0,0,))
    bn = box(length=2,width=2,height=.1, pos=vector(-0.5,0.1+.05,10),color=color.blue)
    #bn = box(length=2,width=1.5,height=.1, pos=vector(0,0,0),color=color.blue)
    #nano = box(length=1.75,width=.6,height=.1,pos=vector(-2,.1+.05,0),color=color.green)

    #bBoard1 = box(length=6,width=2,height=.2,opacity=.8,pos=vector(100,0,0,))
    bn1 = box(length=2,width=2,height=.1, pos=vector(10-0.5,0.1+0.05,0),color=color.green)
    #bn1 = box(length=2,width=1.5,height=.1, pos=vector(0,0,0),color=color.blue)
    #nano1 = box(length=1.75,width=.6,height=.1,pos=vector(100-2,.1+.05,0),color=color.green)

    #myObj=compound([bBoard,bn,nano])

    #myObj1=compound([bBoard1,bn1,nano1])
    #node1_pos = vec(-8.34910163, -5.43642221,  5.59096297)
    #node2_pos = vec(8.26181654,  4.71232978,  0.99964173)
    #node3_pos = vec(-7.90737948,  2.72556502, 10.31785351)
    #node4_pos = vec(8.14254871, -4.72856965,  0.999287)
    #node5_pos = vec(-9.12030646,  2.71085719,  0.9992207)
    #node6_pos = vec(8.97242231,  0.01623987,  9.08576528)
    #
    #node1_col = color.yellow
    #node2_col = node1_col
    #node3_col = color.red
    #node4_col = node3_col
    #node5_col = color.blue
    #node6_col = node5_col
    #node_radius = 0.5
    #
    #bar1_pos = node1_pos
    #bar1_axis = node2_pos - node1_pos
    #origin_offset = bar1_axis * 0.5 + node1_pos

    #node1_pos = node1_pos - origin_offset
    #node2_pos = node2_pos - origin_offset
    #node3_pos = node3_pos - origin_offset
    #node4_pos = node4_pos - origin_offset
    #node5_pos = node5_pos - origin_offset
    #node6_pos = node6_pos - origin_offset

    #

    #bar1_axis = node2_pos - node1_pos
    #angle1 = -bar1_axis.diff_angle(vec(bar1_axis.x,0,bar1_axis.z))
    #node1_pos = rotate(node1_pos, angle1, vec(0,0,1))
    #node2_pos = rotate(node2_pos, angle1, vec(0,0,1))
    #node3_pos = rotate(node3_pos, angle1, vec(0,0,1))
    #node4_pos = rotate(node4_pos, angle1, vec(0,0,1))
    #node5_pos = rotate(node5_pos, angle1, vec(0,0,1))
    #node6_pos = rotate(node6_pos, angle1, vec(0,0,1))

    #bar1_axis = node2_pos - node1_pos
    #angle2 = -bar1_axis.diff_angle(vec(1,0,0))
    #node1_pos = rotate(node1_pos, angle2, vec(0,1,0))
    #node2_pos = rotate(node2_pos, angle2, vec(0,1,0))
    #node3_pos = rotate(node3_pos, angle2, vec(0,1,0))
    #node4_pos = rotate(node4_pos, angle2, vec(0,1,0))
    #node5_pos = rotate(node5_pos, angle2, vec(0,1,0))
    #node6_pos = rotate(node6_pos, angle2, vec(0,1,0))


    ##k = rotate(k, angle=np.pi/2, axis=vector(0,1,0))
    #bar1_pos = node1_pos
    #bar1_size = vec(bar1_axis.mag, 0.2, 0.2)
    #bar1_col = color.yellow
    #bar1_axis = node2_pos - node1_pos

    #bar2_pos = node3_pos
    #bar2_axis = node4_pos - node3_pos
    #bar2_size = vec(bar2_axis.mag, 0.2, 0.2)
    #bar2_col = color.red
    #
    #
    #bar3_pos = node5_pos
    #bar3_axis = node6_pos - node5_pos
    #bar3_size = vec(bar3_axis.mag, 0.2, 0.2)
    #bar3_col = color.blue
    #
    #cab1_pos = node1_pos
    #cab1_axis = node5_pos - node1_pos
    #cab1_size = vec(cab1_axis.mag, 0.1, 0.1)
    #cab1_col = color.white
    #
    #cab1_pos = node1_pos
    #cab1_axis = node5_pos - node1_pos
    #cab1_size = vec(cab1_axis.mag, 0.1, 0.1)
    #cab_col = color.white
    #
    #cab2_pos = node1_pos
    #cab2_axis = node3_pos - node1_pos
    #cab2_size = vec(cab2_axis.mag, 0.1, 0.1)
    #
    #cab3_pos = node3_pos
    #cab3_axis = node5_pos - node3_pos
    #cab3_size = vec(cab3_axis.mag, 0.1, 0.1)
    #
    #cab4_pos = node2_pos
    #cab4_axis = node6_pos - node2_pos
    #cab4_size = vec(cab4_axis.mag, 0.1, 0.1)
    #
    #cab5_pos = node2_pos
    #cab5_axis = node4_pos - node2_pos
    #cab5_size = vec(cab4_axis.mag, 0.1, 0.1)
    #
    #cab6_pos = node4_pos
    #cab6_axis = node6_pos - node4_pos
    #cab6_size = vec(cab5_axis.mag, 0.1, 0.1)
    #
    #cab7_pos = node2_pos
    #cab7_axis = node5_pos - node2_pos
    #cab7_size = vec(cab7_axis.mag, 0.1, 0.1)
    #
    #cab8_pos = node1_pos
    #cab8_axis = node4_pos - node1_pos
    #cab8_size = vec(cab8_axis.mag, 0.1, 0.1)
    #
    #
    #cab9_pos = node3_pos
    #cab9_axis = node6_pos - node3_pos
    #cab9_size = vec(cab9_axis.mag, 0.1, 0.1)
    #
    #
    #
    #
    #
    #
    #node1 = sphere(pos = node1_pos, radius = node_radius, color = node1_col)
    #node2 = sphere(pos = node2_pos, radius = node_radius, color = node2_col)
    #bar1 = cylinder(pos = bar1_pos, axis = bar1_axis, size = bar1_size, color = bar1_col)
    #
    #node3 = sphere(pos = node3_pos, radius = node_radius, color = node3_col)
    #node4 = sphere(pos = node4_pos, radius = node_radius, color = node4_col)
    #bar2 = cylinder(pos = bar2_pos, axis = bar2_axis, size = bar2_size, color = bar2_col)
    #
    #node5 = sphere(pos = node5_pos, radius = node_radius, color = node5_col)
    #node6 = sphere(pos = node6_pos, radius = node_radius, color = node6_col)
    #bar3 = cylinder(pos = bar3_pos, axis = bar3_axis, size = bar3_size, color = bar3_col)
    #
    #cab1 = cylinder(pos = cab1_pos, axis = cab1_axis, size = cab1_size, color = cab_col)
    #cab2 = cylinder(pos = cab2_pos, axis = cab2_axis, size = cab2_size, color = cab_col)
    #cab3 = cylinder(pos = cab3_pos, axis = cab3_axis, size = cab3_size, color = cab_col)
    #cab4 = cylinder(pos = cab4_pos, axis = cab4_axis, size = cab4_size, color = cab_col)
    #cab5 = cylinder(pos = cab5_pos, axis = cab5_axis, size = cab5_size, color = cab_col)
    #cab6 = cylinder(pos = cab6_pos, axis = cab6_axis, size = cab6_size, color = cab_col)
    #cab7 = cylinder(pos = cab7_pos, axis = cab7_axis, size = cab7_size, color = cab_col)
    #cab8 = cylinder(pos = cab8_pos, axis = cab8_axis, size = cab8_size, color = cab_col)
    #cab9 = cylinder(pos = cab9_pos, axis = cab9_axis, size = cab9_size, color = cab_col)
    #myObj = compound([bn, node1, node2, node3, node4, node5, node6, bar1, bar2, bar3, cab1, cab2, cab3, cab4, cab5, cab6, cab7, cab8, cab9])
    
    #g1 = graph(xtitle = 'time [s]', ytitle = 'temperature [C]')
    #temp_curve_1 = gcurve(graph=g1, color=color.red)
    #temp_curve_2 = gcurve(graph=g1, color=color.blue)
    #temp_curve_3 = gcurve(graph=g1, color=color.black)
        
    while True:
        line = await reader.readuntil(b'\n')
        line = str(line, 'utf-8')
        print(line)
        q = re.findall(r"[-+]?\d*\.\d+|\d+", line)#find all float number
        #print(len(q))
        print(q)
        s = re.findall(r"0x[0-9a-f]+", line)#find all hex number
        #print(s)
        if len(q) == 5: 
            q0 = float(q[1])
            q1 = float(q[2])
            q2 = float(q[3])
            q3 = float(q[4])
            roll = -math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))
            sinp = 2*(q0*q2-q3*q1)
            if abs(sinp) >= 1:
                pitch = math.copysign(np.pi/2, sinp)
            else:
                pitch = math.asin(sinp)
            
            #yaw = -math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))-np.pi/2
            yaw = -math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
            if q[0] == '2':
                #print('roll1')
                #print(roll/np.pi*180)
                #print('pitch1')
                #print(pitch/np.pi*180)
                #print('yaw1')
                #print(yaw/np.pi*180)
                k=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
                #print('k_pre')
                #print(k)
                k = rotate(k, angle=-np.pi/2, axis=vector(0,1,0))
                #print('k')
                #print(k)
                y=vector(0,1,0)
                s=cross(k,y)
                v=cross(s,k)
                vrot=v*cos(roll)+cross(k,v)*sin(roll)
                #print('axis')
                #print(k)

                frontArrow.axis=-k
                sideArrow.axis=cross(k,vrot)
                upArrow.axis=vrot
                #myObj.axis=k
                bn.axis=k
                #myObj.up=vrot
                bn.up=vrot
                sideArrow.length=2
                frontArrow.length=4
                upArrow.length=1
            elif q[0] == '1':
                print('roll2')
                print(roll/np.pi*180)
                print('pitch2')
                print(pitch/np.pi*180)
                print('yaw2')
                print(yaw/np.pi*180)
                k1=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
                #print('k_pre')
                #print(k)
                k1 = rotate(k1, angle=-np.pi/2, axis=vector(0,1,0))
                #print('k')
                #print(k)
                y1=vector(0,1,0)
                s1=cross(k1,y1)
                v1=cross(s1,k1)
                vrot1=v1*cos(roll)+cross(k1,v1)*sin(roll)
                #print('axis')
                #print(k)

                frontArrow1.axis=-k1
                sideArrow1.axis=cross(k1,vrot1)
                upArrow1.axis=vrot1
                bn1.axis=k1
                bn1.up=vrot1
                sideArrow1.length=2
                frontArrow1.length=4
                upArrow1.length=1

             
            #k=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))

        #elif len(q) == 3 and float(q[0]) > 0:
        #    print('11111111')
        #    print(q)
        #    rt1 = float(q[0]) / ((4095 - float(q[0])) / 10000)
        #    T1 = 1 / (np.log(rt1 / 10000) / 3500 + 1 / 298.15)
        #    T1 = T1 - 273.15
        #    rt2 = float(q[1]) / ((4095 - float(q[1])) / 10000)
        #    T2 = 1 / (np.log(rt2 / 10000) / 3500 + 1 / 298.15)
        #    T2 = T2 - 273.15
        #    rt3 = float(q[2]) / ((4095 - float(q[2])) / 10000)
        #    T3 = 1 / (np.log(rt3 / 10000) / 3500 + 1 / 298.15)
        #    T3 = T3 - 273.15
        #    now_time_temp = time.time()
        #    temp_cur_time = now_time_temp - start_time
        #    temp_curve_1.plot(pos=(temp_cur_time, T1))
        #    temp_curve_2.plot(pos=(temp_cur_time, T2))
        #    temp_curve_3.plot(pos=(temp_cur_time, T3))
        #    print(temp_cur_time)
        #    print(T1)
        #    print(T2)
        #    print(T3)
        elif len(q) == 3:
            print(q)
        else:
            print('11111111') 
            

# Listen to terminal for user input and relay messages to serial
async def write(writer, action_generator):

    # this wait is not functional, it is just to give the user time to reset Central
    await asyncio.sleep(6) 

    while True:
        
        msg = await action_generator.get_action() 
        msg = msg + '\n'
        #print(msg)
        print('Sending')
        writer.write(msg.encode('UTF-8'))

def serial_process_start(action_generator, device_name):
    asyncio.run(run_connection(action_generator, device_name))

def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val      

def twos(val_str, bytes):
    import sys
    val = int(val_str, 2)
    b = val.to_bytes(bytes, byteorder=sys.byteorder, signed=False)                                                          
    return int.from_bytes(b, byteorder=sys.byteorder, signed=True)    

class CmdLnActionGenerator:
        def __init__(self):
            pass
        def get_action(self):
            return ainput("Message to send over serial terminal: ")

if __name__ == '__main__':
    
    serial_process_start(CmdLnActionGenerator(), sys.argv[1])
