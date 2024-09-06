import sys
import serial
import time
import math
from math import cos,sin,tan
import re
# import csv
import json
import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt
# from vpython import *
from pynput import keyboard
from scipy.spatial.transform import Rotation as R

# ROS messages
import rospy
from tensegrity.msg import Motor, MotorsStamped, Sensor, SensorsStamped, Imu, ImuStamped
# from geometry_msgs.msg import QuaternionStamped

def flush(serial_port):
    serial_port.reset_input_buffer()
    serial_port.reset_output_buffer()
    time.sleep(0.001)

def send_command(serial_port, input_string, delay_time):
    # to_microcontroller_msg = f'{input_string}\n'
    to_microcontroller_msg = '          ' + '{}\n'.format(input_string)
    serial_port.write(to_microcontroller_msg.encode('UTF-8'))
    if delay_time < 0:
        delay_time = 0
    time.sleep(delay_time/1000)

def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val 

def get_imu_calibration(mag,grav):
    """Calculate the IMU's individual calibration using the direction of the magnetic and gravitational fields.
    mag is an iterable of length 3 that represents the magnetic field strength in the IMU's x-, y-, and z-directions
    grav is an iterable of length 3 that represents the gravitational field strength in the IMU's x-, y-, and z-directions
    Returns R, the rotation matrix you can use to pre-multiply all vectors in the IMU frame to transform them into the world frame
    """
    x = np.array(mag)
    x = x/np.linalg.norm(x)
    x = np.reshape(x,(3,1))
    z = -1*np.array(grav)
    z = z/np.linalg.norm(z)
    z = np.reshape(z,(3,1))
    y = np.cross(z,x)
    R = np.hstack(x,y,z)
    return R

def read(serial_port):
    flush(serial_port)
    global state
    global count
    global capacitance
    global acceleration
    global orientation
    global imu
    line = serial_port.read_until(b'\n')# read data string sent from central
    line = str(line).encode('utf-8')# convert to string
    line = line.split('*')[-1] # remove padding
    # print('Whole Line: ',line)
    s = re.findall(r"0x[0-9a-f]+", line)# extract strain data (hex number) from the data string
    q = re.findall(r"[-+]?\d*\.\d+|\d+", line)# extract imu data (decimal number) from the data string
    if len(line) >=2 :# valid data string will be longer than 9 characters    
        # print('line')
        print(line)
        if line[0] == "S":# read strain data
            now_time = time.time()
            meas_time = now_time - start_time
            if len(s) >= num_sensors:
                for i in range(num_sensors):
                    cap[i] = s[i]
            # print(cap)
                #add control code here
            if not 0 in cap:
                for i, cap_hex in enumerate(cap):
                    adc_counts = int(str(cap_hex[2:]), 16)
                    if adc_counts != 0:
                        # capacitance[i] = 16.0 * 0.5 / adc_counts / 3.3 * 1024
                        capacitance[i] = 42.0 / adc_counts / 3.3 * 1024
                    length[i] = (capacitance[i] - b[i]) / m[i] #mm #/ 10
                # print(capacitance)
                #check if motor reached the target
                for i in range(num_motors):
                    pos[i] = (length[i] - min_length) / RANGE# calculate the current position of the motor
                    #check if motor reached the target
                    if pos[i] + tol > states[state, i] and pos[i] - tol < states[state, i]:
                        send_command(serial_port, "d "+str(i+1)+" 0", 0)#stop the motor 
                        done[i] = True
                    # else:
                    #     done[i] = False
                    # update directions
                    # print(done[i])
                    if not done[i]:
                        error[i] = pos[i] - states[state, i]
                        d_error[i] = error[i] - prev_error[i]
                        cum_error[i] = cum_error[i] + error[i]
                        prev_error[i] = error[i]
                        #update speed
                        command[i] = max([min([P*error[i] + I*cum_error[i] + D*d_error[i], 1]), -1])
                        speed = command[i] * max_speed * flip[i]
                        # send_command(serial_port, "d"+str(i+1)+" 0", 0)#stop the motor 
                        send_command(serial_port, "d "+str(i+1)+" "+str(speed), 0)#run the motor at new speed                            
                print('State: ',state)
                print(state)
                print("Position: ",pos)
                print("Target: ",states[state])
                print(pos)
                print(states[state])
                print("Done: ",done)
                print("Length: ",length)
                print("Capacitance: ",capacitance)
                count = count + 1     
                if all(done):
                    state += 1
                    state %= num_steps
                    for i in range(num_motors):
                        done[i] = False
                        prev_error[i] = 0
                        cum_error[i] = 0
                #strain_curve.plot(pos=(meas_time, out2))
                #save strain file
               	#with open(filename1,"a") as f:
                #    writer = csv.writer(f,delimiter=",")
                #    writer.writerow([cur_time4] + [out1] + [out2] + [out3] + [out4])
            #use fdc1004 module 2 to read three strain sensors. Channel 1 doesn't work well for some reason so use Channel 2 3 4
            ### white ###
#            elif line[10] == '2':
#                #strain_curve_1.plot(pos=(cur_time4, out4))
#                with open(filename2,"a") as f:
#                    writer = csv.writer(f,delimiter=",")
#                    writer.writerow([cur_time4] + [out1] + [out2] + [out3] + [out4])
            #use fdc1004 module 3 to read three strain sensors. Channel 1 doesn't work well for some reason so use Channel 2 3 4
#            ### green ###
#            elif line[10] == '3':
#                #strain_curve_1.plot(pos=(cur_time4, out4))
#                with open(filename3,"a") as f:
#                    writer = csv.writer(f,delimiter=",")
#                    writer.writerow([cur_time4] + [out1] + [out2] + [out3] + [out4])
            else:
                print('+++++')

        # read acceleration data    
        elif line[0] == "A":
            q = re.findall(r"[-+]?\d*\.\d+|\d+", line)# extract imu data (decimal number) from the data string
            acceleration = [acc for acc in q]
            # print('Acceleration: ',acceleration)

        # read orientation data
        elif line[0] == "O":
            q = re.findall(r"[-+]?\d*\.\d+|\d+", line)# extract imu data (decimal number) from the data string
            orientation = [orio for orio in q]
            # print('Orientation: ',orientation)

        #read imu data    
        # elif line[0] == 'q' and len(q) == 8: # and abs(float(q[0])) <= 1 and abs(float(q[1])) <= 1 and abs(float(q[2])) <= 1 and abs(float(q[3])) <= 1:
        elif line[0] == 'q' and len(q) == 5:
            #print(q[0])
            #print(q[1])
            #print(q[2])
            #print(q[3])
            # for i,q in enumerate([q[0:4],q[4:]]):
            # q0 = float(q[0])
            # q1 = float(q[1])
            # q2 = float(q[2])
            # q3 = float(q[3])
            q0 = float(q[1])
            q1 = float(q[2])
            q2 = float(q[3])
            q3 = float(q[4])
            roll = -math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))#convert quarternion to Euler angle for roll angle
            #convert quarternion to Euler angle for pitch angle
            sinp = 2*(q0*q2-q3*q1)
            #deal with gimlock
            if abs(sinp) >= 1:
                pitch = math.copysign(np.pi/2, sinp)
            else:
                pitch = math.asin(sinp)
            
            #yaw = -math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))-np.pi/2
            #convert quarternion to Euler angle for pitch angle
            yaw = -math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
            # Roll = roll/np.pi*180
            # print('roll')
            # print(roll/np.pi*180)
            #print('pitch')
            #print(pitch/np.pi*180)
            # print('yaw')
            #print(yaw/np.pi*180)
            #modify this part if we want to integrate imu data to determine the actuation order of three long cables
            #125
            #if Roll>72.85 and Roll<=137.55:
            #    send_command(serial_port, "y 0 1", 0)
            #    send_command(serial_port, "n 2 3 4 5", 0)
            ##145
            #elif Roll>15.95 and Roll<=72.85:
            #    send_command(serial_port, "n 0 1 4 5", 0)
            #    send_command(serial_port, "y 2 3", 0)
            ##256
            #elif (Roll>137.55 and Roll<=180) or (Roll>=-180 and Roll<=-163.5):
            #    send_command(serial_port, "n 0 1 2 3", 0)
            #    send_command(serial_port, "y 4 5", 0)
            ##236    
            #elif Roll>-163.5 and Roll<=-106.05:
            #    send_command(serial_port, "n 0 1 4 5", 0)
            #    send_command(serial_port, "y 2 3", 0)
            ##346  
            #elif Roll>-106.05 and Roll<=-41.9:
            #    send_command(serial_port, "n 2 3 4 5", 0)
            #    send_command(serial_port, "y 0 1", 0)
            ##134
            #elif Roll>-41.9 and Roll<15.95:
            #    send_command(serial_port, "n 0 1 2 3", 0)
            #    send_command(serial_port, "y 4 5", 0)
            #else:
            #    print('wrong orientation')            
            #visualization of the state reconstrution 
            #k=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))

            k=np.array([cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch)])
            ##print('k_pre')
            ##print(k)
            # k = rotate(k, angle=np.pi/2, axis=vector(0,1,0))
            r = R.from_rotvec(np.pi/2 * np.array([0, 1, 0]))
            k = r.apply(k)
            ##print('k')
            ##print(k)
            # y=vector(0,1,0)
            y=np.array([0,1,0])
            s=np.cross(k,y)
            v=np.cross(s,k)
            vrot=v*cos(roll)+np.cross(k,v)*sin(roll)
            ##print('axis')
            ##print(k)
            # imu[i] = np.cross(k,vrot)
            imu[int(q[0])-1] = np.cross(k,vrot)

            #frontArrow.axis=k
            #sideArrow.axis=cross(k,vrot)
            #upArrow.axis=vrot
            #myObj.axis=k
            #myObj.up=vrot
            #sideArrow.length=2
            #frontArrow.length=4
            #upArrow.length=1
    else:
        print('+')
        send_command(serial_port, "s", 0)

    # send ROS messages
    control_msg = MotorsStamped()
    strain_msg = SensorsStamped()
    imu_msg = ImuStamped()
    # get timestamp
    timestamp = rospy.Time.now()
    control_msg.header.stamp = timestamp
    strain_msg.header.stamp = timestamp
    imu_msg.header.stamp = timestamp
    # motors
    for motor_id in range(num_motors):
        motor = Motor()
        motor.id = motor_id
        motor.position = pos[motor_id]
        motor.target = states[state,motor_id]
        motor.speed = command[motor_id] * max_speed #abs(command[motor_id]) * max_speed
        # motor.direction = command[motor_id] > 0
        motor.done = done[motor_id]
        control_msg.motors.append(motor)
    # sensors
    for sensor_id in range(num_sensors):
        sensor = Sensor()
        sensor.id = sensor_id
        sensor.length = length[sensor_id]
        sensor.capacitance = capacitance[sensor_id]
        strain_msg.sensors.append(sensor)
    # imu
    # imu_msg.ax = float(acceleration[0])
    # imu_msg.ay = float(acceleration[1])
    # imu_msg.az = float(acceleration[2])
    # imu_msg.yaw = float(orientation[0])
    # imu_msg.pitch = float(orientation[1])
    # imu_msg.roll = float(orientation[2])
    for imu_id in range(num_imus):
        IMU = Imu()
        IMU.id = imu_id
        if any(imu[imu_id]) == None:
            IMU.x = None
            IMU.y = None
            IMU.z = None
        else:
            IMU.x = imu[imu_id][0]
            IMU.y = imu[imu_id][1]
            IMU.z = imu[imu_id][2]
        imu_msg.imus.append(IMU)
    # publish
    control_pub.publish(control_msg)
    strain_pub.publish(strain_msg)
    imu_pub.publish(imu_msg)


def tensegrity_run(device_name):
    global keep_going
    # A welcome message
    print("Running serial_tx_cmdline node with device: " + device_name)
    # create the serial port object, non-exclusive (so others can use it too)
    serial_port = serial.Serial(port=device_name, baudrate=115200, timeout=1) # flush out any old data
    flush(serial_port)
    #define name of strain files
    # global filename1, filename2, filename3
    # filename1 = f"strain-output-1.csv"
    # filename2 = f"strain-output-2.csv"
    # filename3 = f"strain-output-3.csv"
    # finishing setup.
    print("Opened port. Ctrl-C to stop.")
    # g1 = graph(xtitle = 'time [s]', ytitle = 'capacitance [pF]')
    # strain_curve_1 = gcurve(graph=g1, color=color.red)
    # strain_plot_list = [strain_curve_1]
    global start_time
    start_time = time.time()# set starting time
    
    # If not using ROS, we'll do an infinite loop:
    while keep_going and not rospy.is_shutdown():

        # request something to send
        try:
            # read(serial_port, strain_plot_list)
            read(serial_port)
            
        except KeyboardInterrupt:
            # Nicely shut down this script.
            print("\nShutting down serial_tx_cmdline...")
            #set duty cycle as 0 to turn off the motors
            send_command(serial_port, "d 1 0", 0) 
            send_command(serial_port, "d 2 0", 0)
            send_command(serial_port, "d 3 0", 0)
            send_command(serial_port, "d 4 0", 0) 
            send_command(serial_port, "d 5 0", 0)
            send_command(serial_port, "d 6 0", 0)
            sys.exit()

    # Nicely shut down this script.
    print("\nShutting down serial_tx_cmdline...")
    # for i in range(100):
        #set duty cycle as 0 to turn off the motors
        # send_command(serial_port, "d1 0", 0) 
        # send_command(serial_port, "d2 0", 0)
        # send_command(serial_port, "d3 0", 0)
        # send_command(serial_port, "d4 0", 0) 
        # send_command(serial_port, "d5 0", 0)
        # send_command(serial_port, "d6 0", 0)
    send_command(serial_port, "s", 0)
    sys.exit()

def onpress(key):
    global keep_going
    global states
    global tol
    global done
    if key == keyboard.KeyCode.from_char('q'):
        keep_going = False
    elif key == keyboard.KeyCode.from_char('r'):
        states = np.array([[0.9]*num_motors]*num_steps)
        done = np.array([False] * num_motors)
        tol = 0.03


            # the main function: just call the helper, while parsing the serial port path.
if __name__ == '__main__':
    # init ROS stuff
    rospy.init_node('tensegrity')

    control_pub = rospy.Publisher('control_msg',MotorsStamped,queue_size=10)
    strain_pub = rospy.Publisher('strain_msg',SensorsStamped,queue_size=10)
    imu_pub = rospy.Publisher('imu_msg',ImuStamped,queue_size=10)

    # keyboard listener for quitting
    keep_going = True
    my_listener = keyboard.Listener(on_press=onpress)
    my_listener.start()

    calibration_file = '../calibration/autocalibration.json'

    #m = np.array([0.1512, 0.1626, 0.1728]) # tethered
    #b = np.array([10.7456, 9.5023, 15.7616])
    # m = np.array([0.05015,0.08364,0.08381,0.06574,0.08528,0.07355,0.12693,0.11281,0.11803])
    # b = np.array([12.839,12.546,12.650,13.365,11.526,13.358,13.230,11.761,12.884])

    # m = np.array([0.04686,0.04140,0.04133,0.03620,0.04841,0.04468,0.04296,0.03685,0.03965])
    # b = np.array([15.831,16.588,15.931,14.534,13.733,15.246,10.301,13.761,12.006])

    # m = np.array([0.05061,0.05455,0.04455,0.04701,0.05200,0.04135,0.03161,0.03196,0.03672])
    # b = np.array([14.256,15.232,15.264,16.159,14.024,15.343,15.422,14.297,14.304])

    # m = np.array([0.03738,0.04733,0.03054,0.03657,0.04805,0.04152,0.03161,0.03233,0.03360])
    # b = np.array([17.394,16.841,17.881,18.236,15.622,16.197,15.422,14.327,15.851])
    
    # m = np.array([0.04309,0.05103,0.03923,0.04461,0.04159,0.03893,0.03795,0.02317,0.04196])
    # b = np.array([19.672,18.631,19.306,17.803,19.178,17.478,16.029,20.233,15.409])

    m = np.array([0.04088,0.03714,0.04448,0.03179,0.04833,0.03890,0.03437,0.02701,0.02905])
    b = np.array([15.623,14.650,15.023,10.605,15.213,16.303,14.494,14.508,15.190])

    m = np.array([0.04900,0.05697,0.01787,0.05363,0.04362,0.04604,0.03437,0.02701,0.02905])
    b = np.array([12.764,13.699,15.536,14.558,14.471,13.325,14.494,14.508,15.190])
    # # load autocalibration file
    # data = json.load(open(calibration_file))
    # m = data.get('m')
    # b = data.get('b')

    max_speed = 99# set duty cycle as 99 for the max speed, resolution can be improved by changing the bits in C++ code 
    num_sensors = 9# set number of strain sensors
    num_motors = 6# set number of motors
    num_imus = 2#set number of inertial measurement units
    # num_steps = 12#set number of gait patterns
    min_length = 95#100 #mm #7.2#6.8#6.5 #set minimum length of sensors
    count = 0
    pos = [0] * num_motors
    cap = [0] * num_sensors
    capacitance = [0] * num_sensors
    length = [0] * num_sensors #mm
    imu = [[0,0,0]] * num_imus
    error = [0] * num_motors
    prev_error = [0] * num_motors
    cum_error = [0] * num_motors
    d_error = [0] * num_motors
    command = [0] * num_motors
    flip = [1,1,-1,-1,-1,1]
    acceleration = [0]*3
    orientation = [0]*3
    RANGE = 80 #mm #90
    tol = 0.11#0.07
    #define PID
    P = 5.0 #10.0
    I = 0.01
    D = 0.5

    # P = 2.28
    # I = 2.68
    # D = 0.49

    # P = 5.0
    # I = 0.01
    # D = 0.5

    #states = np.array([[0.0, 1.0, 1.0, 0.0, 1.0, 0.3],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 0.0, 0.3, 1.0, 0.0, 0.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 1.0, 0.0, 0.3, 0.0, 0.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]])
    #states = np.array([[0.0, 1.0, 1.0, 0.0, 1.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 0.0, 1.0, 1.0, 0.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 1.0, 0.0, 1.0, 1.0, 0.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]])
    #states = np.array([[0.0, 1.0, 1.0, 1.0, 0.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 0.0, 1.0, 1.0, 1.0, 0.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 1.0, 0.0, 0.0, 1.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]])
    #states = np.array([[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[0.0, 1.0, 1.0, 0.0, 1.0, 1.0],[0.0, 1.0, 0.2, 0.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 0.2, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 1.0, 0.0, 1.0, 1.0, 0.0],[1.0, 0.0, 0.0, 1.0, 1.0, 0.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 0.2, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 0.0, 1.0, 1.0, 0.0, 1.0],[0.0, 0.0, 1.0, 1.0, 0.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 0.2]])#15 steps gait
    #ignore = [[],[],[1,4,5],[],[],[],[],[0,3,4],[],[],[],[],[2,3,5],[],[]]#15 steps gait
    #states = np.array([[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]])
    # # THIS IS THE GOOD ONE
    # states = np.array([[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 1.0, 1.0, 0.0, 1.0, 1.0],[0.0, 1.0, 1.0, 0.0, 1.0, 1.0],[0.0, 1.0, 0.1, 0.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 0.1, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 0.0],[1.0, 1.0, 0.0, 1.0, 1.0, 0.0],[1.0, 0.1, 0.0, 1.0, 1.0, 0.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 0.1, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 0.0, 1.0],[1.0, 0.0, 1.0, 1.0, 0.0, 1.0],[0.1, 0.0, 1.0, 1.0, 0.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 0.1]])#18 steps gait
    # ignore = [[],[0,1,2,4,5],[1,2,4,5],[1,4,5],[],[0,1,2,3,5],[],[0,1,2,3,4],[0,1,3,4],[0,3,4],[],[0,1,2,4,5],[],[0,1,2,3,5],[0,2,3,5],[2,3,5],[],[0,1,2,3,4]]#18 steps gait
    # # ABOVE IS THE GOOD ONE
    #states = np.array([[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 1.0, 1.0, 0.0, 1.0, 1.0],[0.0, 1.0, 1.0, 0.0, 1.0, 1.0],[0.0, 1.0, 0.1, 0.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 0.0],[1.0, 1.0, 0.0, 1.0, 1.0, 0.0],[1.0, 0.1, 0.0, 1.0, 1.0, 0.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 0.1, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 0.0, 1.0],[1.0, 0.0, 1.0, 1.0, 0.0, 1.0],[0.1, 0.0, 1.0, 1.0, 0.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 0.1]])#18 steps gait
    #ignore = [[],[0,1,2,4,5],[1,2,4,5],[1,4,5],[],[],[0,1,2,3,4],[0,1,3,4],[0,3,4],[],[0,1,2,4,5],[],[0,1,2,3,5],[0,2,3,5],[2,3,5],[],[0,1,2,3,4]]#18 steps gait
    # THIS IS THE VERY GOOD ONE (BELOW)
    # states = np.array([[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 1.0, 1.0, 0.0, 1.0, 1.0],[0.0, 1.0, 0.1, 0.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 0.1, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 0.0],[1.0, 0.1, 0.0, 1.0, 1.0, 0.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 0.1, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 0.0, 1.0],[0.1, 0.0, 1.0, 1.0, 0.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 0.1]])#15 steps gait
    # ignore = [[],[0,1,2,4,5],[1,4,5],[],[0,1,2,3,5],[],[0,1,2,3,4],[0,3,4],[],[0,1,2,4,5],[],[0,1,2,3,5],[2,3,5],[],[0,1,2,3,4]]#15 steps gait
    # ABOVE IS THE VERY GOOD ONE

    # states = np.array([[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[0.0, 1.0, 0.1, 0.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 0.1, 1.0, 1.0, 0.1, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 0.1, 0.0, 1.0, 1.0, 0.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [0.1, 1.0, 1.0, 0.1, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[0.1, 0.0, 1.0, 1.0, 0.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 1.0, 0.1, 1.0, 1.0, 0.1]])#12 steps gait
    # ignore = [[],[1,4,5],[],[0,2,3,5],[],[0,3,4],[],[1,2,4,5],[],[2,3,5],[],[0,1,3,4]]#12 steps gait

    # states = np.array([[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[0.0, 1.0, 0.1, 0.0, 1.0, 1.0], [1.0, 0.1, 1.0, 1.0, 0.1, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 0.1, 0.0, 1.0, 1.0, 0.0], [0.1, 1.0, 1.0, 0.1, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[0.1, 0.0, 1.0, 1.0, 0.0, 1.0],[1.0, 1.0, 0.1, 1.0, 1.0, 0.1]])#12 steps gait
    # ignore = [[],[1,4,5],[0,2,3,5],[],[0,3,4],[1,2,4,5],[],[2,3,5],[0,1,3,4]]#12 steps gait

    # # BEST GAIT
    # quasi-static rolling
    states = np.array([[0.0, 1.0, 0.1, 0.0, 1.0, 1.0], [0.8, 0.1, 1.0, 1.0, 0.1, 1.0], [0.8, 0.1, 0.0, 1.0, 1.0, 0.0], [0.1, 1.0, 1.0, 0.1, 1.0, 1.0], [0.1, 0.0, 1.0, 1.0, 0.0, 1.0],[0.8, 1.0, 0.1, 1.0, 1.0, 0.1]])#6 steps gait
    states = np.array([[0.0, 1.0, 0.1, 0.0, 1.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]])#steps gait
    states = np.array([[0.0, 1.0, 1.0, 0.0, 1.0, 0.1],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]]) # one step and recover
    states = np.array([[0.0, 1.0, 1.0, 0.0, 1.0, 0.1],[1.0, 0.1, 1.0, 1.0, 0.1, 1.0],
                       [1.0, 1.0, 0.0, 1.0, 0.1, 0.0],[0.1, 1.0, 1.0, 0.1, 1.0, 1.0],
                       [1.0, 0.0, 1.0, 0.1, 0.0, 1.0],[1.0, 1.0, 0.1, 1.0, 1.0, 0.1]]) # quasi-static rolling

    states = np.array([[0.0, 1.0, 1.0, 0.0, 0.9, 0.1],
                       [1.0, 1.0, 0.0, 0.9, 0.1, 0.0],
                       [1.0, 0.0, 1.0, 0.1, 0.0, 0.9]]) # dynamic rolling

    # starting with 3 and 6:
    # states = np.array([[1.0, 0.1, 0.0, 1.0, 1.0, 0.0], [0.1, 1.0, 1.0, 0.1, 1.0, 1.0], [0.1, 0.0, 1.0, 1.0, 0.0, 1.0],[0.8, 1.0, 0.1, 1.0, 1.0, 0.1], [0.0, 1.0, 0.1, 0.0, 1.0, 1.0], [0.8, 0.1, 1.0, 1.0, 0.1, 1.0]])#6 steps gait
    # modified quasi-static rolling for going uphill
    # states = np.array([[0.0, 1.0, 0.1, 0.0, 1.0, 1.0], [0.8, 0.0, 1.0, 1.0, 0.0, 1.0], [0.8, 0.1, 0.0, 1.0, 1.0, 0.0], [0.0, 1.0, 1.0, 0.0, 1.0, 1.0], [0.1, 0.0, 1.0, 1.0, 0.0, 1.0],[0.8, 1.0, 0.0, 1.0, 1.0, 0.0]])#6 steps gait
    # ignore = [[1,4,5],[0,2,3,5],[0,3,4],[1,2,4,5],[2,3,5],[0,1,3,4]]#6 steps gait
    # # BEST GAIT

    # turning gait? 6 steps
    #states = np.array([[0.0, 1.0, 0.1, 0.0, 1.0, 1.0], [1.0, 0.5, 1.0, 1.0, 0.1, 0.5], [1.0, 0.1, 0.0, 1.0, 1.0, 0.0], [0.5, 1.0, 1.0, 0.1, 1.0, 1.0], [0.1, 0.0, 1.0, 1.0, 0.0, 1.0],[1.0, 1.0, 0.5, 1.0, 1.0, 0.1]])#6 steps gait
    #ignore = [[1,4,5],[0,2,3,5],[0,3,4],[1,2,4,5],[2,3,5],[0,1,3,4]]#6 steps gait

    # turning gait? 3 steps
    # states = np.array([[0.0, 1.0, 1.0, 0.0, 1.0, 0.1], [1.0, 1.0, 0.0, 1.0, 0.1, 0.0], [1.0, 0.0, 0.8, 0.1, 0.0, 1.0]])#3 steps gait
    # no transition gait 3 steps
    # states = np.array([[0.0, 0.9, 0.1, 0.0, 1.0, 1.0], [0.9, 0.1, 0.0, 1.0, 1.0, 0.0], [0.1, 0.0, 0.9, 1.0, 0.0, 1.0]])#3 steps gait
    #modified no transition gait
    # states = np.array([[0.0, 0.9, 0.1, 0.0, 1.0, 1.0], [0.8, 0.1, 0.0, 1.0, 1.0, 0.0], [0.1, 0.0, 0.9, 1.0, 0.0, 1.0]])#3 steps gait
    # left-turning gait? Nope...
    # states = np.array([[0.0, 0.9, 0.1, 0.0, 1.0, 1.0], [0.0, 0.9, 0.1, 0.0, 0.9, 0.1], [0.9, 0.1, 0.0, 1.0, 1.0, 0.0], [0.9, 0.1, 0.0, 0.9, 0.1, 0.0], [0.1, 0.0, 0.9, 1.0, 0.0, 1.0], [0.1, 0.0, 0.9, 0.1, 0.0, 0.9]])#6 steps gait
    #ignore = [[1,4,5],[0,2,3,5],[0,3,4],[1,2,4,5],[2,3,5],[0,1,3,4]]#6 steps gait
    
    # no transition gait 3 steps
    #states = np.array([[0.0, 1.0, 0.1, 0.0, 1.0, 1.0], [1.0, 0.1, 0.0, 1.0, 1.0, 0.0], [0.1, 0.0, 1.0, 1.0, 1.0, 0.0]])#3 steps gait

    # recovery
    # states = np.array([[1.0]*num_motors])
    # tol = 0.03
    # max_speed = 99
    # recovery

    # experimental gaits

    # # crawling gait
    # states = np.array([[1.0,1.0,1.0,1.0,1.0,1.0],[0.0,1.0,1.0,1.0,1.0,1.0]])
    # # turning clockwise
    # states = np.array([[1.0,1.0,1.0,1.0,1.0,1.0],[0.1,0.5,1.0,0.1,1.0,1.0]])
    # # turn the other direction (actually the same direction??)
    # states = np.array([[1.0,1.0,1.0,1.0,1.0,1.0],[0.1,1.0,1.0,0.1,1.0,0.5]])

    # # starting with 5 and 2 on the ground
    # # crawling gait
    # states = np.array([[1.0,1.0,1.0,1.0,1.0,1.0],[1.0,0.2,1.0,1.0,0.2,1.0]])

    # # turning counterclockwise
    # states = np.array([[0.1,1.0,1.0,1.0,1.0,1.0],[0.8,1.0,1.0,0.3,1.0,1.0]])
    # # multi-step turning
    # # states = np.array([[1.0,1.0,1.0,0.1,1.0,1.0],[0.3,1.0,1.0,0.8,1.0,1.0]])

    # states = np.array([[1.0,1.0,1.0,1.0,0.0,1.0],[1.0,0.0,1.0,1.0,1.0,1.0]])
    # states = np.array([[1.0,0.0,1.0,1.0,1.0,1.0],[1.0,1.0,1.0,1.0,0.0,1.0]])

    #ignore = [[]]
    #states = np.array([[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[0.0, 1.0, 0.2, 0.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 0.2, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 0.2, 0.0, 1.0, 1.0, 0.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 0.2, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [0.2, 0.0, 1.0, 1.0, 0.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],[1.0, 1.0, 1.0, 1.0, 1.0, 0.2]])#12 steps gait
    #ignore = [[],[1,4,5],[],[],[],[0,3,4],[],[],[],[2,3,5],[],[]]#12 steps gait
    
    # states = np.array([[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]])
    # tol = 0.03

    # after the chirality changed
    # states = np.array([[0.0, 0.1, 1.0, 0.0, 1.0, 1.0], [1.0, 1.0, 0.1, 1.0, 1.0, 0.1],[1.0, 0.0, 0.1, 1.0, 0.0, 1.0],[0.1, 1.0, 1.0, 0.1, 1.0, 1.0],[0.1, 1.0, 0.0, 1.0, 1.0, 0.0], [1.0, 0.1, 1.0, 1.0, 0.1, 1.0]])#6 steps gait
    # states = np.array([[0.0, 0.1, 0.9, 0.0, 0.8, 1.0],[0.9, 0.0, 0.1, 1.0, 0.0, 0.8],[0.1, 0.9, 0.0, 0.8, 1.0, 0.0]])#3 steps gait
    # states = np.array([[0.0, 0.2, 1.0, 0.0, 1.0, 1.0],[1.0, 0.0, 0.2, 1.0, 0.0, 1.0],[0.2, 1.0, 0.0, 1.0, 1.0, 0.0]])#3 steps gait
    # states = np.array([[0.0, 0.1, 1.0, 0.0, 1.0, 1.0]])#3 steps gait
    # states = np.array([[0.0, 0.1, 1.0, 0.0, 1.0, 1.0],[1.0, 0.0, 1.0, 1.0, 0.0, 0.1],[0.1, 1.0, 0.0, 1.0, 1.0, 0.0]])#3 steps gait

    # states = np.array([[0.0, 0.1, 0.9, 0.0, 0.8, 1.0],
    #                    [1.0, 1.0, 0.1, 1.0, 1.0, 0.1],
    #                    [0.9, 0.0, 0.1, 1.0, 0.0, 1.0],
    #                    [0.1, 1.0, 1.0, 0.1, 1.0, 1.0],
    #                    [0.1, 0.9, 0.0, 1.0, 1.0, 0.0],
    #                    [1.0, 0.1, 1.0, 1.0, 0.1, 1.0]])

    # one actuator crawling
    # num_motors = 1
    # states = np.array([[1.0],[0.0]])
    # states = np.array([[1.0,1.0,1.0,1.0,1.0,1.0],[0.0,1.0,1.0,1.0,1.0,1.0]])

    # # contactless trials
    # states = np.array([[1.0,1.0,1.0,1.0,1.0,1.0],
    #                    [0.2,1.0,1.0,1.0,1.0,1.0],
    #                    [1.0,1.0,1.0,1.0,1.0,1.0],
    #                    [1.0,0.2,1.0,1.0,1.0,1.0],
    #                    [1.0,1.0,1.0,1.0,1.0,1.0],
    #                    [1.0,1.0,0.2,1.0,1.0,1.0],
    #                    [1.0,1.0,1.0,1.0,1.0,1.0],
    #                    [1.0,1.0,1.0,0.2,1.0,1.0],
    #                    [1.0,1.0,1.0,1.0,1.0,1.0],
    #                    [1.0,1.0,1.0,1.0,0.2,1.0],
    #                    [1.0,1.0,1.0,1.0,1.0,1.0],
    #                    [1.0,1.0,1.0,1.0,1.0,0.2]])

    # states = np.array([[1.0]*num_motors,[0.2]*num_motors])


    # states = np.array([[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]])

    # states = np.array([[1.0, 1.0, 1.0, 0.3, 0.3, 0.3]])

    # states = np.array([[0.6]*num_motors])

    # states = np.array([[1.0,1.0,1.0,1.0,1.0,1.0],[1.0,1.0,1.0,0.2,1.0,1.0]])

    num_steps = len(states)
    state = 0
    done = np.array([False] * num_motors)
    try:
        # the 0-th arg is the name of the file itself, so we want the 1st.
        tensegrity_run(sys.argv[1])
    except KeyboardInterrupt:
        # why is this here?
        pass
