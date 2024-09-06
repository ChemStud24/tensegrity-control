import sys
from argparse import ArgumentParser
import serial
import time
import math
from math import cos,sin,tan
import re
import json
import xlrd
import numpy as np
from pynput import keyboard
from scipy.spatial.transform import Rotation as R

# ROS messages
import rospy
from tensegrity.msg import Motor, Info, MotorsStamped, Sensor, SensorsStamped, Imu, ImuStamped

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

def read_calibration_file(filename):
    if filename[-4:] == '.xls':
        # hand calibration excel file
        workbook = xlrd.open_workbook(filename)
        shortsheet = workbook.sheet_by_name('Short Sensors')
        longsheet = workbook.sheet_by_name('Long Sensors')
        m = np.array([float(shortsheet.cell_value(9,col)) for col in xrange(0,12,2)] + [float(longsheet.cell_value(10,col)) for col in xrange(0,6,2)])
        b = np.array([float(shortsheet.cell_value(9,col)) for col in xrange(1,13,2)] + [float(longsheet.cell_value(10,col)) for col in xrange(1,7,2)])
    elif filename[-5:] == '.json':
        # autocalibration JSON file
        data = json.load(open(filename))
        m = np.array(data.get('m'))
        b = np.array(data.get('b'))
    else:
        error('Invalid calibration file')
    return m,b

def read(serial_port):
    flush(serial_port)
    global state
    global count
    global capacitance
    global acceleration
    global orientation
    global imu
    global P
    global max_speed
    global RANGE
    line = serial_port.read_until(b'\n')# read data string sent from central
    line = str(line)#.encode('utf-8')# convert to string
    line = line.split('*')[-1] # remove padding
    # print('Whole Line: ',line)
    s = re.findall(r"0x[0-9a-f]+", line)# extract strain data (hex number) from the data string
    q = re.findall(r"[-+]?\d*\.\d+|\d+", line)# extract imu data (decimal number) from the data string
    if len(line) >=2 :# valid data string will be longer than 9 characters    
        # print('line')
        print(line)
        if line[0] == "S":# read strain data
            # now_time = time.time()
            # meas_time = now_time - start_time
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
                # print(state)
                print("Position: ",pos)
                print("Target: ",states[state])
                # print(pos)
                # print(states[state])
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
            else:
                print('+++++')

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

            k=np.array([cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch)])
            r = R.from_rotvec(np.pi/2 * np.array([0, 1, 0]))
            k = r.apply(k)
            y=np.array([0,1,0])
            s=np.cross(k,y)
            v=np.cross(s,k)
            vrot=v*cos(roll)+np.cross(k,v)*sin(roll)
            imu[int(q[0])-1] = np.cross(k,vrot)
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
    # gait info
    info = Info()
    info.min_length = min_length
    info.RANGE = RANGE
    info.max_speed = max_speed
    info.tol = tol
    info.P = P
    info.I = I
    info.D = D
    control_msg.info = info
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
    # finishing setup.
    print("Opened port. Ctrl-C to stop.")
    
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
            end_command(serial_port, "s", 0)
            sys.exit()

    # Nicely shut down this script.
    print("\nShutting down serial_tx_cmdline...")
    send_command(serial_port, "s", 0)
    sys.exit()

def onpress(key):
    global keep_going
    global states
    global tol
    global done
    global P
    global max_speed
    global RANGE
    if key == keyboard.KeyCode.from_char('q'):
        keep_going = False
    elif key == keyboard.KeyCode.from_char('r'):
        states = np.array([[1.0]*num_motors]*num_steps)
        done = np.array([False] * num_motors)
        tol = 0.03
        P = 5.0
        max_speed = 60
        # RANGE = 80
    elif key == keyboard.KeyCode.from_char('p'):
        states = np.array([[-0.1]*num_motors]*num_steps)
        done = np.array([False] * num_motors)
        tol = 0.3
        P = 5.0
        max_speed = 60
        RANGE = 80


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

    parser = ArgumentParser()
    parser.add_argument("--tol", default=0.1)
    parser.add_argument("--min_length", default=100) #mm
    parser.add_argument("--RANGE", default="pcd/yale/struct_with_socks_new.ply")
    parser.add_argument("--top_endcap_mesh_file", default="pcd/yale/end_cap_top_new.obj")
    parser.add_argument("--bottom_endcap_mesh_file", default="pcd/yale/end_cap_bottom_new.obj")
    parser.add_argument("--start_frame", default=0, type=int)
    parser.add_argument("--max_correspondence_distances", default=[0.3, 0.15, 0.1, 0.06, 0.03], type=float, nargs="+")
    parser.add_argument("--add_dummy_points", action="store_true")
    parser.add_argument("--num_dummy_points", type=int, default=50)
    parser.add_argument("--dummy_weights", type=float, default=0.1)
    parser.add_argument("--filter_observed_pts", action="store_true")
    parser.add_argument("--add_constrained_optimization", action="store_true")
    parser.add_argument("--add_physical_constraints", action="store_true")
    parser.add_argument("--add_ground_constraints", action="store_true")
    parser.add_argument("-v", "--visualize", action="store_true")
    args = parser.parse_args()

    # calibration_file = '../calibration/autocalibration.json'
    calibration_file = '../calibration/beta calibration.xls'

    # alpha calibration
    # m = np.array([0.01471,0.03030,0.02173,0.04449,0.05502,0.04038,0.03791,0.04207,0.04089])
    # b = np.array([16.798,15.130,16.524,10.523,14.322,14.986,14.018,12.048,13.028])
    # m = np.array([0.01431,0.03204,0.02052,0.04632,0.04941,0.03933,0.03791,0.04207,0.04089])
    # b = np.array([16.851,15.102,16.904,9.558,14.856,14.143,14.018,12.048,13.028])
    m = np.array([0.04437,0.06207,0.02356,0.04440,0.04681,0.05381,0.02841,0.03599,0.03844])
    b = np.array([15.763,13.524,15.708,10.084,15.628,15.208,16.356,12.575,13.506])
    # beta calibration
    # m = np.array([0.04900,0.05697,0.01787,0.05363,0.04362,0.04604,0.03437,0.02701,0.02905])
    # b = np.array([12.764,13.699,15.536,14.558,14.471,13.325,14.494,14.508,15.190])
    # # load autocalibration file
    # data = json.load(open(calibration_file))
    # m = data.get('m')
    # b = data.get('b')

    m,b = read_calibration_file(calibration_file)


    max_speed = 99# set duty cycle as 99 for the max speed, resolution can be improved by changing the bits in C++ code 
    num_sensors = 9# set number of strain sensors
    num_motors = 6# set number of motors
    num_imus = 2#set number of inertial measurement units
    min_length = 100#95#100 #mm #7.2#6.8#6.5 #set minimum length of sensors
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
    # alpha flip
    flip = [1,-1,-1,-1,-1,1]
    # beta flip
    flip = [1,-1,-1,1,-1,1]
    # flip = [-1,-1,1,-1,-1,1]
    acceleration = [0]*3
    orientation = [0]*3
    RANGE = 90#160#90#160 #mm #90
    tol = 0.16#0.13
    tol = 0.2
    tol =0.1
    # tol = 0.07
    #define PID
    # P = 10.0
    P = 6.0
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

    # # back and forth quasi-static gait
    # states = np.array([[0.0, 1.0, 1.0, 0.0, 1.0, 0.1],[1.0, 0.1, 1.0, 1.0, 0.1, 1.0],
    #                    [1.0, 1.0, 0.0, 1.0, 0.1, 0.0],[0.1, 1.0, 1.0, 0.1, 1.0, 1.0],
    #                    [1.0, 0.0, 1.0, 0.1, 0.0, 1.0],[1.0, 1.0, 0.1, 1.0, 1.0, 0.1],
    #                    [1.0, 0.1, 1.0, 1.0, 0.1, 1.0],[0.0, 0.1, 1.0, 0.0, 1.0, 1.0],
    #                    [1.0, 1.0, 0.1, 1.0, 1.0, 0.1],[1.0, 0.0, 0.1, 1.0, 0.0, 1.0],
    #                    [0.1, 1.0, 1.0, 0.1, 1.0, 1.0],[0.1, 1.0, 0.0, 1.0, 1.0, 0.0]]) # quasi-static rolling

    # states = np.array([[0.0, 1.0, 1.0, 0.0, 0.9, 0.1],
    #                    [1.0, 1.0, 0.0, 0.9, 0.1, 0.0],
    #                    [1.0, 0.0, 1.0, 0.1, 0.0, 0.9]]) # dynamic rolling
    # states = np.array([[0.0, 1.0, 1.0, 0.0, 1.0, 0.1],
    #                    [1.0, 1.0, 0.0, 1.0, 0.1, 0.0],
    #                    [1.0, 0.0, 1.0, 0.1, 0.0, 1.0]]) # dynamic rolling

    # # uphill climbing
    # states = np.array([[0.0, 1.0, 0.2, 0.0, 1.0, 0.1],[0.8, 0.1, 0.8, 0.8, 0.1, 0.8],
    #                    [1.0, 0.2, 0.0, 1.0, 0.1, 0.0],[0.1, 0.8, 0.8, 0.1, 0.8, 0.8],
    #                    [0.2, 0.0, 1.0, 0.1, 0.0, 1.0],[0.8, 0.8, 0.1, 0.8, 0.8, 0.1]]) # quasi-static rolling
    # states = np.array([[0.0, 1.0, 1.0, 0.0, 0.8, 0.1],[0.4, 0.1, 1.0, 0.4, 0.1, 1.0],[1.0, 0.1, 1.0, 0.8, 0.1, 0.8],
    #                    [1.0, 1.0, 0.0, 0.8, 0.1, 0.0],[0.1, 1.0, 0.4, 0.1, 1.0, 0.4],[0.1, 1.0, 1.0, 0.1, 0.8, 0.8],
    #                    [1.0, 0.0, 1.0, 0.1, 0.0, 0.8],[1.0, 0.4, 0.1, 1.0, 0.4, 0.1],[1.0, 1.0, 0.1, 0.8, 0.8, 0.1]]) # quasi-static rolling
    # # This is best
    # states = np.array([[0.0, 1.0, 1.0, 0.0, 0.8, 0.1],[0.4, 0.1, 1.0, 0.4, 0.1, 1.0],[1.0, 0.1, 1.0, 0.7, 0.1, 0.7],
    #                    [1.0, 1.0, 0.0, 0.8, 0.1, 0.0],[0.1, 1.0, 0.4, 0.1, 1.0, 0.4],[0.1, 1.0, 1.0, 0.1, 0.7, 0.7],
    #                    [1.0, 0.0, 1.0, 0.1, 0.0, 0.8],[1.0, 0.4, 0.1, 1.0, 0.4, 0.1],[1.0, 1.0, 0.1, 0.7, 0.7, 0.1]]) # quasi-static rolling
    # # right above is best

    # states = np.array([[0.0, 1.0, 1.0, 0.0, 1.0, 0.1],[0.0, 0.1, 1.0, 0.0, 0.1, 1.0],
    #                    [1.0, 1.0, 0.0, 1.0, 0.1, 0.0],[0.1, 1.0, 0.0, 0.1, 1.0, 0.0],
    #                    [1.0, 0.0, 1.0, 0.1, 0.0, 1.0],[1.0, 0.0, 0.1, 1.0, 0.0, 0.1]]) # quasi-static rolling
    
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

    # contactless trials
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

    states = np.array([[0.2]*num_motors,[1.0]*num_motors])

    # max_speed = 60
    # states = np.array([[1.0,0.2,1.0,1.0,0.2,1.0],[1.0,1.0,1.0,1.0,1.0,1.0]])

    num_steps = len(states)
    state = 0
    done = np.array([False] * num_motors)
    try:
        # the 0-th arg is the name of the file itself, so we want the 1st.
        tensegrity_run(sys.argv[1])
    except KeyboardInterrupt:
        # why is this here?
        pass
