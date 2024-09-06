This file describes the format of the .json data files included in these zip folders.  In addition to the .json files in the "data" directory, you should have a directory called "color" that gives the corresponding RGB images at each timestep, a directory called "depth" that gives the corresponding depth images at each timestep, a pairs.csv file that defines which sensors are betwen which motion capture markers (using their id#s), and a labels.png file that defines which colored endcaps in the RGB image correspond to which motion capture markers.  More information about pairs.csv can be found at the bottom of this file.

header: information from the ROS header of the data messages
	secs: The timestamp in seconds
	seq: The message number in sequential order

sensors: information from the capacitive strain sensors indexed by sensor id#
	sensor_id#:
		capacitance: the capacitance of the sensor in picofarads
		length: the length of the sensor in mm (calculated from the capacitance)

motors: information about the motors and control commands indexed by motor id#
	motor_id#:
		position: the cable's current position between 0 (full contraction) and 1 (full extension)
		target: the motor's commanded target between 0 (full contraction) and 1 (full extension)
		speed: the motor's commanded speed between -99 (backward) and 99 (forward)
		done: TRUE if the motor has reached its target and stopped; FALSE otherwise

mocap: information from the motion capture system indexed by marker id#
	marker_id#: "null" if the marker is not seen, else
		x: the x-position of this marker in mm in the frame of the motion capture system
		y: the y-position of this marker in mm in the frame of the motion capture system
		z: the z-position of this marker in mm in the frame of the motion capture system


imu: oritentation information from the onboard IMUs indexed by imu id#
	imu_id#:
		x: the x-component of a vector pointing in the direction of the IMUs bar in the global frame where +x is South and +y is Up.
		y: the y-component of a vector pointing in the direction of the IMUs bar in the global frame where +x is South and +y is Up.
		z: the z-component of a vector pointing in the direction of the IMUs bar in the global frame where +x is South and +y is Up.

======== PAIRS.CSV ========
The pairs.csv file has as many lines as there are strain sensors.  Each line corresponds to the sensor with that line number and it contains two marker numbers separated by a comma.  The sensor number corresponding to that line is between the nodes labeled with those marker numbers.  For example, if pairs.csv is:

0,2
4,5
1,3

Then...

Sensor 0 is the tendon between Node 0 and Node 2.
Sensor 1 is the tendon between Node 4 and Node 5.
Sensor 2 is the tendon between Node 1 and Node 3.

The length of Sensor 0 as measured my the motion capture system is the distance between the points data.mocap.0 and data.mocap.2.