import rospy
# from tensegrity.msg import Motor, MotorsStamped, Sensor, SensorsStamped, Imu, ImuStamped
from tensegrity.msg import Motor, Sensor, Imu, TensegrityStamped
# from geometry_msgs.msg import QuaternionStamped
from random import random
from time import sleep

if __name__ == '__main__':

    rospy.init_node('tensegrity')

    control_pub = rospy.Publisher('control_msg',TensegrityStamped,queue_size=10)
    # strain_pub = rospy.Publisher('strain_msg',SensorsStamped,queue_size=10)
    # imu_pub = rospy.Publisher('imu_msg',ImuStamped,queue_size=10)

    while not rospy.is_shutdown():
        # rospy.sleep(1.0/30)
        # generate messages
        control_msg = TensegrityStamped()
        # control_msg = MotorsStamped()
        # strain_msg = SensorsStamped()
        # imu_msg = ImuStamped()

        # get timestamp
        timestamp = rospy.Time.now()
        control_msg.header.stamp = timestamp
        # strain_msg.header.stamp = timestamp
        # imu_msg.header.stamp = timestamp

        # random data
        for motor_id in range(6):
            motor = Motor()
            motor.id = motor_id
            motor.target = random()
            motor.speed = random()*100
            # motor.forward = True
            motor.done = True
            control_msg.motors.append(motor)

        values = [167,171,173,173,180,155,310,309,305]
        for sensor_id in range(9):
            sensor = Sensor()
            sensor.id = sensor_id
            sensor.length = values[sensor_id]
            sensor.capacitance = random()*2 + 19
            control_msg.sensors.append(sensor)

        for imu_id in range(2):
            IMU = Imu()
            IMU.id = imu_id
            IMU.x = 0
            IMU.y = 1
            IMU.z = 0
            control_msg.imus.append(IMU)

        control_pub.publish(control_msg)
        # strain_pub.publish(strain_msg)
        # imu_pub.publish(imu_msg)
        rospy.sleep(.1)