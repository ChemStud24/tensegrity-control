import rospy
import numpy as np
from math import ceil
from tensegrity.msg import State, Action
from geometry_msgs.msg import Point

class Tester:

	def __init__(self,trajectory,prev_action='100_100',reverse_the_gait=False):
		sub_topic = '/action_msg'
		pub_topic = '/state_msg'
		self.sub = rospy.Subscriber(sub_topic,Action,self.callback)
		self.pub = rospy.Publisher(pub_topic,State,queue_size=10)

		# message
		self.state_msg = State()
		for x,y in trajectory:
			point = Point()
			point.x = x
			point.y = y
			self.state_msg.trajectory.append(point)
		self.state_msg.prev_action = prev_action
		self.state_msg.reverse_the_gait = reverse_the_gait

		# publish the first message to get the ball rolling
		self.pub.publish(self.state_msg)
		print('I published the message')

	def callback(self,msg):
		print('I finished motion planning')
		# self.pub.publish(self.state_msg)
		for act in msg.actions:
			print(act)
		print(msg.cost)
		for com in msg.COMs:
			print(com)
		for pa in msg.PAs:
			print(pa)
		print('\n')

	def run(self,rate):
		while not rospy.is_shutdown():
			self.pub.publish(self.state_msg)
			rate.sleep()

if __name__ == '__main__':
	rospy.init_node('tester')

	# trajectory
	ppm = 10 # points per meter
	starting_point = [0.1,0.6]
	ending_point = [1.7,0.6]
	num_points = ceil(ppm*(ending_point[0] - starting_point[0]))
	trajectory = np.linspace(ending_point,starting_point,num_points)

	rate = rospy.Rate(1)
	tester = Tester(trajectory)
	tester.run(rate)