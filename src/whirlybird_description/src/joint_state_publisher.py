#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from whirlybird_msgs.msg import Whirlybird

class JointStatePublisher():

	def __init__(self):
		self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size = 1)
		self.whirlybird_sub = rospy.Subscriber('whirlybird', Whirlybird, self.whirlybird_callback)
		rospy.spin()

	def whirlybird_callback(self, msg):
		state = JointState()
		state.header.stamp = rospy.Time.now()
		state.name = ['yaw_joint', 'pitch_joint', 'roll_joint']
		state.position = [-msg.yaw, -msg.pitch, msg.roll] # switch from NED to NWU
		self.joint_state_pub.publish(state)

if __name__ == '__main__':
	rospy.init_node('joint_state_publisher')
	try:
		jsp = JointStatePublisher()
	except:
		rospy.ROSInterruptException
	pass
