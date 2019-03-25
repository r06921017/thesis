#! /usr/bin/python2
# -*- coding: utf-8 -*-

import rospy
from jsk_gui_msgs.msg import VoiceMessage
from std_msgs.msg import String
pub = rospy.Publisher('/pepper_come', String, queue_size=10)


def voice_callback(data):  # voice trigger
	global pub
	sep = data.texts[0].split(' ')

	if 'office' in sep:
		if 'come' and 'to' and ('me' or 'Me') in sep:
			print sep
			pub.publish('office')
			rospy.set_param('voice_location', 'office')
			return

		if 'go' and 'to' in sep:
			print sep
			pub.publish('office')
			rospy.set_param('voice_location', 'office')
			return

	elif 'living' in sep:
		if 'come' and 'to' and ('me' or 'Me') in sep:
			print sep
			pub.publish('living')
			rospy.set_param('voice_location', 'living')
			return

		if 'go' and 'to' in sep:
			print sep
			pub.publish('living')
			rospy.set_param('voice_location', 'living')
			return

	elif 'dining' in sep:
		if 'come' and 'to' and ('me' or 'Me') in sep:
			print sep
			pub.publish('dining')
			rospy.set_param('voice_location', 'dining')
			return

		if 'go' and 'to' in sep:
			print sep
			pub.publish('dining')
			rospy.set_param('voice_location', 'dining')
			return

	elif 'bedroom' in sep:
		if 'come' and 'to' and ('me' or 'Me') in sep:
			print sep
			pub.publish('bedroom')
			rospy.set_param('voice_location', 'bedroom')
			return

		if 'go' and 'to' in sep:
			print sep
			pub.publish('bedroom')
			rospy.set_param('voice_location', 'bedroom')
			return

	elif 'hey' in sep:
		print sep
		pub.publish('hey')
		return

	elif 'start' in sep:
		print sep
		pub.publish('scenario2')
		return

	elif ('guests' or 'guest') in sep:
		print sep
		pub.publish('guest')
		rospy.set_param('voice_location', 'init')

	elif 'schedule' in sep:
		pub.publish('schedule')


if __name__ == '__main__':

	while not rospy.is_shutdown():
		rospy.init_node('pepper_st')
		rospy.Subscriber('/Tablet/voice', VoiceMessage, voice_callback)
		rospy.spin()
