#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosgraph
from jsk_gui_msgs.msg import VoiceMessage


def voice_msg_callback(data):
	callerid = data._connection_header['callerid']  # type: 'str'
	master = rosgraph.Master('listener')
	ip = master.lookupNode(callerid)
	ip_num = ip.split(':')[1][2:]  # type: 'str'

	print 'ip_num = ', ip_num
	print 'data = ', data

	return


if __name__ == '__main__':

	rospy.init_node('check_voice_ip', anonymous=True, disable_signals=True)
	rospy.Subscriber('/Tablet/voice', VoiceMessage, voice_msg_callback)
	rospy.loginfo("Start voice recognition!")
	rospy.spin()
