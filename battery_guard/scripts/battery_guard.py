#!/usr/bin/env python

import rospy

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

if __name__ == '__main__':
	rospy.init_node('battery_guard')
	soundclient = SoundClient()

	last_complain= rospy.Time()
	r= rospy.Rate(.1)
	while not rospy.is_shutdown():
		with open('/sys/class/power_supply/BAT0/capacity') as f:
			capacity= int(f.readline().rstrip())
		with open('/sys/class/power_supply/BAT0/status') as f:
			status= f.readline().rstrip()
		if status != "Charging" and rospy.Time.now() - last_complain > rospy.Duration(60):
			if capacity < 10:
				rospy.logerr("No battery power remaining. Connect the robot laptop to power immediately.")
				soundclient.play(SoundRequest.NEEDS_PLUGGING_BADLY)
				last_complain= rospy.Time.now()
			elif capacity < 15:
				rospy.logwarn("Only little battery power remaining. Please connect the robot laptop to power.")
				soundclient.play(SoundRequest.NEEDS_PLUGGING)
				last_complain= rospy.Time.now()
		r.sleep()
