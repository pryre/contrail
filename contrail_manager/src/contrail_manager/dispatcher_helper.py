import rospy
import actionlib

from math import *

from contrail.msg import TrajectoryAction, TrajectoryGoal

class DispatcherHelper():
	def __init__(self, action_topic, wait_for_contrail=False):
		self.client = actionlib.SimpleActionClient(action_topic, TrajectoryAction)

		self.dip = False

		if wait_for_contrail:
			self.client.wait_for_server()

	def wait(self):
		self.client.wait_for_result()

	def dispatch_continuous(self, wps, duration, done_cb=None):
			goal = TrajectoryGoal()

			goal.start = rospy.Time.now() + rospy.Duration.from_sec(1)
			goal.duration = rospy.Duration.from_sec(duration)

			for i in range(len(wps)):
				goal.positions.append(wps[i].position)
				goal.yaws.append(wps[i].yaw)

			self.action_waiting = True
			self.client.send_goal(goal)

	def dispatch_discrete(self, wps, nom_lvel, nom_rvel):
		self.discrete_counter = 0
		self.discrete_wps = wps
		self.discrete_nom_lvel = nom_lvel
		self.discrete_nom_rvel = nom_rvel

		self.next_discrete_wp()

	def discrete_in_progress(self):
		return self.dip

	def next_discrete_wp(self):
		if self.discrete_counter < (len(self.discrete_wps) - 1):
			self.dip = True

			i = self.discrete_counter

			dx = self.discrete_wps[i+1].position.x - self.discrete_wps[i].position.x
			dy = self.discrete_wps[i+1].position.y - self.discrete_wps[i].position.y
			dz = self.discrete_wps[i+1].position.z - self.discrete_wps[i].position.z

			lt = sqrt((dx*dx)+(dy*dy)+(dz*dz)) / self.discrete_nom_lvel
			rt = (self.discrete_wps[i+1].yaw - self.discrete_wps[i].yaw ) / self.discrete_nom_rvel

			self.dispatch_continuous([self.discrete_wps[i], self.discrete_wps[i+1]], max([lt,rt]), done_cb=self.next_discrete_wp)

			self.discrete_counter += 1
		else:
			self.dip = False
