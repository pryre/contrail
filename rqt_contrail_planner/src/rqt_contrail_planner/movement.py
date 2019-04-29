
from rqt_contrail_planner.waypoint import Waypoint

class Movement():
	def __init__(self, filename="", movement=None):
		self.is_valid = False
		self.is_discrete = False
		self.duration = 0.0
		self.nom_vel = 0.0
		self.nom_rate = 0.0
		self.waypoints = []
		self.filename = filename

		if movement:
			self.parse_movement(movement)

	def parse_movement(self,m):
		if 'waypoints' in m:
			try:
				mode_ok = False
				wps = m['waypoints']

				if wps['mode'] == 'continuous':
					self.duration = float(wps['duration'])
					mode_ok = True
				elif wps['mode'] == 'discrete':
					self.is_discrete = True
					self.nom_vel = float(wps['nominal_velocity'])
					self.nom_rate = float(wps['nominal_yawrate'])
					mode_ok = True

				if mode_ok:
					self.waypoints = []

					i = 0
					while 'wp'+str(i) in wps:
						self.waypoints.append(Waypoint(wp_yaml=wps['wp'+str(i)]))
						#rospy.loginfo(self.waypoints[i])
						i+=1

					self.is_valid = True
			except KeyError as e:
				rospy.logerr("File malformed: %s" % e)
		else:
			rospy.logerr("File not recognised!")

	def encode_yaml(self):
		yaml_move = {}
		yaml_move['waypoints'] = {}

		if self.is_discrete:
			yaml_move['waypoints']['mode'] = 'discrete'
			yaml_move['waypoints']['duration'] = float(self.nom_vel)
			yaml_move['waypoints']['duration'] = float(self.nom_rate)
		else:
			yaml_move['waypoints']['mode'] = 'continuous'
			yaml_move['waypoints']['duration'] = float(self.duration)

		for i in xrange(len(self.waypoints)):
			yaml_move['waypoints']['wp' + str(i)] = self.waypoints[i].encode_yaml()

		return yaml_move
