
class Waypoint():
	def __init__(self, wp_yaml=None, x=0.0, y=0.0, z=0.0, yaw=0.0):
		self.x = float(x)
		self.y = float(y)
		self.z = float(z)
		self.yaw = float(yaw)

		if wp_yaml:
			self.x = float(wp_yaml['x'])
			self.y = float(wp_yaml['y'])
			self.z = float(wp_yaml['z'])
			self.yaw = float(wp_yaml['yaw'])

	def encode_yaml(self):
		wp = {}
		wp['x'] = self.x
		wp['y'] = self.y
		wp['z'] = self.z
		wp['yaw'] = self.yaw

		return wp

	def __repr__(self):
		return "Waypoint()"

	def __str__(self):
		return "<x=%0.2f, y=%0.2f, z=%0.2f; yaw=%0.2f>" % (self.x, self.y, self.z, self.yaw)
