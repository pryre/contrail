import rospy

from contrail_msgs.msg import Waypoint

def load_waypoints():
	if( rospy.has_param("~waypoints/wp0/x") and \
		rospy.has_param("~waypoints/wp0/y") and \
		rospy.has_param("~waypoints/wp0/z") and \
		rospy.has_param("~waypoints/wp0/yaw") ):

		i = 0
		waypoints = []
		while( rospy.has_param("~waypoints/wp%i/x" % (i)) and \
			   rospy.has_param("~waypoints/wp%i/y" % (i)) and \
			   rospy.has_param("~waypoints/wp%i/z" % (i)) and \
			   rospy.has_param("~waypoints/wp%i/yaw" % (i)) ):

			waypoints.append(Waypoint())
			waypoints[i].position.x = rospy.get_param("~waypoints/wp%i/x" % (i))
			waypoints[i].position.y = rospy.get_param("~waypoints/wp%i/y" % (i))
			waypoints[i].position.z = rospy.get_param("~waypoints/wp%i/z" % (i))
			waypoints[i].yaw = rospy.get_param("~waypoints/wp%i/yaw" % (i))

			i += 1

		return (True, waypoints)

	else:
		return (False, None)

