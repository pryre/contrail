import os
import math
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from geometry_msgs.msg import Pose, Vector3, Quaternion, PoseStamped

import actionlib
from contrail.msg import TrajectoryAction, TrajectoryGoal

class Commander(Plugin):
	def __init__(self, context):
		super(Commander, self).__init__(context)
		# Give QObjects reasonable names
		self.setObjectName('MantisCommander')
		rp = rospkg.RosPack()

		# Process standalone plugin command-line arguments
		#from argparse import ArgumentParser
		#parser = ArgumentParser()
		# Add argument(s) to the parser.
		#parser.add_argument("-q", "--quiet", action="store_true",
		#              dest="quiet",
		#              help="Put plugin in silent mode")
		#args, unknowns = parser.parse_known_args(context.argv())
		#if not args.quiet:
		#    print 'arguments: ', args
		#    print 'unknowns: ', unknowns

		# Create QWidget
		self._widget = QWidget()
		# Get path to UI file which is a sibling of this file
		# in this example the .ui and .py file are in the same folder
		ui_file = os.path.join(rp.get_path('rqt_contrail_commander'), 'resource', 'ContrailCommander.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._widget.setObjectName('ContrailCommanderUi')
		# Show _widget.windowTitle on left-top of each plugin (when
		# it's set in _widget). This is useful when you open multiple
		# plugins at once. Also if you open multiple instances of your
		# plugin at once, these lines add number to make it easy to
		# tell from pane to pane.ns
		if context.serial_number() > 1:
			self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
		# Add widget to the user interface
		context.add_widget(self._widget)

		self._widget.textbox_pose.textChanged.connect(self.pose_changed)
		self._widget.textbox_contrail.textChanged.connect(self.contrail_changed)

		self._widget.button_send_command.clicked.connect(self.button_send_command_pressed)

		self.has_pose = False
		self.current_pose = None
		self.sub_pose = None
		self.client_base = None

	def shutdown_plugin(self):
		if self.sub_pose is not None:
			self.sub_pose.unregister()

	def save_settings(self, plugin_settings, instance_settings):
		instance_settings.set_value('pose_prefix', self._widget.textbox_pose.text())
		instance_settings.set_value('contrail_prefix', self._widget.textbox_contrail.text())

	def restore_settings(self, plugin_settings, instance_settings):
		pose = "/uav/pose"
		contrail = "/mavel/contrail"

		if instance_settings.value('pose_prefix') is not None:
			pose = str(instance_settings.value('pose_prefix'))

		if instance_settings.value('contrail_prefix') is not None:
			contrail = str(instance_settings.value('contrail_prefix'))

		self._widget.textbox_pose.setText(pose)
		self._widget.textbox_contrail.setText(contrail)

		self.pose_changed()
		self.contrail_changed()

		#self.sub_pose = rospy.Subscriber(self._widget.textbox_pose.text(), PoseStamped, self.callback_pose)
		#self.client_base = actionlib.SimpleActionClient(self._widget.textbox_contrail.text(), TrajectoryAction)

	#def trigger_configuration(self):
		# Comment in to signal that the plugin has a way to configure
		# This will enable a setting button (gear icon) in each dock widget title bar
		# Usually used to open a modal configuration dialog

	def quaternion_to_rpy(self,q):
		if not isinstance(q, Quaternion):
			raise TypeError('Input should be a geometry_msgs/Quaternion')

		q2sqr = q.y * q.y;
		t0 = -2.0 * (q2sqr + q.z * q.z) + 1.0
		t1 = 2.0 * (q.x * q.y + q.w * q.z)
		t2 = -2.0 * (q.x * q.z - q.w * q.y)
		t3 = 2.0 * (q.y * q.z + q.w * q.x)
		t4 = -2.0 * (q.x * q.x + q2sqr) + 1.0

		if(t2 > 1.0):
			t2 = 1.0
		elif(t2 < -1.0):
			t2 = -1.0

		e = Vector3()
		roll = math.atan2(t3, t4);
		pitch = math.asin(t2);
		yaw = math.atan2(t1, t0);

		return (roll,pitch,yaw)

	def pose_changed(self):
		if self.sub_pose is not None:
			self.sub_pose.unregister()

		try:
			self.sub_pose = rospy.Subscriber(self._widget.textbox_pose.text(), PoseStamped, self.callback_pose)
		except:
			rospy.logwarn("Unable to subscribe to pose: %s" % self._widget.textbox_pose.text())
			self.sub_pose = None

	def contrail_changed(self):
		try:
			self.client_base = actionlib.SimpleActionClient(self._widget.textbox_contrail.text(), TrajectoryAction)
		except:
			rospy.logwarn("Unable to subscribe to action: %s" % self._widget.textbox_contrail.text())
			self.client_base = None

	def button_send_command_pressed(self):
		rospy.logdebug("Send command pressed!")

		if self.client_base is not None:
			if self.client_base.wait_for_server(rospy.Duration(0.5)):
				try:
					if self.has_pose:
						dur = rospy.Duration(float(self._widget.textbox_goto_duration.text()))

						pos_x = float(self._widget.textbox_goto_pos_x.text())
						pos_y = float(self._widget.textbox_goto_pos_y.text())
						pos_z = float(self._widget.textbox_goto_pos_z.text())
						yaw = float(self._widget.textbox_goto_yaw.text())
						_, _, yaw_c = self.quaternion_to_rpy(self.current_pose.orientation)

						goal_base = TrajectoryGoal()
						goal_base.duration = dur
						goal_base.positions = [Vector3(self.current_pose.position.x,self.current_pose.position.y,self.current_pose.position.z),
											   Vector3(pos_x,pos_y,pos_z)]
						goal_base.yaws = [yaw_c, yaw]

						timestamp = rospy.Time.now() + rospy.Duration.from_sec(0.5)

						goal_base.start = timestamp
						self.client_base.send_goal(goal_base)

						rospy.loginfo("Sending command...")
					else:
						rospy.logerr("No state information received, can't generate command action!")
				except:
					rospy.logerr("Unexpected error!")
			else:
				rospy.logerr("Not connected to action server!")
		else:
			rospy.logerr("Action server not set up!")

	def callback_pose(self, msg_in):
		self.has_pose = True
		self.current_pose = msg_in.pose
