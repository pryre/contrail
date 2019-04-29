import os
import math
import rospkg
import rospy
import yaml

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFileDialog

from rqt_contrail_planner.movement import Movement
from rqt_contrail_planner.waypoint import Waypoint

class Planner(Plugin):
	def __init__(self, context):
		super(Planner, self).__init__(context)
		# Give QObjects reasonable names
		self.setObjectName('ContrailPlanner')
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
		ui_file = os.path.join(rp.get_path('rqt_contrail_planner'), 'resource', 'ContrailPlanner.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._widget.setObjectName('ContrailPlannerUi')
		# Show _widget.windowTitle on left-top of each plugin (when
		# it's set in _widget). This is useful when you open multiple
		# plugins at once. Also if you open multiple instances of your
		# plugin at once, these lines add number to make it easy to
		# tell from pane to pane.
		if context.serial_number() > 1:
			self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
		# Add widget to the user interface
		context.add_widget(self._widget)

		# Interface Functionality
		# Management
		self._widget.button_load.clicked.connect(self.button_load_pressed)
		self._widget.button_save.clicked.connect(self.button_save_pressed)
		self._widget.button_save_as.clicked.connect(self.button_save_as_pressed)

		# Flight Plan
		self._widget.combobox_mode.currentIndexChanged.connect(self.mode_changed)

		# Waypoint
		self._widget.list_waypoints.currentItemChanged.connect(self.list_item_changed)

		self._widget.button_insert.clicked.connect(self.button_insert_pressed)
		self._widget.button_append.clicked.connect(self.button_append_pressed)
		self._widget.button_overwrite.clicked.connect(self.button_overwrite_pressed)
		self._widget.button_move_up.clicked.connect(self.button_move_up_pressed)
		self._widget.button_move_down.clicked.connect(self.button_move_down_pressed)
		self._widget.button_remove.clicked.connect(self.button_remove_pressed)
		#self._widget.button_cal_esc.clicked.connect(self.button_cal_esc_pressed)
		#self._widget.button_cal_rc.clicked.connect(self.button_cal_rc_pressed)

		# Class Variales

		self.loaded_movement = Movement()
		self.clear_display()

	def shutdown_plugin(self):
		pass

	def save_settings(self, plugin_settings, instance_settings):
		# TODO save intrinsic configuration, usually using:
		# instance_settings.set_value(k, v)
		pass

	def restore_settings(self, plugin_settings, instance_settings):
		# TODO restore intrinsic configuration, usually using:
		# v = instance_settings.value(k)
		pass

	#def trigger_configuration(self):
		# Comment in to signal that the plugin has a way to configure
		# This will enable a setting button (gear icon) in each dock widget title bar
		# Usually used to open a modal configuration dialog

	def button_save_pressed(self):
		if not self.loaded_movement.filename:
			self.button_save_as_pressed()

		if self.loaded_movement.filename:
			self.set_flight_plan()

			yaml_move = self.loaded_movement.encode_yaml()

			with open(self.loaded_movement.filename, 'w') as outfile:
				yaml.dump(yaml_move, outfile,default_flow_style = False)

			rospy.loginfo("Movements saved: %s" % self.loaded_movement.filename)
		else:
			rospy.logerr("No filename specified!")

	def button_save_as_pressed(self):
		(name,filt) = QFileDialog.getSaveFileName(caption='Save Movement',filter="YAML (*.yaml)")
		self.loaded_movement.filename = name

		if self.loaded_movement.filename:
			self.button_save_pressed()
		else:
			rospy.logerr("No filename specified!")

	def button_load_pressed(self):
		(name,filt) = QFileDialog.getOpenFileName(caption='Open Movement')

		self.loaded_movement = Movement()

		if name:
			stream = open(name, 'r')

			try:
				with stream:
					self.loaded_movement = Movement(filename=name, movement=yaml.safe_load(stream))
			except yaml.YAMLError as e:
				rospy.logerror(e)

		self.update_display()

	def update_display(self):
		self.clear_display()

		self.update_flight_plan()

		for i in xrange(len(self.loaded_movement.waypoints)):
			self._widget.list_waypoints.addItem(str(i) + ": " + str(self.loaded_movement.waypoints[i]))

	def clear_display(self):
		self._widget.list_waypoints.clear()

		self._widget.input_x.setText("0.0")
		self._widget.input_y.setText("0.0")
		self._widget.input_z.setText("0.0")
		self._widget.input_psi.setText("0.0")

	def mode_changed(self):
		mode = self._widget.combobox_mode.currentText()

		if mode == 'Continuous':
			self._widget.label_duration.setEnabled(True)
			self._widget.input_duration.setEnabled(True)
			self._widget.label_nom_vel.setEnabled(False)
			self._widget.input_nom_vel.setEnabled(False)
			self._widget.label_nom_rate.setEnabled(False)
			self._widget.input_nom_rate.setEnabled(False)
		else: # Discrete
			self._widget.label_duration.setEnabled(False)
			self._widget.input_duration.setEnabled(False)
			self._widget.label_nom_vel.setEnabled(True)
			self._widget.input_nom_vel.setEnabled(True)
			self._widget.label_nom_rate.setEnabled(True)
			self._widget.input_nom_rate.setEnabled(True)

	def list_item_changed(self):
		wp = self.loaded_movement.waypoints[self._widget.list_waypoints.currentRow()]

		self._widget.input_x.setText(str(wp.x))
		self._widget.input_y.setText(str(wp.y))
		self._widget.input_z.setText(str(wp.z))
		self._widget.input_psi.setText(str(wp.yaw))

	def set_flight_plan(self):
		mode = self._widget.combobox_mode.currentText()

		if mode == 'Discrete':
			self.loaded_movement.is_discrete = True
		else:
			self.loaded_movement.is_discrete = False

		self.loaded_movement.duration = float(self._widget.input_duration.text())
		self.loaded_movement.nom_vel = float(self._widget.input_nom_vel.text())
		self.loaded_movement.nom_rate = float(self._widget.input_nom_rate.text())

	def update_flight_plan(self):
		self._widget.combobox_mode.currentText()

		if self.loaded_movement.is_discrete:
			self._widget.combobox_mode.setCurrentIndex(1)
		else:
			self._widget.combobox_mode.setCurrentIndex(0)

		self._widget.input_duration.setText(str(self.loaded_movement.duration))
		self._widget.input_nom_vel.setText(str(self.loaded_movement.nom_vel))
		self._widget.input_nom_rate.setText(str(self.loaded_movement.nom_rate))

	def prepare_waypoint(self):
		return Waypoint(x=self._widget.input_x.text(),
						y=self._widget.input_y.text(),
						z=self._widget.input_z.text(),
						yaw=self._widget.input_psi.text())

	def button_insert_pressed(self):
		ind = self._widget.list_waypoints.currentRow()
		self.loaded_movement.waypoints.insert(ind, self.prepare_waypoint())
		self.update_display()
		self._widget.list_waypoints.setCurrentRow(ind)

	def button_append_pressed(self):
		ind = self._widget.list_waypoints.currentRow() + 1
		self.loaded_movement.waypoints.insert(ind, self.prepare_waypoint())
		self.update_display()
		self._widget.list_waypoints.setCurrentRow(ind)

	def button_overwrite_pressed(self):
		ind = self._widget.list_waypoints.currentRow()
		if (len(self.loaded_movement.waypoints) > ind) and (ind >= 0):
			self.loaded_movement.waypoints[ind] = self.prepare_waypoint()
			self.update_display()
			self._widget.list_waypoints.setCurrentRow(ind)

	def button_move_up_pressed(self):
		ind = self._widget.list_waypoints.currentRow()
		if (len(self.loaded_movement.waypoints) > ind) and (ind > 0):
			self.loaded_movement.waypoints.insert(ind-1, self.loaded_movement.waypoints.pop(ind))
			self.update_display()
			self._widget.list_waypoints.setCurrentRow(ind-1)

	def button_move_down_pressed(self):
		ind = self._widget.list_waypoints.currentRow()
		if (len(self.loaded_movement.waypoints)-1 > ind) and (ind >= 0):
			self.loaded_movement.waypoints.insert(ind+1, self.loaded_movement.waypoints.pop(ind))
			self.update_display()
			self._widget.list_waypoints.setCurrentRow(ind+1)

	def button_remove_pressed(self):
		ind = self._widget.list_waypoints.currentRow()
		if (len(self.loaded_movement.waypoints) > ind) and (ind >= 0):
			self.loaded_movement.waypoints.pop(ind)
			self.update_display()
			self._widget.list_waypoints.setCurrentRow(ind)






