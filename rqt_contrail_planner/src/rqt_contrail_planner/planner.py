import os
import math
import rospkg
import rospy
import yaml

# Qt ROS binding for GUI
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore
from python_qt_binding.QtWidgets import QWidget, QFileDialog, QVBoxLayout, QTableWidgetItem

# MatPlotLib for display backend
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

# Contrail assets
from rqt_contrail_planner.movement import Movement
from rqt_contrail_planner.waypoint import Waypoint

from contrail_spline_lib._quintic_spline_wrapper_py import InterpolatedQuinticSpline

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
		self._widget.button_reset.clicked.connect(self.button_reset_pressed)

		# Flight Plan
		self._widget.combobox_mode.currentIndexChanged.connect(self.mode_changed)

		# Waypoint
		self._widget.table_waypoints.currentItemChanged.connect(self.list_item_changed)
		#self._widget.table_waypoints.itemChanged.connect(self.waypoint_item_changed)
		self._widget.input_x.returnPressed.connect(self.button_overwrite_pressed)
		self._widget.input_y.returnPressed.connect(self.button_overwrite_pressed)
		self._widget.input_z.returnPressed.connect(self.button_overwrite_pressed)
		self._widget.input_psi.returnPressed.connect(self.button_overwrite_pressed)

		self._widget.button_insert.clicked.connect(self.button_insert_pressed)
		self._widget.button_append.clicked.connect(self.button_append_pressed)
		self._widget.button_overwrite.clicked.connect(self.button_overwrite_pressed)
		self._widget.button_move_up.clicked.connect(self.button_move_up_pressed)
		self._widget.button_move_down.clicked.connect(self.button_move_down_pressed)
		self._widget.button_remove.clicked.connect(self.button_remove_pressed)

		# Class Variales
		#self.loaded_movement = Movement()
		#self.num_interp = 10

		# this is the Navigation widget
		# it takes the Canvas widget and a parent
		self.plot_figure = Figure()
		self.plot_figure.patch.set_facecolor('white')
		self.plot_canvas = FigureCanvas(self.plot_figure)
		self.plot_toolbar = NavigationToolbar(self.plot_canvas, self._widget.widget_plot)
		self.plot_ax = self.plot_figure.add_subplot(111, projection='3d')

		plot_layout = QVBoxLayout()
		plot_layout.addWidget(self.plot_toolbar)
		plot_layout.addWidget(self.plot_canvas)
		self._widget.widget_plot.setLayout(plot_layout)

		self.reset_flight_plan()

	def shutdown_plugin(self):
		pass

	def save_settings(self, plugin_settings, instance_settings):
		instance_settings.set_value("num_interp", self.num_interp)

	def restore_settings(self, plugin_settings, instance_settings):
		if instance_settings.value("num_interp") is not None:
			self.num_interp = int( instance_settings.value("num_interp") )

	#def trigger_configuration(self):
		#TODO: allow user to set 'self.num_interp'

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
			rospy.logerr("Can't save, no filename set")

	def button_save_as_pressed(self):
		(name,filt) = QFileDialog.getSaveFileName(caption='Save Movement',filter="YAML (*.yaml)")
		self.loaded_movement.filename = name

		if self.loaded_movement.filename:
			self.button_save_pressed()
		else:
			rospy.logerr("No filename specified")

	def button_load_pressed(self):
		(name,filt) = QFileDialog.getOpenFileName(caption='Open Movement')

		if name:
			stream = open(name, 'r')

			try:
				with stream:
					self.loaded_movement = Movement(filename=name, movement=yaml.safe_load(stream))
			except yaml.YAMLError as e:
				rospy.logerror(e)

		self.update_display()

	def button_reset_pressed(self):
		self.reset_flight_plan()

	def reset_flight_plan(self):
		self.plot_ax.view_init(azim=-135)

		self.loaded_movement = Movement()

		self.update_display()

	def update_display(self):
		self.clear_display()

		self.update_flight_plan()

		for i in xrange(len(self.loaded_movement.waypoints)):
			self._widget.table_waypoints.insertRow(i)
			item_x = QTableWidgetItem(str(self.loaded_movement.waypoints[i].x))
			item_y = QTableWidgetItem(str(self.loaded_movement.waypoints[i].y))
			item_z = QTableWidgetItem(str(self.loaded_movement.waypoints[i].z))
			item_yaw = QTableWidgetItem(str(self.loaded_movement.waypoints[i].yaw))
			item_x.setTextAlignment(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignRight)
			item_y.setTextAlignment(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignRight)
			item_z.setTextAlignment(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignRight)
			item_yaw.setTextAlignment(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignRight)
			self._widget.table_waypoints.setItem(i, 0, item_x)
			self._widget.table_waypoints.setItem(i, 1, item_y)
			self._widget.table_waypoints.setItem(i, 2, item_z)
			self._widget.table_waypoints.setItem(i, 3, item_yaw)

		self.update_plot()

	def update_plot(self):
		self.clear_plot()

		if len(self.loaded_movement.waypoints):
			plot_data = [[],[],[]]
			for i in xrange(len(self.loaded_movement.waypoints)):
				plot_data[0].append(self.loaded_movement.waypoints[i].x)
				plot_data[1].append(self.loaded_movement.waypoints[i].y)
				plot_data[2].append(self.loaded_movement.waypoints[i].z)

			# Plot data
			x = plot_data[0]
			y = plot_data[1]
			z = plot_data[2]
			n = len(x)
			xi = x
			yi = y
			zi = z
			ni = n

			# Waypoint data
			if not self.loaded_movement.is_discrete and (n > 1):
				self.plot_ax.plot(x, y, z, 'b--')

				# Plot spline display if in contiuous mode
				iqs_x = InterpolatedQuinticSpline()
				iqs_y = InterpolatedQuinticSpline()
				iqs_z = InterpolatedQuinticSpline()

				if iqs_x.interpolate(x) and iqs_y.interpolate(y) and iqs_z.interpolate(z):
					ni = n*self.num_interp
					ui = [float(i) / (ni-1) for i in range(ni)]
					xi = [iqs_x.lookup(u)[0] for u in ui]
					yi = [iqs_y.lookup(u)[0] for u in ui]
					zi = [iqs_z.lookup(u)[0] for u in ui]

					self.plot_ax.plot(xi, yi, zi, 'g-')
				else:
					rospy.logerror("Could not interpolate spline!")
			else:
				self.plot_ax.plot(x, y, z, 'g-')

			self.plot_ax.plot(x, y, z, 'bo')

			sel_ind = self._widget.table_waypoints.currentRow()
			if sel_ind >= 0:
				self.plot_ax.plot([x[sel_ind]], [y[sel_ind]], [z[sel_ind]], 'ro')

			# Calculate nice limits
			minx = min([min(x),min(xi)])
			miny = min([min(y),min(yi)])
			minz = min([min(z),min(zi)])
			maxx = max([max(x),max(xi)])
			maxy = max([max(y),max(yi)])
			maxz = max([max(z),max(zi)])

			max_range = max([maxx-minx, maxy-miny, maxz-minz]) / 2.0

			mid_x = (maxx+minx) / 2.0
			mid_y = (maxy+miny) / 2.0
			mid_z = (maxz+minz) / 2.0
			if max_range:
				self.plot_ax.set_xlim(mid_x - max_range, mid_x + max_range)
				self.plot_ax.set_ylim(mid_y - max_range, mid_y + max_range)
				self.plot_ax.set_zlim(mid_z - max_range, mid_z + max_range)

		#self.plot_ax.axis('equal')
		self.plot_ax.set_xlabel('X (m)')
		self.plot_ax.set_ylabel('Y (m)')
		self.plot_ax.set_zlabel('Z (m)')

		# Refresh canvas
		self.plot_canvas.draw()

	def clear_plot(self):
		# Discards the old graph
		self.plot_ax.clear()

	def clear_inputs(self):
		self._widget.input_x.setText("0.0")
		self._widget.input_y.setText("0.0")
		self._widget.input_z.setText("0.0")
		self._widget.input_psi.setText("0.0")

	def clear_display(self):
		self._widget.table_waypoints.setRowCount(0)
		self.clear_inputs()
		self.clear_plot()

	def mode_changed(self):
		mode = self._widget.combobox_mode.currentText()

		if mode == 'Continuous':
			self.loaded_movement.is_discrete = False

			self._widget.label_duration.setEnabled(True)
			self._widget.input_duration.setEnabled(True)
			self._widget.label_nom_vel.setEnabled(False)
			self._widget.input_nom_vel.setEnabled(False)
			self._widget.label_nom_rate.setEnabled(False)
			self._widget.input_nom_rate.setEnabled(False)
		else: # Discrete
			self.loaded_movement.is_discrete = True

			self._widget.label_duration.setEnabled(False)
			self._widget.input_duration.setEnabled(False)
			self._widget.label_nom_vel.setEnabled(True)
			self._widget.input_nom_vel.setEnabled(True)
			self._widget.label_nom_rate.setEnabled(True)
			self._widget.input_nom_rate.setEnabled(True)

		self.update_plot()

	#def waypoint_item_changed(self, item):
	#	if item:
	#		ind = item.row()
	#		dind = item.column()
	#		data = float(item.text())
	#		if dind == 0:
	#			self.loaded_movement.waypoints[ind].x = data
	#		elif dind == 1:
	#			self.loaded_movement.waypoints[ind].y = data
	#		elif dind == 2:
	#			self.loaded_movement.waypoints[ind].z = data
	#		elif dind == 3:
	#			self.loaded_movement.waypoints[ind].yaw = data
	#
	#		self.list_item_changed()
	#		self.update_plot()

	def list_item_changed(self):
		ind = self._widget.table_waypoints.currentRow()
		if ind >= 0:
			wp = self.loaded_movement.waypoints[ind]

			self._widget.input_x.setText(str(wp.x))
			self._widget.input_y.setText(str(wp.y))
			self._widget.input_z.setText(str(wp.z))
			self._widget.input_psi.setText(str(wp.yaw))
		else:
			self.clear_inputs()

		self.update_plot()

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
		ind = self._widget.table_waypoints.currentRow()
		self.loaded_movement.waypoints.insert(ind, self.prepare_waypoint())
		self.update_display()
		self._widget.table_waypoints.selectRow(ind)

	def button_append_pressed(self):
		ind = self._widget.table_waypoints.currentRow() + 1
		self.loaded_movement.waypoints.insert(ind, self.prepare_waypoint())
		self.update_display()
		self._widget.table_waypoints.selectRow(ind)

	def button_overwrite_pressed(self):
		ind = self._widget.table_waypoints.currentRow()
		if (len(self.loaded_movement.waypoints) > ind) and (ind >= 0):
			self.loaded_movement.waypoints[ind] = self.prepare_waypoint()
			self.update_display()
			self._widget.table_waypoints.selectRow(ind)

	def button_move_up_pressed(self):
		ind = self._widget.table_waypoints.currentRow()
		if (len(self.loaded_movement.waypoints) > ind) and (ind > 0):
			self.loaded_movement.waypoints.insert(ind-1, self.loaded_movement.waypoints.pop(ind))
			self.update_display()
			self._widget.table_waypoints.selectRow(ind-1)

	def button_move_down_pressed(self):
		ind = self._widget.table_waypoints.currentRow()
		if (len(self.loaded_movement.waypoints)-1 > ind) and (ind >= 0):
			self.loaded_movement.waypoints.insert(ind+1, self.loaded_movement.waypoints.pop(ind))
			self.update_display()
			self._widget.table_waypoints.selectRow(ind+1)

	def button_remove_pressed(self):
		ind = self._widget.table_waypoints.currentRow()
		if (len(self.loaded_movement.waypoints) > ind) and (ind >= 0):
			self.loaded_movement.waypoints.pop(ind)
			self.update_display()
			self._widget.table_waypoints.selectRow(ind-1)
