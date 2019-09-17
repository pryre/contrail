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

		# Plot Tabs
		self._widget.tabgroup_plots.currentChanged.connect(self.draw_focused_plot)

		# Management
		self._widget.button_load.clicked.connect(self.button_load_pressed)
		self._widget.button_save.clicked.connect(self.button_save_pressed)
		self._widget.button_save_as.clicked.connect(self.button_save_as_pressed)
		self._widget.button_reset.clicked.connect(self.button_reset_pressed)
		self._widget.button_yaw_smooth.clicked.connect(self.button_yaw_smooth_pressed)

		# Flight Plan
		self._widget.combobox_mode.currentIndexChanged.connect(self.mode_changed)
		self._widget.input_duration.textEdited.connect(self.set_flight_plan_duration)
		self._widget.input_nom_vel.textEdited.connect(self.set_flight_plan_nom_vel)
		self._widget.input_nom_rate.textEdited.connect(self.set_flight_plan_nom_rate)

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
		self.loaded_movement = Movement()
		self.num_interp = 25

		#Create plot figures
		self.plot_3d_figure = Figure()
		self.plot_3d_figure.patch.set_facecolor('white')
		self.plot_3d_canvas = FigureCanvas(self.plot_3d_figure)
		self.plot_3d_toolbar = NavigationToolbar(self.plot_3d_canvas, self._widget.widget_plot_3d)
		self.plot_3d_ax = self.plot_3d_figure.add_subplot(1,1,1, projection='3d')
		self.set_plot_layout(self._widget.widget_plot_3d, self.plot_3d_toolbar, self.plot_3d_canvas)
		self.plot_3d_ax.set_title('3D Projection')
		self.plot_3d_ax.set_xlabel('Position (X)')
		self.plot_3d_ax.set_ylabel('Position (Y)')
		self.plot_3d_ax.set_zlabel('Position (Z)')
		self.plot_3d_ax.set_aspect('equal', 'box')

		self.plot_x_figure = Figure()
		self.plot_x_figure.patch.set_facecolor('white')
		self.plot_x_canvas = FigureCanvas(self.plot_x_figure)
		self.plot_x_toolbar = NavigationToolbar(self.plot_x_canvas, self._widget.widget_plot_x)
		(self.plot_x_ax_pos, self.plot_x_ax_vel, self.plot_x_ax_acc) = self.set_subplots(self.plot_x_figure, 'X', 'm')
		self.set_plot_layout(self._widget.widget_plot_x, self.plot_x_toolbar, self.plot_x_canvas)

		self.plot_y_figure = Figure()
		self.plot_y_figure.patch.set_facecolor('white')
		self.plot_y_canvas = FigureCanvas(self.plot_y_figure)
		self.plot_y_toolbar = NavigationToolbar(self.plot_y_canvas, self._widget.widget_plot_y)
		(self.plot_y_ax_pos, self.plot_y_ax_vel, self.plot_y_ax_acc) = self.set_subplots(self.plot_y_figure, 'Y', 'm')
		self.set_plot_layout(self._widget.widget_plot_y, self.plot_y_toolbar, self.plot_y_canvas)

		self.plot_z_figure = Figure()
		self.plot_z_figure.patch.set_facecolor('white')
		self.plot_z_canvas = FigureCanvas(self.plot_z_figure)
		self.plot_z_toolbar = NavigationToolbar(self.plot_z_canvas, self._widget.widget_plot_z)
		(self.plot_z_ax_pos, self.plot_z_ax_vel, self.plot_z_ax_acc) = self.set_subplots(self.plot_z_figure, 'Z', 'm')
		self.set_plot_layout(self._widget.widget_plot_z, self.plot_z_toolbar, self.plot_z_canvas)

		self.plot_psi_figure = Figure()
		self.plot_psi_figure.patch.set_facecolor('white')
		self.plot_psi_canvas = FigureCanvas(self.plot_psi_figure)
		self.plot_psi_toolbar = NavigationToolbar(self.plot_psi_canvas, self._widget.widget_plot_psi)
		(self.plot_psi_ax_pos, self.plot_psi_ax_vel, self.plot_psi_ax_acc) = self.set_subplots(self.plot_psi_figure, 'Yaw', 'rad')
		self.set_plot_layout(self._widget.widget_plot_psi, self.plot_psi_toolbar, self.plot_psi_canvas)

		self.reset_flight_plan()

	def set_subplots(self, figure, title, unit):
		ax_pos = figure.add_subplot(3,1,1)
		ax_vel = figure.add_subplot(3,1,2)
		ax_acc = figure.add_subplot(3,1,3)

		ax_pos.grid(b=True)
		ax_vel.grid(b=True)
		ax_acc.grid(b=True)

		ax_pos.set_title(title + ' Axis')
		ax_pos.set_ylabel('Pos. (' + unit + ')')
		ax_vel.set_ylabel('Vel. (' + unit + '/s)')
		ax_acc.set_ylabel('Acc. (' + unit + '/s/s)')
		ax_acc.set_xlabel('Time (s)')

		return (ax_pos, ax_vel, ax_acc)

	def set_plot_layout(self, widget, toolbar, canvas):
		layout = QVBoxLayout()
		layout.addWidget(toolbar)
		layout.addWidget(canvas)
		widget.setLayout(layout)

	def shutdown_plugin(self):
		pass

	def save_settings(self, plugin_settings, instance_settings):
		instance_settings.set_value("num_interp", self.num_interp)

	def restore_settings(self, plugin_settings, instance_settings):
		try:
			self.num_interp = int( instance_settings.value("num_interp") )
		except (AttributeError,TypeError) as e:
			self.num_interp = 25

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


	def make_yaw_continuous(self, psi_c, psi_p):
		while math.fabs(psi_c - psi_p) > math.pi:
			if psi_c > psi_p:
				psi_c -= 2*math.pi
			else:
				psi_c += 2*math.pi

		return psi_c

	def button_yaw_smooth_pressed(self):
		for i in xrange(len(self.loaded_movement.waypoints)):
			if i > 1:
				self.loaded_movement.waypoints[i].yaw = self.make_yaw_continuous(self.loaded_movement.waypoints[i].yaw,self.loaded_movement.waypoints[i-1].yaw)

		self.update_display()

	def reset_flight_plan(self):
		self.plot_3d_ax.view_init(azim=-135)

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

	def quiver_dir_from_yaw(self,psi):
		n = len(psi)
		u = [0.0]*n
		v = [0.0]*n
		w = [0.0]*n

		for i in xrange(n):
			u[i] = math.cos(psi[i])
			v[i] = math.sin(psi[i])
			#w[i] = 0.0
		return (u,v,w)

	def update_plot(self):
		print("plot update")
		self.clear_plot()

		if len(self.loaded_movement.waypoints):
			plot_data = [[],[],[],[]]
			for i in xrange(len(self.loaded_movement.waypoints)):
				plot_data[0].append(self.loaded_movement.waypoints[i].x)
				plot_data[1].append(self.loaded_movement.waypoints[i].y)
				plot_data[2].append(self.loaded_movement.waypoints[i].z)
				plot_data[3].append(self.loaded_movement.waypoints[i].yaw)

			d = float(self.loaded_movement.duration)
			d2 = d*d

			# Plot data
			n = len(plot_data[0])
			ni = n
			t = [0.0]*n
			x = plot_data[0]
			xd = [0.0]*n
			xdd = [0.0]*n
			y = plot_data[1]
			yd = [0.0]*n
			ydd = [0.0]*n
			z = plot_data[2]
			zd = [0.0]*n
			zdd = [0.0]*n
			psi = plot_data[3]
			psid = [0.0]*n
			psidd = [0.0]*n
			ti = [0.0]*ni
			xi = x
			xdi = [0.0]*ni
			xddi = [0.0]*ni
			yi = y
			ydi = [0.0]*ni
			yddi = [0.0]*ni
			zi = z
			zdi = [0.0]*ni
			zddi = [0.0]*ni
			psii = psi
			psidi = [0.0]*ni
			psiddi = [0.0]*ni

			if (n > 1) and (d > 0.0) and not self.loaded_movement.is_discrete:
				dt = d / (n-1)
				t = [dt * i for i in range(n)]

				# Plot spline display if in contiuous mode
				iqs_x = InterpolatedQuinticSpline()
				iqs_y = InterpolatedQuinticSpline()
				iqs_z = InterpolatedQuinticSpline()
				iqs_psi = InterpolatedQuinticSpline()

				if iqs_x.interpolate(x) and iqs_y.interpolate(y) and iqs_z.interpolate(z) and iqs_psi.interpolate(psi):
					xd = [i/d for i in iqs_x.get_dvias()]
					xdd = [i/d2 for i in iqs_x.get_ddvias()]
					yd = [i/d for i in iqs_y.get_dvias()]
					ydd = [i/d2 for i in iqs_y.get_ddvias()]
					zd = [i/d for i in iqs_z.get_dvias()]
					zdd = [i/d2 for i in iqs_z.get_ddvias()]
					psid = [i/d for i in iqs_psi.get_dvias()]
					psidd = [i/d2 for i in iqs_psi.get_ddvias()]

					ni = n*self.num_interp
					ui = [float(i) / (ni-1) for i in range(ni)]
					xvi = [iqs_x.lookup(u) for u in ui]
					yvi = [iqs_y.lookup(u) for u in ui]
					zvi = [iqs_z.lookup(u) for u in ui]
					psivi = [iqs_psi.lookup(u) for u in ui]

					xi = [i[0] for i in xvi]
					xdi = [i[1]/d for i in xvi]
					xddi = [i[2]/d2 for i in xvi]

					yi = [i[0] for i in yvi]
					ydi = [i[1]/d for i in yvi]
					yddi = [i[2]/d2 for i in yvi]

					zi = [i[0] for i in zvi]
					zdi = [i[1]/d for i in zvi]
					zddi = [i[2]/d2 for i in zvi]

					psii = [i[0] for i in psivi]
					psidi = [i[1]/d for i in psivi]
					psiddi = [i[2]/d2 for i in psivi]

					dti = d / (ni-1)
					ti = [dti * i for i in range(ni)]
				else:
					rospy.logerror("Could not interpolate spline!")

				self.plot_3d_ax.plot(x, y, z, 'b--')
				self.plot_3d_ax.plot(xi, yi, zi, 'g-')
				self.plot_x_ax_pos.plot(ti, xi, 'g-')
				self.plot_x_ax_vel.plot(ti, xdi, 'g-')
				self.plot_x_ax_acc.plot(ti, xddi, 'g-')
				self.plot_y_ax_pos.plot(ti, yi, 'g-')
				self.plot_y_ax_vel.plot(ti, ydi, 'g-')
				self.plot_y_ax_acc.plot(ti, yddi, 'g-')
				self.plot_z_ax_pos.plot(ti, zi, 'g-')
				self.plot_z_ax_vel.plot(ti, zdi, 'g-')
				self.plot_z_ax_acc.plot(ti, zddi, 'g-')
				self.plot_psi_ax_pos.plot(ti, psii, 'g-')
				self.plot_psi_ax_vel.plot(ti, psidi, 'g-')
				self.plot_psi_ax_acc.plot(ti, psiddi, 'g-')
			# Visual waypoint data
			elif self.loaded_movement.is_discrete:
				# Generate fake t data for some form of display
				t = xrange(len(x))
				self.plot_3d_ax.plot(x, y, z, 'g-')
				self.plot_x_ax_pos.plot(t, x, 'g-')
				self.plot_x_ax_vel.plot(t, xd, 'g-')
				self.plot_x_ax_acc.plot(t, xdd, 'g-')
				self.plot_y_ax_pos.plot(t, y, 'g-')
				self.plot_y_ax_vel.plot(t, yd, 'g-')
				self.plot_y_ax_acc.plot(t, ydd, 'g-')
				self.plot_z_ax_pos.plot(t, z, 'g-')
				self.plot_z_ax_vel.plot(t, zd, 'g-')
				self.plot_z_ax_acc.plot(t, zdd, 'g-')
				self.plot_psi_ax_pos.plot(t, psi, 'g-')
				self.plot_psi_ax_vel.plot(t, psid, 'g-')
				self.plot_psi_ax_acc.plot(t, psidd, 'g-')

			self.plot_3d_ax.plot(x, y, z, 'bo')
			(qu,qv,qw) = self.quiver_dir_from_yaw(psi)
			self.plot_3d_ax.quiver(x, y, z, qu, qv, qw, length=0.25, pivot='tail')#, cmap='Reds', lw=2)
			self.plot_x_ax_pos.plot(t, x, 'bo')
			self.plot_x_ax_vel.plot(t, xd, 'bo')
			self.plot_x_ax_acc.plot(t, xdd, 'bo')
			self.plot_y_ax_pos.plot(t, y, 'bo')
			self.plot_y_ax_vel.plot(t, yd, 'bo')
			self.plot_y_ax_acc.plot(t, ydd, 'bo')
			self.plot_z_ax_pos.plot(t, z, 'bo')
			self.plot_z_ax_vel.plot(t, zd, 'bo')
			self.plot_z_ax_acc.plot(t, zdd, 'bo')
			self.plot_psi_ax_pos.plot(t, psi, 'bo')
			self.plot_psi_ax_vel.plot(t, psid, 'bo')
			self.plot_psi_ax_acc.plot(t, psidd, 'bo')

			sel_ind = self._widget.table_waypoints.currentRow()
			if sel_ind >= 0:
				self.plot_3d_ax.plot([x[sel_ind]], [y[sel_ind]], [z[sel_ind]], 'ro')
				self.plot_3d_ax.quiver([x[sel_ind]], [y[sel_ind]], [z[sel_ind]], [qu[sel_ind]], [qv[sel_ind]], [qw[sel_ind]], length=0.25, pivot='tail', colors=[[1.0,0.0,0.0]], lw=3)

				self.plot_x_ax_pos.plot([t[sel_ind]], [x[sel_ind]], 'ro')
				self.plot_x_ax_vel.plot([t[sel_ind]], [xd[sel_ind]], 'ro')
				self.plot_x_ax_acc.plot([t[sel_ind]], [xdd[sel_ind]], 'ro')
				self.plot_y_ax_pos.plot([t[sel_ind]], [y[sel_ind]], 'ro')
				self.plot_y_ax_vel.plot([t[sel_ind]], [yd[sel_ind]], 'ro')
				self.plot_y_ax_acc.plot([t[sel_ind]], [ydd[sel_ind]], 'ro')
				self.plot_z_ax_pos.plot([t[sel_ind]], [z[sel_ind]], 'ro')
				self.plot_z_ax_vel.plot([t[sel_ind]], [zd[sel_ind]], 'ro')
				self.plot_z_ax_acc.plot([t[sel_ind]], [zdd[sel_ind]], 'ro')
				self.plot_psi_ax_pos.plot([t[sel_ind]], [psi[sel_ind]], 'ro')
				self.plot_psi_ax_vel.plot([t[sel_ind]], [psid[sel_ind]], 'ro')
				self.plot_psi_ax_acc.plot([t[sel_ind]], [psidd[sel_ind]], 'ro')


			# Calculate nice limits for 3D
			minx = min([min(x),min(xi)])
			miny = min([min(y),min(yi)])
			minz = min([min(z),min(zi)])
			maxx = max([max(x),max(xi)])
			maxy = max([max(y),max(yi)])
			maxz = max([max(z),max(zi)])

			max_range = 1.25*max([maxx-minx, maxy-miny, maxz-minz]) / 2.0

			mid_x = (maxx+minx) / 2.0
			mid_y = (maxy+miny) / 2.0
			mid_z = (maxz+minz) / 2.0

			if max_range:
				self.plot_3d_ax.set_xlim(mid_x - max_range, mid_x + max_range)
				self.plot_3d_ax.set_ylim(mid_y - max_range, mid_y + max_range)
				self.plot_3d_ax.set_zlim(mid_z - max_range, mid_z + max_range)

			self.set_nice_limits_2d(self.plot_x_ax_pos, 0, t[-1], min([min(x),min(xi)]), max([max(x),max(xi)]))
			self.set_nice_limits_2d(self.plot_x_ax_vel, 0, t[-1], min([min(xd),min(xdi)]), max([max(xd),max(xdi)]))
			self.set_nice_limits_2d(self.plot_x_ax_acc, 0, t[-1], min([min(xdd),min(xddi)]), max([max(xdd),max(xddi)]))
			self.set_nice_limits_2d(self.plot_y_ax_pos, 0, t[-1], min([min(y),min(yi)]), max([max(y),max(yi)]))
			self.set_nice_limits_2d(self.plot_y_ax_vel, 0, t[-1], min([min(yd),min(ydi)]), max([max(yd),max(ydi)]))
			self.set_nice_limits_2d(self.plot_y_ax_acc, 0, t[-1], min([min(ydd),min(yddi)]), max([max(ydd),max(yddi)]))
			self.set_nice_limits_2d(self.plot_z_ax_pos, 0, t[-1], min([min(z),min(zi)]), max([max(z),max(zi)]))
			self.set_nice_limits_2d(self.plot_z_ax_vel, 0, t[-1], min([min(zd),min(zdi)]), max([max(zd),max(zdi)]))
			self.set_nice_limits_2d(self.plot_z_ax_acc, 0, t[-1], min([min(zdd),min(zddi)]), max([max(zdd),max(zddi)]))
			self.set_nice_limits_2d(self.plot_psi_ax_pos, 0, t[-1], min([min(psi),min(psii)]), max([max(psi),max(psii)]))
			self.set_nice_limits_2d(self.plot_psi_ax_vel, 0, t[-1], min([min(psid),min(psidi)]), max([max(psid),max(psidi)]))
			self.set_nice_limits_2d(self.plot_psi_ax_acc, 0, t[-1], min([min(psidd),min(psiddi)]), max([max(psidd),max(psiddi)]))

		# Refresh current canvas
		self.draw_focused_plot(self._widget.tabgroup_plots.currentIndex())

	def set_nice_limits_2d(self, ax, x_min, x_max, y_min, y_max):
		mr = 1.25*(y_max-y_min) / 2.0
		y_mid = (y_max+y_min) / 2.0
		ax.set_xlim(x_min, x_max)
		ax.set_ylim(y_mid - mr, y_mid + mr)

	def draw_focused_plot(self,index):
		if(index == 0):
			self.plot_3d_canvas.draw()
		elif(index == 1):
			self.plot_x_ax_pos.relim()
			self.plot_x_ax_vel.relim()
			self.plot_x_ax_acc.relim()
			self.plot_x_canvas.draw()
		elif(index == 2):
			self.plot_y_ax_pos.relim()
			self.plot_y_ax_vel.relim()
			self.plot_y_ax_acc.relim()
			self.plot_y_canvas.draw()
		elif(index == 3):
			self.plot_z_ax_pos.relim()
			self.plot_z_ax_vel.relim()
			self.plot_z_ax_acc.relim()
			self.plot_z_canvas.draw()
		elif(index == 4):
			self.plot_psi_ax_pos.relim()
			self.plot_psi_ax_vel.relim()
			self.plot_psi_ax_acc.relim()
			self.plot_psi_canvas.draw()

	def clear_plot(self):
		# Discards the old graph data
		artists = self.plot_3d_ax.lines + self.plot_3d_ax.collections \
				+ self.plot_x_ax_pos.lines + self.plot_x_ax_pos.collections \
				+ self.plot_x_ax_vel.lines + self.plot_x_ax_vel.collections \
				+ self.plot_x_ax_acc.lines + self.plot_x_ax_acc.collections \
				+ self.plot_y_ax_pos.lines + self.plot_y_ax_pos.collections \
				+ self.plot_y_ax_vel.lines + self.plot_y_ax_vel.collections \
				+ self.plot_y_ax_acc.lines + self.plot_y_ax_acc.collections \
				+ self.plot_z_ax_pos.lines + self.plot_z_ax_pos.collections \
				+ self.plot_z_ax_vel.lines + self.plot_z_ax_vel.collections \
				+ self.plot_z_ax_acc.lines + self.plot_z_ax_acc.collections \
				+ self.plot_psi_ax_pos.lines + self.plot_psi_ax_pos.collections \
				+ self.plot_psi_ax_vel.lines + self.plot_psi_ax_vel.collections \
				+ self.plot_psi_ax_acc.lines + self.plot_psi_ax_acc.collections

		for artist in artists:
			artist.remove()

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

	def set_flight_plan_duration(self, text):
		self.loaded_movement.duration = float(text)
		self.update_plot()

	def set_flight_plan_nom_vel(self, text):
		self.loaded_movement.nom_vel = float(text)
		self.update_plot()

	def set_flight_plan_nom_rate(self, text):
		self.loaded_movement.nom_rate = float(text)
		self.update_plot()

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
