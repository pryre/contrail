#!/usr/bin/env python2

from contrail_spline_lib._quintic_spline_solver_wrapper_cpp import QuinticSplineSolverWrapper
from contrail_spline_lib._interpolated_quintic_spline_wrapper_cpp import InterpolatedQuinticSplineWrapper


class QuinticSplineSolver(object):
	def __init__(self):
		self._qss = QuinticSplineSolverWrapper()

	def solver(self, q0, qd0, qdd0, qf, qdf, qddf):
		return self._qss.solver(q0, qd0, qdd0, qf, qdf, qddf)

	def linear_derivative_est(self, vias, dt):
		return self._qss.linear_derivative_est(vias,dt)

	def lookup(self, u, c):
		point = []
		if len(c) == 6:
			point = self._qss.lookup(u, c)
		else:
			raise IndexError('List of coefficients must have length 6')
		return point

class InterpolatedQuinticSpline(object):
	def __init__(self):
		self._iqs = InterpolatedQuinticSplineWrapper()

	def interpolate(self, vias):
		return self._iqs.interpolate(vias)

	def get_vias(self):
		return self._iqs.get_vias()

	def get_dvias(self):
		return self._iqs.get_dvias()

	def get_ddvias(self):
		return self._iqs.get_ddvias()

	def lookup(self, u):
		return self._iqs.lookup(u)
