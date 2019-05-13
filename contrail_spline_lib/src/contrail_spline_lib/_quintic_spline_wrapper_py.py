from contrail_spline_lib._quintic_spline_solver_wrapper_cpp import QuinticSplineSolverWrapper
from contrail_spline_lib._interpolated_quintic_spline_wrapper_cpp import InterpolatedQuinticSplineWrapper


class QuinticSplineSolver(object):
    def __init__(self):
        self._quintic_spline_solver = QuinticSplineSolverWrapper()

    def add(self, l):
        self._quintic_spline.add(l)
        #return val

class InterpolatedQuinticSpline(object):
    def __init__(self):
        self._interpolated_quintic_spline = InterpolatedQuinticSplineWrapper()

    def add(self, l):
        #self._quintic_spline.add(l)
        #return val
		pass
