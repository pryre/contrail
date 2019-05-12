from contrail_spline_lib._quintic_spline_solver_wrapper_cpp import QuinticSplineSolverWrapper


class QuinticSplineSolver(object):
    def __init__(self):
        self._quintic_spline_solver = QuinticSplineSolverWrapper()

    def add(self, l):
        self._quintic_spline_solver.add(l)
        #return val
