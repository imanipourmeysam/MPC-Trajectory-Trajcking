import numpy as np
from scipy import interpolate
from scipy.interpolate import CubicSpline

class Spline:
    def __init__(self, ds=0.1):
        self.ds = ds
        self.rx = None
        self.ry = None
        self.ryaw = None

    def calculate_spline(self, x,y):
        dx = np.diff(x)
        dy = np.diff(y)
        ds_array = np.hypot(dx, dy)
        s = np.insert(np.cumsum(ds_array), 0, 0)
        # natural cubic splines: s -> x, s -> y
        cs_x = CubicSpline(s, x, bc_type="natural")
        cs_y = CubicSpline(s, y, bc_type="natural")
        # uniform arc-length samples
        s_uniform = np.arange(0, s[-1], self.ds)
        self.rx = cs_x(s_uniform)
        self.ry = cs_y(s_uniform)
        derx = np.diff(self.rx)
        dery = np.diff(self.ry)
        yaw = np.arctan2(dery, derx)
        # same length as rx,ry (pad last value)
        self.ryaw = np.append(yaw, yaw[-1])
        return (self.rx, self.ry, self.ryaw)