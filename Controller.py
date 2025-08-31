import numpy as np
import cvxpy as cp
import math
from scipy import interpolate
from scipy.interpolate import CubicSpline


class Controller:
    def __init__(self):
        self.dt = 0.1 # Sampling time.
        self.N_horizon = 15 # MPC horizon
        self.Q = np.diag([10, 10, 1]) # State tracking
        self.R = np.diag([0.01, 0.01]) # Input effort
        self.Rd = np.diag([1, 1]) # Input rate
        self.A = np.eye(3)
        self.B = np.zeros((3,2))
        self.computed_trajectory = None

    def wrap_angle(self,angle):
        return (angle + np.pi) % (2*np.pi) - np.pi     

    def get_model_matrices(self, v_ref, yaw_ref):
        
        self.A[0,2] = -v_ref * np.sin(yaw_ref) * self.dt
        self.A[1,2] =  v_ref * np.cos(yaw_ref) * self.dt

        self.B[0,0] = np.cos(yaw_ref) * self.dt
        self.B[1,0] = np.sin(yaw_ref) * self.dt
        self.B[2,1] = self.dt
        return self.A, self.B


    def build_ref_inputs(self, trajectory_ref):
        rx, ry, ryaw = trajectory_ref
        Nref = trajectory_ref.shape[1]
        v_r = np.zeros(Nref-1)
        yaw_r = np.zeros(Nref-1)
        for k in range(Nref-1):
            ds = np.hypot(rx[k+1]-rx[k], ry[k+1]-ry[k])
            v_r[k] = ds / self.dt
            dth = (ryaw[k+1] - ryaw[k])
            yaw_r[k] = dth / self.dt
        return v_r, yaw_r

    def mpc_control(self, x0, trajectory_ref):
        N = min(self.N_horizon, trajectory_ref.shape[1]-1)  # adjust horizon if near end
        v_r, yaw_r = self.build_ref_inputs(trajectory_ref)
        x = cp.Variable((3, N+1))
        u = cp.Variable((2, N))

        cost = 0
        constraints = [x[:,0] == x0]

        for k in range(N):
            # reference at step k
            trajectory_ref_k = trajectory_ref[:, k]
            trajectory_ref_k1= trajectory_ref[:,k+1]
            u_ref_k = np.array([v_r[k], yaw_r[k]])

            self.A, self.B = self.get_model_matrices(v_r[k], yaw_r[k])

            # cost function
            e_theta = (x[2,k]-trajectory_ref_k[2])
            e_xy = x[0:2,k] - trajectory_ref_k[0:2]
            cost += cp.quad_form(cp.hstack([e_xy, e_theta]), self.Q)
            cost += cp.quad_form(u[:,k]-u_ref_k, self.R)
            if k > 0:
                cost += cp.quad_form(u[:,k] - u[:,k-1], self.Rd)

            # dynamics constraint
            constraints += [x[:,k+1]-trajectory_ref_k1 == self.A @ (x[:,k]-trajectory_ref_k) + self.B @ (u[:,k]-u_ref_k)]
            #constraints += [x[:,k+1] == self.A @ x[:,k] + self.B @ u[:,k]]
        
        #constraints += [
        #u[0, :] <= 1.0,
        #u[0, :] >= -0.5,
        #u[1, :] <= 1.0,
        #u[1, :] >= -1.0]

        # Solve MPC
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.ECOS)

        # Return first control input
        return u[:,0].value

    def compute_trajectory(self, trajectory_ref):
        state = trajectory_ref[:,0]
        compute_trajectory_ = [state.copy()]

        for t in range(len(trajectory_ref[0]) - 1):
            # wrap reference yaw for stability
            #trajectory_ref[2, :] = self.wrap_angle(trajectory_ref[2, :])

            u = self.mpc_control(state, trajectory_ref[:,t:])
            v, omega = u[0], u[1]
            # update robot state
            state[0] += v * np.cos(state[2]) * self.dt
            state[1] += v * np.sin(state[2]) * self.dt
            state[2] += omega * self.dt
            state[2] = self.wrap_angle(state[2]) 
            compute_trajectory_.append(state.copy())
        compute_trajectory_ = np.array(compute_trajectory_)
        self.computed_trajectory = (compute_trajectory_[:,0], compute_trajectory_[:,1], compute_trajectory_[:,2])
        return self.computed_trajectory
