import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation


class RobotSim:
    def __init__(self, fig, ax, trajectory):
        
        #trajectory
        self.trajectory = trajectory

        # figure , ax
        self.fig = fig
        self.ax = ax
        #self.ax.set_xlim(0,10)
        #self.ax.set_ylim(0,10)
        #self.ax.set_aspect('equal', 'box')
        #self.ax.grid(True)
        #self.ax.legend()

        # robot
        self.robot_body, = self.ax.plot([], [], 'b-')   # rectangle
        self.heading, = ax.plot([], [], 'g-')      # heading arrow
        self.path, = ax.plot([], [], 'b.', alpha=0.3)  # traced path
        self.robot_length = 0.5
        self.robot_width = 0.3

        # animation 
        self.ani = animation.FuncAnimation(
            self.fig,
            self.update,
            frames=len(self.trajectory[0]),
            interval=100,
            blit=True,
            repeat=False
        )


    def update(self, frame):
        x = self.trajectory[0][frame]
        y = self.trajectory[1][frame]
        yaw = self.trajectory[2][frame]

        corners = np.array([
            [ self.robot_length/2,  self.robot_width/2],
            [ self.robot_length/2, -self.robot_width/2],
            [-self.robot_length/2, -self.robot_width/2],
            [-self.robot_length/2,  self.robot_width/2],
            [ self.robot_length/2,  self.robot_width/2]
        ])
        R = np.array([[np.cos(yaw), -np.sin(yaw)],
                      [np.sin(yaw),  np.cos(yaw)]])
        world_corners = (R @ corners.T).T + np.array([x, y])
        self.robot_body.set_data(world_corners[:,0], world_corners[:,1])

        arrow = np.array([[0,0], [self.robot_length,0]])
        arrow_world = (R @ arrow.T).T + np.array([x,y])
        self.heading.set_data(arrow_world[:,0], arrow_world[:,1])

        self.path.set_data(self.trajectory[0][:frame], self.trajectory[1][:frame])
        return self.robot_body, self.heading, self.path
