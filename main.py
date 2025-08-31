from matplotlib import pyplot as plt
from InterFace import InterFace
from Spline import Spline
from Controller import Controller
from TrajectoryDB import TrajectoryDB
from RobotSim import RobotSim
import numpy as np

fig, ax = plt.subplots()
ui = InterFace(fig, ax)
spline = Spline()
mpc_controller = Controller()
trajectorydb_ref = TrajectoryDB()
trajectorydb_computed = TrajectoryDB()

def generate_path_callback(event):
    x, y = ui.get_x_y()  
    if len(x) < 2:
        print("Click at least 2 points!")
        return
    rx, ry, ryaw = spline.calculate_spline(x, y)
    ui.plot_path(rx, ry, '-r', 'Reference')
    trajectorydb_ref.set_trajectorydb(np.vstack((rx, ry, ryaw)))

def simulate(event):
    ui.get_ax().set_title("Please wait for calculation...")
    ui.get_fig().canvas.draw()
    plt.pause(0.01)

    trajectory_ref = trajectorydb_ref.get_trajectorydb()
    trajectorydb_computed.set_trajectorydb(mpc_controller.compute_trajectory(trajectory_ref))

    ui.get_ax().set_title("Simulating...")
    ui.get_fig().canvas.draw()

    global robotsim
    robotsim = RobotSim(ui.get_fig(), ui.get_ax(), trajectorydb_computed.get_trajectorydb())


ui.add_button('generate path', [0.45, 0.01, 0.2, 0.05], callback=generate_path_callback)
ui.add_button('simulate', [0.7, 0.01, 0.2, 0.05], callback=simulate)

plt.show()
