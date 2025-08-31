import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button


class InterFace:
    def __init__(self, fig, ax):
        self.fig = fig
        self.ax = ax
        self.x = []
        self.y = []
        self.buttons = []
        ax.set_xlim(0,10)
        ax.set_ylim(0,10)
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.on_click)

    def on_click(self, event):
        if event.inaxes != self.ax: 
            return
        if event.xdata is None or event.ydata is None:
            return 

        self.x.append(np.round(event.xdata,2))
        self.y.append(np.round(event.ydata,2))
        self.ax.scatter(self.x[-1],self.y[-1],marker='.',color='b')
        self.fig.canvas.draw()    

    def get_x_y(self):
        return (self.x, self.y)

    def plot_path(self, xpath, ypath, pathstyle, pathlabel):
        self.ax.plot(xpath, ypath, pathstyle, lw=1, label=pathlabel)
        self.ax.legend()
        self.fig.canvas.draw()

    def add_button(self, label, position, callback):
        ax_btn = self.fig.add_axes(position)
        button = Button(ax_btn, label)
        button.on_clicked(callback)
        self.buttons.append(button)
        return button
    
    def get_fig(self):
        return self.fig
    
    def get_ax(self):
        return self.ax


