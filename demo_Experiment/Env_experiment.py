#-*- coding: utf-8 -*-
"""
There are usefull tools to do Experiment

"""
import sys
sys.path.append('../')
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
from collections import deque

from tools.Decorator import run_once
from tools.Mathfunction import Mathfunction, Integration
from tools.Log import Log_data
from Drone.Drone_model import Drone, State
from Controller.Controllers import Controllers

class Env_Experiment(Mathfunction):

# * ========================= Initialize part ============================ * 
    def __init__(self, Texp, Tsam, num):

        # * Experiment parametor
        self.Tend = Texp
        self.dt = Tsam
        self.num = num
        self.t = 0        
        
        self.log = Log_data(num)

    def init_state(self, drone = 0, 
                                P=np.array([0.0, 0.0, 0.0]),   
                                V=np.array([0.0, 0.0, 0.0]), 
                                R=np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]), 
                                Euler=np.array([0.0, 0.0, 0.0]), 
                                Wb=np.array([0.0, 0.0, 0.0]), 
                                Euler_rate=np.array([0.0, 0.0, 0.0])):

        R = self.Euler2Rot(Euler)
        self.P          = P
        self.V          = V
        self.Euler      = Euler
        self.R          = R
        self.Wb         = Wb
        self.Euler_rate = Euler_rate
        self.M = drone.M


        drone.set_initial_state(P, V, R, Euler, Wb, Euler_rate, self.dt)

    def init_plot(self,ax = None):
        if ax is None:
            fig = plt.figure()
            ax = Axes3D.Axes3D(fig)
            ax.set_xlim((-2,2))
            ax.set_ylim((-2,2))
            ax.set_zlim((0,1.5))
        ax.plot([], [], [], '-', c='red',zorder = 10)
        ax.plot([], [], [], '-', c='blue',zorder = 10)
        ax.plot([], [], [], '-', c='green', marker='o', markevery=2,zorder = 10)
        ax.plot([], [], [], '.', c='green', markersize=2,zorder = 10)
        self.lines = ax.get_lines()[-4:]
        self.pos_history = deque(maxlen=200)

    def update_plot(self,frame):

        # * plot quadrotor bodies in frame
        lines_data = [frame[:,[0,2]], frame[:,[1,3]], frame[:,[4,5]]]

        for line, line_data in zip(self.lines[:3], lines_data):
            x, y, z = line_data
            line.set_data(x, y)
            line.set_3d_properties(z)

        self.pos_history.append(frame[:,4])
        history = np.array(self.pos_history)
        self.lines[-1].set_data(history[:,0], history[:,1])
        self.lines[-1].set_3d_properties(history[:,-1])

# * ==================================================================== * 

    def set_clock(self, t):
        self.t = t

    def set_reference(self, controller,  
                            P=np.array([0.0, 0.0, 0.0]),   
                            V=np.array([0.0, 0.0, 0.0]), 
                            R=np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]), 
                            Euler=np.array([0.0, 0.0, 0.0]), 
                            Wb=np.array([0.0, 0.0, 0.0]), 
                            Euler_rate=np.array([0.0, 0.0, 0.0]),
                            traj="circle",
                            controller_type="pid",
                            command = "hovering",
                            init_controller=True,
                            tmp_P=np.zeros(3)): 

        # * reinitialize controller
        if init_controller:
            controller.select_controller()
        
        # * set reference command to controller
        if controller_type == "pid":
            if command =="hovering":
                controller.set_reference(P, V, R, Euler, Wb, Euler_rate, controller_type)    
            elif command == "land":
                controller.set_reference(P, V, R, Euler, Wb, Euler_rate, controller_type) 
            else:
                controller.set_reference(P, V, R, Euler, Wb, Euler_rate, controller_type)
        elif controller_type == "mellinger":
            controller.set_reference(traj, self.t, tmp_P)
        
    def set_dt(self, dt):
        self.dt = dt
        
    def update_state(self, drone):
        
        self.P          =     drone.P.now
        self.V          =     drone.V.now
        self.Euler      =     drone.Euler.now
        self.R          =     drone.R.now
        self.Wb         =     drone.Wb.now
        self.Euler_rate =     drone.Euler_rate.now
        drone.Euler_rate.now[1] =  -drone.Euler_rate.now[1] # * inverse the pitch rate same as crazyflie
        self.M = drone.M

    def take_log(self, ctrl):
        self.log.write_state(self.t, self.P, self.V, self.R, self.Euler, np.zeros(3), np.zeros(3), self.M)
        ctrl.log(self.log, self.t)
        
    def save_log(self):
      self.log.close_file()

    def time_check(self, Tint, Tend):
        if self.t > Tend:
            return True
        return False

# * ======================== controll commands ================================ *  

    @run_once
    def land(self, controller):
        controller.switch_controller("pid")
        self.set_reference(controller=controller, command="land", init_controller=True, P=self.land_P)
        
    @run_once
    def hovering(self, controller, P=np.array([0.0, 0.0, 1.0])):
        self.set_reference(controller=controller, command="hovering", P=P)
        self.land_P = np.array([0.0, 0.0, 0.1])

    def takeoff(self, controller, Pinit=np.array([0.0, 0.0, 0.0])):
        self.set_reference(controller=controller, traj="takeoff", controller_type="mellinger", tmp_P=Pinit)
        self.land_P = np.array([0.0, 0.0, 0.1])

    def land_track(self, controller):
        self.set_reference(controller=controller, traj="land", controller_type="mellinger", init_controller=False, tmp_P=np.array([self.P[0], self.P[1], 0.0]))

    def track_circle(self, controller, flag):
        self.set_reference(controller=controller, traj="circle", controller_type="mellinger", init_controller=flag)
    
    def track_straight(self, controller, flag):
        self.set_reference(controller=controller, traj="straight", controller_type="mellinger", init_controller=flag)

    def stop_track(self, controller):
        self.set_reference(controller=controller, traj="stop", controller_type="mellinger", init_controller=False, tmp_P=np.array([self.P[0], self.P[1], 1.0]))
        self.land_P[0:2] = self.P[0:2]