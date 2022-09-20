#-*- coding: utf-8 -*-
"""
tips 

自作モジュールを読み込みたい時はsettingsのところでpython.analysis.extraPathsにパスを登録する

"""
import sys
sys.path.append('../')
import numpy as np

from tools.Decorator import run_once
from tools.Mathfunction import Mathfunction, Integration
from tools.Log import Log_data
from Drone.Drone_model import Drone, State
from Controller.Controllers import Controllers

class Env_Experiment(Mathfunction):
    def __init__(self, Texp, Tsam, num):

        # Experiment Parametor
        self.Tend = Texp
        self.dt = Tsam
        self.num = num        
        
        self.log = Log_data(num)

    def init_state(self, drone = 0, 
                                P=np.array([0.0, 0.0, 0.0]),   
                                V=np.array([0.0, 0.0, 0.0]), 
                                R=np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]), 
                                Euler=np.array([0.0, 0.0, 0.0]), 
                                Wb=np.array([0.0, 0.0, 0.0]), 
                                Euler_rate=np.array([0.0, 0.0, 0.0])):

        print("set Initial state")
        self.P          = P
        self.V          = V
        self.Euler      = Euler
        self.R          = R
        self.Wb         = Wb
        self.Euler_rate = Euler_rate
        self.M = drone.M

        drone.set_initial_state(P, V, R, Euler, Wb, Euler_rate, self.dt)

# ------------------------------- ここまで　初期化関数 ---------------------
    def set_reference(self, controller,  
                            P=np.array([0.0, 0.0, 0.0]),   
                            V=np.array([0.0, 0.0, 0.0]), 
                            R=np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]), 
                            Euler=np.array([0.0, 0.0, 0.0]), 
                            Wb=np.array([0.0, 0.0, 0.0]), 
                            Euler_rate=np.array([0.0, 0.0, 0.0]),
                            traj="circle",
                            controller_type="pid",
                            command = "hovering"):
        controller.switch_controller(controller_type)
        if controller_type == "pid":
            if command =="hovering":
                P = np.array([0.0, 0.0, 1.0])
                controller.set_reference(P, V, R, Euler, Wb, Euler_rate, controller_type)    
            elif command == "land":
                P = np.array([0.0, 0.0, 0.0])
                controller.set_reference(P, V, R, Euler, Wb, Euler_rate, controller_type) 
            else:
                controller.set_reference(P, V, R, Euler, Wb, Euler_rate, controller_type)
        elif controller_type == "mellinger":
            controller.set_reference(traj)
        

    def update_state(self, drone):
        
        self.P          =     drone.P.now
        self.V          =     drone.V.now
        self.Euler      =     drone.Euler.now
        self.R          =     drone.R.now
        self.Wb         =     drone.Wb.now
        self.Euler_rate =     drone.Euler_rate.now
        drone.Euler_rate.now[1] =  -drone.Euler_rate.now[1]
        self.M = drone.M

    def take_log(self, t, ctrl):
        self.log.write_state(t, self.P, self.V, self.R, self.Euler, np.zeros(3), np.zeros(3), self.M)
        ctrl.log(self.log, t)
        
    def save_log(self):
      self.log.close_file()

    def time_check(self, t, Tint, Tend):
        if t > Tend:
            return True
        return False

    @run_once
    def land(self, controller):
        self.set_reference(controller=controller, command="land")
        
    @run_once
    def hovering(self, controller):
        self.set_reference(controller=controller, command="hovering")

    def track_circle(self, controller):
        self.set_reference(controller=controller, traj="circle", controller_type="mellinger")

    def stop_track(self, controller):
        self.set_reference(controller=controller, traj="stop", controller_type="mellinger")