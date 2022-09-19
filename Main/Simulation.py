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
# from Controller import Controller
from Controller.Controllers import Controllers



class Simulation(Mathfunction):
    def __init__(self, EXP_time, dt, num_drone):

        # Experiment Parametor
        self.T_exp = EXP_time
        self.dt = dt
        self.num_drone = num_drone
        self.drones = [0]*num_drone
        self.ctrls = [0]*num_drone
        self.logs = [0]*num_drone
        controller_type = "pid"
        for i in range(self.num_drone):
          self.logs[i] = Log_data(i)
          self.drones[i] = Drone(self.dt)
          self.ctrls[i] = Controllers(dt, controller_type)
          self.ctrls[i].select_controller()
          self.ctrls[i].init_controller()
          self.initialize_state(drone=self.drones[i])
          self.set_reference(controller=self.ctrls[i], controller_type=controller_type, command="hovering")
        

    def initialize_state(self, drone = 0, 
                                P=np.array([0.0, 0.0, 0.0]),   
                                V=np.array([0.0, 0.0, 0.0]), 
                                R=np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]), 
                                Euler=np.array([0.0, 0.0, 0.0]), 
                                Wb=np.array([0.0, 0.0, 0.0]), 
                                Euler_rate=np.array([0.0, 0.0, 0.0])):

        print("set Initial state")

        drone.set_initial_state(P, V, R, Euler, Wb, Euler_rate, self.dt)

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
        


    def state_step(self, drone):
        
        self.P          =     drone.P.now
        self.V          =     drone.V.now
        self.Euler      =     drone.Euler.now
        self.R          =     drone.R.now
        self.Wb         =     drone.Wb.now
        self.Euler_rate =     drone.Euler_rate.now
        drone.Euler_rate.now[1] =  -drone.Euler_rate.now[1]
        

    def state_step_flow(self, t, M):

        for i in range(self.num_drone):
            self.state_step(self.drones[i])
            self.logs[i].write_state(t=t, P=self.P, V=self.V, R=self.R, Euler=self.Euler, Wb=self.Wb, Euler_rate=self.Euler_rate*180/np.pi, M=M)
    
    def controll_flow(self, t):
        for i in range(self.num_drone):
            self.ctrls[i].set_state(P=self.P, V=self.V, R=self.R, Euler=self.Euler, Wb=self.Wb, Euler_rate=self.Euler_rate)
            self.ctrls[i].get_output(t)
            self.ctrls[i].log(self.logs[i], t)

    def Drone_move_flow(self, t):
        for i in range(self.num_drone):
            acc = self.ctrls[i].input_acc
            Wb = self.ctrls[i].input_Wb
            self.drones[i].main(acc, Wb)
            self.M = self.drones[i].M

    def Log_flow(self):
        for i in range(self.num_drone):
            self.logs[i].close_file()
    
    @run_once
    def land(self):
        for i in range(self.num_drone):
            self.set_reference(controller=self.ctrls[i], command="land")
    @run_once
    def hovering(self):
        for i in range(self.num_drone):
            self.set_reference(controller=self.ctrls[i], command="hovering")
    @run_once
    def track_circle(self):
        for i in range(self.num_drone):
            self.set_reference(controller=self.ctrls[i], traj="circle", controller_type="mellinger")
    @run_once
    def stop_track(self):
        for i in range(self.num_drone):
            self.ctrls[i].stop_tracking()



    def main(self):
        print("main function")
        t = 0.0
        self.M = np.array([0.0, 0.0 ,0.0, 0.0])
        while self.T_exp > t:
            self.state_step_flow(t, self.M)
            self.controll_flow(t)
            self.Drone_move_flow(t)
            t += self.dt
        self.Log_flow()
    
    def hovering_and_land(self):
        print("main function")
        t = 0.0
        self.M = np.array([0.0, 0.0 ,0.0, 0.0])
        self.hovering()
        while self.T_exp > t:
            self.state_step_flow(t, self.M)
            self.controll_flow(t)
            self.Drone_move_flow(t)
            t += self.dt
            if t > 10:
                self.land()
        self.Log_flow()
    
    def circle_track_experiment(self):
        print("main function")
        t = 0.0
        self.M = np.array([0.0, 0.0 ,0.0, 0.0])
        self.hovering()
        while self.T_exp > t:
            self.state_step_flow(t, self.M)
            self.controll_flow(t)
            self.Drone_move_flow(t)
            t += self.dt
            if 15 > t > 5:
                self.track_circle()
            if 20 > t > 15:
                self.stop_track()
            if t > 20:
                self.land()
        self.Log_flow()

            
    


Simulation(25.0, 0.001, 1).circle_track_experiment()