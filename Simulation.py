#-*- coding: utf-8 -*-
"""
tips 

自作モジュールを読み込みたい時はsettingsのところでpython.analysis.extraPathsにパスを登録する

"""

import numpy as np
from Mathfunction import Mathfunction, Integration
from Drone_model import Drone, State
# from Controller import Controller
from Controllers import Controllers
from Log import Log_data



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
        if controller_type == "pid":
            if command =="hovering":
                P = np.array([0.0, 0.0, 1.0])
                controller.set_reference(P, V, R, Euler, Wb, Euler_rate, controller_type)    
            elif command == "land":
                P = np.array([0.0, 0.0, 0.0])
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
        

    def main(self):
        print("main function")
        t = 0.0
        T, M = 0.0, np.array([0.0, 0.0, 0.0])
        while self.T_exp > t:
            
            for i in range(self.num_drone):
                self.state_step(self.drones[i])
                self.logs[i].write_state(t=t, P=self.P, V=self.V, R=self.R, Euler=self.Euler, Wb=self.Wb, Euler_rate=self.Euler_rate*180/np.pi, T=T, M=M)
                
            
            for i in range(self.num_drone):
                self.ctrls[i].set_state(P=self.P, V=self.V, R=self.R, Euler=self.Euler, Wb=self.Wb, Euler_rate=self.Euler_rate)
                self.ctrls[i].get_output(t)
                self.ctrls[i].log(self.logs[i], t)
                
            for i in range(self.num_drone):
                T = self.ctrls[i].input_thrust_pwm
                M = self.ctrls[i].input_M_pwm
                self.drones[i].main(T, M)

            t += self.dt
        for i in range(self.num_drone):
            self.logs[i].close_file()
    
Simulation(10.0, 0.001, 1).main()