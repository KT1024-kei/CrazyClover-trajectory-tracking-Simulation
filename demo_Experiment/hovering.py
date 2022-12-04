#!/user/bin/env python3
# coding: UTF-8

import sys
sys.path.append('../')
import time

import numpy as np
import matplotlib.pyplot as plt

from demo_Experiment.Env_experiment import Env_Experiment
from Controller.Controllers import Controllers
from Drone.Drone_model import Drone

def Experiment(Texp, Tsam, num_drone):

    # * initialize simulation environment
    Env = Env_Experiment(Texp, Tsam, 0)
    Drone_env = [0]*num_drone
    Drone_ctrl = [0]*num_drone

    # * set flag for Experiment
    track_flag = True
    stop_flag = True
    land_flag = True

    # * set control flow time 
    takeoff_time = 10
    Texp = Texp + takeoff_time
    stop_time = Texp + 0
    land_time = stop_time + 10

    # * set simulation time, controller, takeoff command
    for i in range(num_drone):
        Drone_env[i] = Env_Experiment(Texp, Tsam, i)
        Drone_ctrl[i] = Controllers(Tsam, "mellinger")
        Drone_env[i].takeoff(Drone_ctrl[i])

    # * set initial state, plot dorne
    cf = [0]*num_drone
    for i in range(num_drone):
      cf[i] = Drone(Tsam)
      Drone_env[i].init_state(cf[i])
      Drone_env[i].init_plot(None)

    # * simulation loop
    t = 0
    cnt = 0
    while True:
        
        # * set time and take log
        for i in range(num_drone):
            Drone_env[i].set_clock(t)
            Env.set_clock(t)
            Drone_env[i].take_log(Drone_ctrl[i])

         # * update states
        for i in range(num_drone):
            Drone_env[i].update_state(cf[i])

        # * set controller drone states and get controller input
        for i in range(num_drone):
            Drone_ctrl[i].set_state(Drone_env[i].P, Drone_env[i].V, Drone_env[i].R, Drone_env[i].Euler)
            Drone_ctrl[i].get_output(t)

        # * calculate drone motion 
        for i in range(num_drone):
            cf[i].main(Drone_ctrl[i].input_acc, Drone_ctrl[i].input_Wb)

        # * land part
        if t > Texp:
            for i in range(num_drone):
                if land_flag:
                    Drone_env[i].land_track(Drone_ctrl[i])
                    land_flag = False

        # * update plot 
        for i in range(num_drone):
            if cnt/10 == 1:
                Drone_env[i].update_plot(cf[i].world_frame)
                plt.pause(Tsam*10)
                cnt = 0
            cnt += 1
        
        # * time managment
        if Env.time_check(Tsam, land_time): break
        t += Tsam
    
    # * save log 
    for i in range(num_drone):
        Drone_env[i].save_log()

if __name__ == "__main__":
  Experiment(5, 0.01, 1)




