#!/user/bin/env python
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
    stop_time = Texp + 5
    land_time = stop_time + 10

    for i in range(num_drone):
        Drone_env[i] = Env_Experiment(Texp, Tsam, i)
        Drone_ctrl[i] = Controllers(Tsam, "mellinger")
        Drone_env[i].takeoff(Drone_ctrl[i], [-1.0, 0.0, 0.0])

    cf = [0]*num_drone
    for i in range(num_drone):
      cf[i] = Drone(Tsam)
      Drone_env[i].init_state(cf[i], P=np.array([-1.0, 0.0, 0.0]))
      Drone_env[i].init_plot(None)

    # * simulation loop
    t = 0
    cnt = 0
    while True:

        # * set time and log state
        for i in range(num_drone):
            Env.set_clock(t)
            Drone_env[i].set_clock(t)
            Drone_env[i].take_log(Drone_ctrl[i])

        # * update state
        for i in range(num_drone):
            Drone_env[i].update_state(cf[i])

        # * trajectory track
        if takeoff_time < t < Texp:
            if track_flag:
                Drone_env[i].track_circle(Drone_ctrl[i], False)
                track_flag = False

        # * stop tracking
        if Texp < t:
            if stop_flag:
                Drone_env[i].stop_track(Drone_ctrl[i])
                stop_flag = False

        # * landing
        if t > stop_time:
            for i in range(num_drone):
                if land_flag:
                    Drone_env[i].land_track(Drone_ctrl[i])
                    land_flag = False

        # * set state to controller and calculate controller input
        for i in range(num_drone):
            Drone_ctrl[i].set_state(Drone_env[i].P, Drone_env[i].V, Drone_env[i].R, Drone_env[i].Euler)
            Drone_ctrl[i].get_output(t)


        # * input command to quadrotor
        for i in range(num_drone):
            cf[i].main(Drone_ctrl[i].input_acc, Drone_ctrl[i].input_Wb)

        for i in range(num_drone):
            if cnt/10 == 1:
                Drone_env[i].update_plot(cf[i].world_frame)
                plt.pause(Tsam*10)
                cnt = 0
            cnt += 1
        
        # * manage Experiment time
        if Env.time_check(Tsam, land_time): break
        t += Tsam

    # ????????????????????????
    for i in range(num_drone):
        Drone_env[i].save_log()

if __name__ == "__main__":
  Experiment(5, 0.01, 1)




