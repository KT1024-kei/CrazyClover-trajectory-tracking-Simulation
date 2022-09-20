#!/user/bin/env python
# coding: UTF-8

import sys
sys.path.append('../')
import time
import numpy as np


from demo_Experiment.Env_experiment import Env_Experiment
from Controller.Controllers import Controllers
from Drone.Drone_model import Drone

def Experiment(Texp, Tsam, num_drone):

    Env = Env_Experiment(Texp, Tsam, 0)
    Drone_env = [0]*num_drone
    Drone_ctrl = [0]*num_drone
    Drone_log = [0]*num_drone
    # swarm = Crazyswarm()
    # timeHelper = swarm.timeHelper
    # cf = swarm.allcfs.crazyflies


    zero = np.zeros(3)
    for i in range(num_drone):
        Drone_env[i] = Env_Experiment(Texp, Tsam, i)
        Drone_ctrl[i] = Controllers(Tsam, "pid")
        Drone_env[i].hovering(Drone_ctrl[i])

    cf = [0]*num_drone
    for i in range(num_drone):
      cf[i] = Drone(Tsam)
      Drone_env[i].init_state(cf[i])

    t = 0
    while True:

        for i in range(num_drone):
            Drone_env[i].take_log(t, Drone_ctrl[i])

        for i in range(num_drone):
            Drone_env[i].update_state(cf[i])

        for i in range(num_drone):
            Drone_ctrl[i].set_state(Drone_env[i].P, Drone_env[i].V, Drone_env[i].R, Drone_env[i].Euler)
            Drone_ctrl[i].get_output(t)

        for i in range(num_drone):
            cf[i].main(Drone_ctrl[i].input_acc, Drone_ctrl[i].input_Wb)

        if t > 10:
            for i in range(num_drone):
              Drone_env[i].land(Drone_ctrl[i])
        if Env.time_check(t, Tsam, Texp): break

        t += Tsam
    
    for i in range(num_drone):
        Drone_env[i].save_log()
if __name__ == "__main__":
  Experiment(15, 0.001, 1)




