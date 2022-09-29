#!/user/bin/env python
# coding: UTF-8

import sys

from matplotlib.pyplot import flag
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

    circle_flag = True
    stop_flag = True

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

        # リアルな状態とノミナルな状態を記録
        for i in range(num_drone):
            Drone_env[i].take_log(t, Drone_ctrl[i])

        # 状態を更新
        for i in range(num_drone):
            Drone_env[i].update_state(cf[i])

        # 5秒から15秒でコントローラを軌道追従に変更
        if 5 < t < 15:
            if circle_flag:
                Drone_ctrl[i].switch_controller("mellinger")
                Drone_env[i].track_circle(Drone_ctrl[i], False)
                circle_flag = False

        # 実験終了したら軌道追従を終了
        if Texp < t:
            if stop_flag:
                Drone_env[i].stop_track(Drone_ctrl[i])
                stop_flag = False

        # 実験終了5秒後に着陸
        if t > Texp+5:
            for i in range(num_drone):
              Drone_env[i].land(Drone_ctrl[i])

        # 現在の状態をコントローラに渡して入力を計算
        for i in range(num_drone):
            Drone_ctrl[i].set_state(Drone_env[i].P, Drone_env[i].V, Drone_env[i].R, Drone_env[i].Euler)
            Drone_ctrl[i].get_output(t)


        # ドローンへ加速度と加速度を入力
        for i in range(num_drone):
            cf[i].main(Drone_ctrl[i].input_acc, Drone_ctrl[i].input_Wb)


        # 実験時間を管理
        if Env.time_check(t, Tsam, Texp+10): break
        t += Tsam

    # ログデータを保存
    for i in range(num_drone):
        Drone_env[i].save_log()

if __name__ == "__main__":
  Experiment(20, 0.001, 1)



