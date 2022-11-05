## CrazyClover-Simulation

### This project contains
- 中型ドローン（CrazyClover）を利用する実験のためのシミュレーション環境

### Dependencies
- numpy, matplotlib, rtree-linux

### Experiments ( demo_Experiment )
- hovering.py : ホバリング実験
- circle_tracking.py : 円軌道追従実験 ( sin and cos function )
- polytraj_tracking.py : 直線軌道追従 ( polynominal trajectory )


#### circle trajectory tracking
```python
cd demo_Experiment
python3 circle_tracking.py
```

https://user-images.githubusercontent.com/64090003/200113167-985e8bf8-c826-4505-8f41-04366166c731.mp4

### references 
- [Minimum Snap Trajectory Generation and Control for Quadrotors - Daniel Mellinger and Vijay Kumar](https://arxiv.org/pdf/1706.06478.pdf)
- [Polynomial Trajectory Planning for Aggressive Quadrotor Flight in Dense Indoor Environments - Charles Richter, Adam Bry, and Nicholas Roy](https://groups.csail.mit.edu/rrg/papers/Richter_ISRR13.pdf)