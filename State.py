class State():
  def __init__(self, dt, initial):
    print("Initialize State")
    self.dt = float(dt)
    self.pre = initial
    self.now = initial
    self.pre = 0.0

    self.deriv = 0.0

  def integration(self, d_state):
    self.pre = self.now
    self.now += d_state*self.dt    
  
  def derivertive(self):
    self.deriv = (self.now - self.pre)/self.dt
  
  def update(self, new):
    self.pre = self.now
    self.now = new