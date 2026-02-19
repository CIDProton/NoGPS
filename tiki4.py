from client2server import client2server
import time

class tracker:
    def __init__(self):
        self.last_dx = 0
        self.last_time = time.time()
        self.Kp = 0.90
        self.Kd = 0.4
        self.min_move = 3

    def run(self, tracklog):
        c2s = client2server()
        while True:
            status = c2s.getStatus()
            
            dx = int(status) & 0x0fff
            if dx > 2048:
                dx = dx - 4096
            
            current_time = time.time()
            dt = current_time - self.last_time
            if dt == 0: dt = 0.01

            derivative = (dx - self.last_dx) / dt
            
            k = int((abs(dx) * self.Kp) + (abs(derivative) * self.Kd) + self.min_move)
            
            if k > 100: 
                k = 100

            if dx > 0:
                c2s.moveLeft(k)
            elif dx < 0:
                c2s.moveRight(k)
            
            self.last_dx = dx
            self.last_time = current_time
            