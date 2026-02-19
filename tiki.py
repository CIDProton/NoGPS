from client2server import client2server

class tracker:
    def run(tracklog):
        c2s = client2server()
        while True:
            status = c2s.getStatus()
            dx = int(status) & 0x0fff
            if dx > 2048:
                dx = dx - 4096
            k=(int((abs(dx))*0.64)+2)
            if abs(dx) < 500:
                if dx > 0:
                    c2s.moveLeft(k)
                else:
                    c2s.moveRight(k)
            else:
                dx2 = (int(status) >> 16) & 0x0f
                if dx2 == 3:
                    c2s.moveRight(100)
                if dx2 ==  4:
                    c2s.moveLeft(100)