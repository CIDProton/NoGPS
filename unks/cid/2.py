from client2server import client2server
import time

class tracker:
    def __init__(self):
        self.last_dx = 0
        self.last_time = time.time()
        
        # --- ТВОИ ИДЕАЛЬНЫЕ КОЭФФИЦИЕНТЫ ---
        self.Kp = 0.64   # Тот самый твой кэф. Задает базовую реакцию на ошибку.
        
        # Интеграл (Cruise Control). Именно он убирает отставание!
        # Начинай с 0.2. Если все равно отстает — ставь 0.5. Если начинает болтаться вокруг центра — уменьшай.
        self.Ki = 0.2    
        
        # Дифференциал (Тормоз). Гасит резкие рывки.
        self.Kd = 0.15   
        
        self.integral = 0 
        self.max_move = 100 # Твой максимум

    def run(self, tracklog):
        c2s = client2server()
        
        while True:
            status = c2s.getStatus()
            
            # Декодирование DX
            dx = int(status) & 0x0fff
            if dx > 2048:
                dx = dx - 4096
            
            # Считаем время кадра (dt)
            current_time = time.time()
            dt = current_time - self.last_time
            if dt <= 0.001: dt = 0.01 # Защита от деления на ноль, если цикл пролетел слишком быстро
            
            # --- ПИД РЕГУЛЯТОР ---
            
            # 1. Пропорция (твоя старая добрая dx * 0.64)
            P = dx * self.Kp
            
            # 2. Интеграл (копит отставание). 
            self.integral += dx * dt
            # Защита от переполнения (чтобы интеграл не накрутил лишнего, пока спутника нет)
            # Ограничим его вклад максимум 60 единицами скорости
            max_i = 60 / self.Ki if self.Ki > 0 else 0
            self.integral = max(min(self.integral, max_i), -max_i)
            I = self.integral * self.Ki
            
            # 3. Производная (считает скорость сближения)
            derivative = (dx - self.last_dx) / dt
            D = derivative * self.Kd
            
            # Складываем всё вместе. 
            k_raw = P + I + D
            
            # --- ФОРМИРОВАНИЕ КОМАНДЫ ---
            k = int(abs(k_raw)) # Берем модуль
            
            if k > self.max_move: 
                k = self.max_move
            elif k < 1 and dx != 0: # Чтобы не залипал на микро-ошибках
                k = 1

            # Движение (знак k_raw сам решает, куда крутить)
            if k_raw > 0:
                c2s.moveLeft(k)
            elif k_raw < 0:
                c2s.moveRight(k)
            else:
                pass # Если всё идеально по нулям - стоим

            # Сохраняем состояние для следующего тика
            self.last_dx = dx
            self.last_time = current_time
            
            # Небольшая пауза, чтобы не насиловать проц и датчики
            time.sleep(0.01)