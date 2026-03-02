from client2server import client2server
import time

class tracker:
    def __init__(self):
        self.last_dx = 0
        self.last_time = time.time()
        
        # Коэффициенты ПИД (НУЖНО ПОДБИРАТЬ)
        self.Kp = 1.0   # Реакция на текущее отклонение (увеличь, если тупит)
        self.Ki = 0.2   # ИНТЕГРАЛ: Убирает отставание! Запоминает скорость спутника
        self.Kd = 0.3   # Демпфер: тормозит антенну при точном наведении, чтобы не дергалась
        
        self.integral = 0 # Память для накопленной ошибки
        self.max_move = 100 # Увеличил лимит. Возможно, 75 было просто слишком медленно аппаратно

    def run(self, tracklog):
        c2s = client2server()
        
        while True:
            status = c2s.getStatus()
            
            # 1. Декодирование DX
            dx = int(status) & 0x0fff
            if dx > 2048:
                dx = dx - 4096
            
            # 2. Расчет dt
            current_time = time.time()
            dt = current_time - self.last_time
            if dt <= 0: dt = 0.01 # Защита
            
            # 3. ПИД МАГИЯ (считаем с учетом знаков!)
            
            # Пропорциональный (тянет к цели)
            P = dx * self.Kp
            
            # Интегральный (копит отставание и превращает в постоянную скорость)
            self.integral += dx * dt
            # Ограничиваем интеграл, чтобы его не раздуло до небес (anti-windup)
            self.integral = max(min(self.integral, 300), -300) 
            I = self.integral * self.Ki
            
            # Дифференциальный (предугадывает скорость)
            derivative = (dx - self.last_dx) / dt
            D = derivative * self.Kd
            
            # Итоговая сила (может быть + или -)
            k_raw = P + I + D
            
            # 4. Перевод в команды для моторов
            k = int(abs(k_raw)) # Берем модуль для скорости
            
            if k > self.max_move: 
                k = self.max_move
            elif k < 2 and dx != 0: # Защита от остановки при малом dx
                k = 2

            # 5. Логика движения (знак k_raw определяет направление)
            if k_raw > 0:
                c2s.moveLeft(k)
            elif k_raw < 0:
                c2s.moveRight(k)

            # 6. Сохраняем состояние
            self.last_dx = dx
            self.last_time = current_time
            
            # ВАЖНО: нужна микропауза. Иначе цикл крутится миллион раз в секунду, 
            # dt становится микроскопическим, а производная (D) улетает в космос.
            time.sleep(0.01)