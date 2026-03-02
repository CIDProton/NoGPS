from client2server import client2server
import time

LOST_DX = -1536

class tracker:
    @staticmethod
    def run(tracklog):
        c2s = client2server()

        # --- Настройки PID ---
        KP = 0.085         
        KI = 0.0050        
        KD = 0.025         
        I_MAX = 2000.0     

        MIN_SPD = 1
        MAX_SPD = 8
        SEARCH_SPD = 6
        
        ALPHA = 0.35       # Коэффициент сглаживания (low-pass filter)

        LOST_ENTER_CNT = 2
        SEEN_EXIT_CNT = 2
        KEEPALIVE = 0.15   # Отправка команды удержания

        # --- Состояния ---
        mode = "TRACK"
        lost_cnt = 0
        seen_cnt = 0

        dx_f = 0.0
        prev_dx_f = 0.0
        integ = 0.0
        
        last_cmd = ("none", 0)
        
        last_good_dx = 0
        search_dir_left = True  # Направление поиска при потере

        wcnt = 0
        
        # --- Кэширование методов для максимальной скорости цикла ---
        # (В Python вызов локальной функции быстрее, чем обращение к методу класса через точку)
        get_status = c2s.getStatus
        move_stop = c2s.moveStop
        move_left = c2s.moveLeft
        move_right = c2s.moveRight
        log_write = tracklog.write
        log_flush = tracklog.flush
        get_time = time.monotonic
        
        last_cmd_t = get_time()
        last_flush_t = get_time()
        prev_t = get_time()

        while True:
            # 1. Чтение сенсоров (Inlined bitwise operations)
            st = int(get_status())
            
            dx = st & 0x0FFF
            if dx >= 0x800:
                dx -= 0x1000
                
            pos = (st >> 16) & 0xF
            
            # 2. Тайминги
            now = get_time()
            dt = now - prev_t
            prev_t = now
            if dt <= 0.0:
                dt = 0.001 # Защита от деления на ноль при сверхбыстром цикле

            # 3. Логирование (быстрое форматирование байт)
            try:
                log_write(b"%d\n" % dx)
                wcnt += 1
                if wcnt >= 100 or (now - last_flush_t) > 0.3:
                    log_flush()
                    wcnt = 0
                    last_flush_t = now
            except Exception:
                pass

            # 4. Детекция потери/нахождения
            if dx == LOST_DX:
                lost_cnt += 1
                seen_cnt = 0
            else:
                seen_cnt += 1
                lost_cnt = 0
                last_good_dx = dx

            # Переходы между режимами
            if mode == "TRACK" and lost_cnt >= LOST_ENTER_CNT:
                mode = "SEARCH"
                integ = 0.0
            elif mode == "SEARCH" and seen_cnt >= SEEN_EXIT_CNT:
                mode = "TRACK"
                dx_f = float(last_good_dx)
                prev_dx_f = dx_f
                integ = 0.0

            # 5. Логика управления
            cmd = "none"
            spd = 0

            if mode == "SEARCH":
                # Концевики (pos) имеют приоритет над last_good_dx
                if pos == 3:       # Уперлись влево -> ищем вправо
                    search_dir_left = False
                elif pos == 4:     # Уперлись вправо -> ищем влево
                    search_dir_left = True
                else:
                    # Иначе двигаемся туда, где спутник был в последний раз
                    if last_good_dx > 0:
                        search_dir_left = True
                    elif last_good_dx < 0:
                        search_dir_left = False

                cmd = "left" if search_dir_left else "right"
                spd = SEARCH_SPD

            else: # mode == "TRACK"
                # Экспоненциальное сглаживание и дериватив
                dx_f = (1.0 - ALPHA) * dx_f + ALPHA * float(dx)
                d_dx = (dx_f - prev_dx_f) / dt
                prev_dx_f = dx_f

                # Для узконаправленного луча не используем мертвую зону (DEAD_STOP)
                # Останавливаемся только если попали ИДЕАЛЬНО в цель (менее 0.5 пикселя)
                if abs(dx_f) < 0.5:
                    cmd = "stop"
                    spd = 0
                else:
                    # Вычисление PID
                    u = KP * dx_f + KI * integ + KD * d_dx
                    
                    cmd = "left" if u > 0 else "right"
                    abs_u = abs(u)
                    
                    # Зажим (Clamp) скорости
                    if abs_u < MIN_SPD:
                        spd = MIN_SPD
                    elif abs_u > MAX_SPD:
                        spd = MAX_SPD
                    else:
                        spd = int(round(abs_u))
                    
                    # Интеллектуальный Anti-Windup
                    # Копим ошибку только если мы не на максимальной скорости
                    # ИЛИ если ошибка толкает нас в сторону, обратную насыщению
                    if not (spd == MAX_SPD and ((u > 0 and dx_f > 0) or (u < 0 and dx_f < 0))):
                        integ += dx_f * dt
                        if integ > I_MAX: integ = I_MAX
                        elif integ < -I_MAX: integ = -I_MAX

            # 6. Отправка команды
            key = (cmd, spd)
            if key != last_cmd or (now - last_cmd_t) >= KEEPALIVE:
                if cmd == "stop":
                    move_stop()
                elif cmd == "left":
                    move_left(spd)
                else:
                    move_right(spd)
                
                last_cmd = key
                last_cmd_t = now