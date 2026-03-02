from client2server import client2server
import time

LOST_DX = -1536  # маркер не найден


def _dx_from_status(st: int) -> int:
    dx = st & 0x0FFF
    if dx >= 0x800:
        dx -= 0x1000
    return dx


def _pos_from_status(st: int) -> int:
    return (st >> 16) & 0xF  # 3=крайнее левое, 4=крайнее правое


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class tracker:
    def run(tracklog):
        c2s = client2server()

        # --- Настройки под твой track.log ---
        DEAD_STOP = 3          # стоп если |dx| <= 3
        DEAD_START = 6         # снова начать двигаться только если |dx| >= 6 (гистерезис)

        # PID (под диапазон |dx| до ~150)
        KP = 0.085             # агрессивнее, чем раньше (чтобы p90/p95 снижались)
        KI = 0.0040            # убирает "залипание" на смещении
        KD = 0.020             # демпфирование (меньше перерегулирование)

        I_MAX = 2500.0         # анти-виндап

        MIN_SPD = 1
        MAX_SPD = 8

        # сглаживание dx, чтобы D-часть не шумела
        ALPHA = 0.35

        # Поиск (когда dx==-1536)
        SEARCH_SPD = 6
        LOST_ENTER_CNT = 2
        SEEN_EXIT_CNT = 2

        # Переотправка команды, чтобы не было эффекта "радар не двигается"
        KEEPALIVE = 0.20

        LOOP_DT = 0.004

        # --- Состояния ---
        mode = "TRACK"
        lost_cnt = 0
        seen_cnt = 0

        dx_f = 0.0
        prev_dx_f = 0.0
        integ = 0.0
        prev_t = time.monotonic()

        last_cmd = ("none", 0)
        last_cmd_t = time.monotonic()

        last_good_dx = 0
        search_dir = +1

        # логирование
        wcnt = 0
        last_flush_t = time.monotonic()

        moving = False  # для гистерезиса DEAD_STOP/DEAD_START

        def send(cmd: str, spd: int):
            nonlocal last_cmd, last_cmd_t
            now = time.monotonic()
            key = (cmd, int(spd))

            if key == last_cmd and (now - last_cmd_t) < KEEPALIVE:
                return

            if cmd == "stop":
                c2s.moveStop()
            elif cmd == "left":
                c2s.moveLeft(int(spd))
            else:
                c2s.moveRight(int(spd))

            last_cmd = key
            last_cmd_t = now

        while True:
            st = int(c2s.getStatus())
            dx = _dx_from_status(st)
            pos = _pos_from_status(st)

            now = time.monotonic()
            dt = now - prev_t
            prev_t = now
            if dt <= 0:
                dt = 1e-3

            # --- LOG dx ---
            try:
                tracklog.write(f"{dx}\n".encode())
                wcnt += 1
                if wcnt % 80 == 0 or (now - last_flush_t) > 0.3:
                    tracklog.flush()
                    last_flush_t = now
            except Exception:
                pass

            # --- потери маркера ---
            if dx == LOST_DX:
                lost_cnt += 1
                seen_cnt = 0
            else:
                seen_cnt += 1
                lost_cnt = 0
                last_good_dx = dx

            # --- переход режимов ---
            if mode == "TRACK" and lost_cnt >= LOST_ENTER_CNT:
                mode = "SEARCH"
                integ = 0.0
                moving = False
            elif mode == "SEARCH" and seen_cnt >= SEEN_EXIT_CNT:
                mode = "TRACK"
                dx_f = float(last_good_dx)
                prev_dx_f = dx_f
                integ = 0.0

            # --- SEARCH ---
            if mode == "SEARCH":
                if pos == 3:
                    search_dir = -1
                elif pos == 4:
                    search_dir = +1

                # сначала пробуем в сторону, где был последний dx
                if last_good_dx > 0:
                    send("left", SEARCH_SPD)
                elif last_good_dx < 0:
                    send("right", SEARCH_SPD)
                else:
                    send("left", SEARCH_SPD) if search_dir > 0 else send("right", SEARCH_SPD)

                time.sleep(LOOP_DT)
                continue

            # --- TRACK (PID) ---
            dx_f = (1.0 - ALPHA) * dx_f + ALPHA * float(dx)
            d_dx = (dx_f - prev_dx_f) / dt
            prev_dx_f = dx_f

            a = abs(dx_f)

            # гистерезис около нуля
            if moving:
                if a <= DEAD_STOP:
                    moving = False
                    send("stop", 0)
                    time.sleep(LOOP_DT)
                    continue
            else:
                if a < DEAD_START:
                    send("stop", 0)
                    time.sleep(LOOP_DT)
                    continue
                moving = True

            # интеграл копим только когда маркер виден и ошибка не гигантская
            integ = _clamp(integ + dx_f * dt, -I_MAX, I_MAX)

            u = KP * dx_f + KI * integ + KD * d_dx
            spd = int(round(_clamp(abs(u), MIN_SPD, MAX_SPD)))

            if u > 0:
                send("left", spd)
            else:
                send("right", spd)

            time.sleep(LOOP_DT)
