import time
import threading
from pymavlink import mavutil

MIN_ALTITUDE = 5
MAX_ALTITUDE = 50
LOW_BATTERY_LEVEL = 20
TAKEOFF_TIMEOUT = 30

# Подключение дрона
def connect_to_dron():
    print("Подключение к дрону...")
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    master.wait_heartbeat(timeout=10)
    print("Соединение установлено! Получен heartbeat!")
    return master

# Установка режима полёта
def set_mode(master, mode_name):
    print(f"Переключение в режим {mode_name}...")
    
    # Для симулятора PX4 используем числовой custom_mode
    if mode_name == "OFFBOARD":
        custom_mode = 6
    elif mode_name == "LAND":
        custom_mode = 9
    else:
        print("Неизвестный режим")
        return False
    
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,
        custom_mode,
        0, 0, 0, 0, 0
    )
    
    # Ожидание подтверждения (393216 = OFFBOARD)
    start = time.time()
    while time.time() - start < 10:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            cm = msg.custom_mode
            print(f"Текущий custom_mode: {cm}")
            if cm == 393216 or (cm & 0xFFFF) == custom_mode:
                print(f"Режим успешно изменён на {mode_name}!")
                return True
    print("Не удалось переключить режим")
    return False

# Проверка батареи
def check_battery(master):
    msg = master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
    if not msg:
            print("Не удалось получить данные о батарее (SYS_STATUS).")
            return False
    
    battery = msg.battery_remaining

    print(f"Текущий уровень заряда {battery}%")

    if battery < LOW_BATTERY_LEVEL:
        print("Ошибка: низкий уровень заряда батареи.")
        return False
    return True

# Запуск двигателей
# ARM - разблокировка двигателей
def arm(master):
    print("Выполняется ARM двигателей...")
    
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("Двигатели успешно запущены!")
    
# Взлёт дрона
def takeoff(master, altitude):
    print(f"Взлёт на высоту {altitude} м...")
    
    # Получение текущих координат
    pos = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
    x = pos.x if pos else 0.0
    y = pos.y if pos else 0.0
    target_z = -altitude # Координата NED (вниз), отрицательная при наборе высоты
    
    start_time = time.time()
    while True:
        master.mav.set_position_target_local_ned_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000,
            x, y, target_z,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if msg:
            current_alt = -msg.z
            print(f"Текущая высота: {current_alt:.2f}м")
            if current_alt >= altitude * 0.95:
                print("Высота достигнута!")
                break
        
        if time.time() - start_time > TAKEOFF_TIMEOUT:
            print("ОШИБКА: превышено время набора высоты.")
            set_mode(master, "LAND")
            break

def monitor_flight(master):
    print("Мониторинг параметров полёта...")
    while True:
        pos = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=2)
        sys = master.recv_match(type='SYS_STATUS', blocking=False)
        hb = master.recv_match(type='HEARTBEAT', blocking=False)
        
        alt = -pos.z if pos else 0.0
        battery = sys.battery_remaining if sys else -1
        mode = hb.custom_mode if hb else 0
        
        print(f"Режим: {mode} | Высота: {alt:.2f} | Батарея: {battery}%")
        
        if battery >= 0 and battery < LOW_BATTERY_LEVEL:
            print("Низкий заряд! Принудительная посадка.")
            set_mode(master, "LAND")
            break
        time.sleep(2)

def land(master):
    print("Переключение в режим LAND...")
    set_mode(master, "LAND")
    master.motors_disarmed_wait()
    print("Посадка завершена! Двигатели выключены.")

def main():
    try:
        altitude = float(input("Введите высоту взлёта (м): "))
        if altitude < MIN_ALTITUDE or altitude > MAX_ALTITUDE:
            print("Ошибка: Допустимый диапазон высоты от 5 до 50 м.")
            return
        
        master = connect_to_dron()
        
        # Proof-of-life — сообщения-уведомления о работоспособности дрона
        print("Запускаем proof-of-life...")
        def send_loop():
            while True:
                master.mav.set_position_target_local_ned_send(
                    0,
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0b0000111111000,
                    0, 0, 0,
                    0, 0, 0,
                    0, 0, 0,
                    0, 0
                )
                time.sleep(0.2)
        
        threading.Thread(target=send_loop, daemon=True).start()
        time.sleep(2.5)
        
        set_mode(master, "OFFBOARD")
        
        if not check_battery(master):
            return
        
        arm(master)
        takeoff(master, altitude)
        monitor_flight(master)
        land(master)
    
    except Exception as e:
        print(f"Критическая ошибка приложения: {e}")

if __name__ == "__main__":
    main()