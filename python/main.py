# main.py  (mesmo que antes, sem alterações)
#!/usr/bin/env python3
import serial
import threading
import pyautogui
import time

USB_PORT = '/dev/tty.usbmodem102'
BT_PORT  = '/dev/rfcomm0'
BAUD_RATE = 115200

pyautogui.FAILSAFE = False
pyautogui.PAUSE = 0

def reader(ser):
        
    fire_pressed = False
    while True:
        line = ser.readline().decode('ascii', errors='ignore').strip()
        if not line:
            continue
        parts = line.split(',')
        if len(parts) != 7:
            continue

        try:
            r, p, y, clk, enc, aim, fire = parts
            r   = float(r)
            p   = float(p)
            y   = float(y)
            clk = bool(int(clk))
            enc = int(enc)
            aim = bool(int(aim))
            fire= bool(int(fire))
        except ValueError:
            continue

        # movimenta cursor
        pyautogui.moveRel(r/4, -p/4, duration=0)

        # clique por inclinação
        if clk:
            pyautogui.click()

        # encoder → teclas '9' e '3'
        if enc > 0:
            for _ in range(enc):
                pyautogui.press('s')
        elif enc < 0:
            for _ in range(-enc):
                pyautogui.press('s')

        # mirar (left click)
        if aim:
            pyautogui.leftClick()
        # atirar (right click)

        if fire and not fire_pressed:
            # Pressiona e mantém “w”
            pyautogui.keyDown('w')
            fire_pressed = True
        elif not fire and fire_pressed:
            # Solta “w”
            pyautogui.keyUp('w')
            fire_pressed = False

if __name__ == '__main__':
    ser_bt = serial.Serial(BT_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    t = threading.Thread(target=reader, args=(ser_bt,), daemon=True)
    t.start()
    print("Leitura via Bluetooth iniciada. Ctrl+C para sair.")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        ser_bt.close()
        print("Encerrando...")