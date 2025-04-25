# main.py
#!/usr/bin/env python3
import serial, threading, pyautogui, time

# você terá duas portas: USB e Bluetooth
USB_PORT      = '/dev/ttyACM0'
BT_PORT       = '/dev/rfcomm0'   # ou 'COM6' no Windows
BAUD_RATE     = 115200
pyautogui.FAILSAFE = False
pyautogui.PAUSE    = 0

def reader(ser):
    while True:
        line = ser.readline().decode('ascii', errors='ignore').strip()
        if not line: continue
        parts = line.split(',')
        if len(parts)!=5: continue
        r,p,y,clk,enc = parts
        try:
            r = float(r); p = float(p); y = float(y)
            clk = bool(int(clk)); enc = int(enc)
        except:
            continue

        # move cursor
        pyautogui.moveRel(r*2, -p*2, duration=0)
        if clk: pyautogui.click()

        # encoderclk
        if enc>0:
            for _ in range(enc): pyautogui.press('W')
        elif enc<0:
            for _ in range(-enc): pyautogui.press('S')

if __name__=='__main__':
    # abra a porta BT (ou USB, se preferir)
    ser_bt  = serial.Serial(BT_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    t = threading.Thread(target=reader, args=(ser_bt,), daemon=True)
    t.start()
    print("Leitura via Bluetooth iniciada. Ctrl+C para sair.")
    try:
        while True: time.sleep(0.1)
    except KeyboardInterrupt:
        ser_bt.close()
        print("Encerrando...")