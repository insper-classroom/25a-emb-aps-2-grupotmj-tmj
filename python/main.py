import serial
import pyautogui

def process_packet(packet):
    """
    Processa um pacote conforme o header:
      - Se header for 0x00, é um evento de joystick (6 bytes):
          [0x00, msb(x), lsb(x), msb(y), lsb(y), EOP]
      - Se header for 0x01, é um evento de botão (3 bytes):
          [0x01, key, EOP]
    """
    header = packet[0]
    if header == 0x00:
        if len(packet) != 6 or packet[5] != 0xFF:
            print("Pacote de joystick incorreto:", packet)
            return 6  # descarta 6 bytes ou tenta realinhar
        # Extrai e converte os valores de X e Y (big endian, com sinal)
        x = int.from_bytes(packet[1:3], byteorder='big', signed=True)
        y = int.from_bytes(packet[3:5], byteorder='big', signed=True)
        print(f"Recebido Joystick - X: {x}, Y: {y}")
        # Move o mouse de forma integrada
        pyautogui.moveRel(x, y)
        return 6
    elif header == 0x01:
        if len(packet) != 3 or packet[2] != 0xFF:
            print("Pacote de botão incorreto:", packet)
            return 3
        key = chr(packet[1])
        print(f"Recebido Botão - Key: {key}")
        # Simula o pressionamento da tecla (pyautogui.press aceita string com a tecla)
        pyautogui.press(key.lower())
        return 3
    else:
        print("Header desconhecido:", header)
        return 1

def main():
    # Atualize a porta conforme o ambiente (ex.: '/dev/tty.usbmodem102' ou 'COM3')
    ser = serial.Serial('/dev/tty.usbmodem102', 115200, timeout=0.1)
    print("Iniciando a leitura da porta serial...")
    
    buffer = bytearray()
    while True:
        bytes_available = ser.in_waiting or 1
        data = ser.read(bytes_available)
        if data:
            buffer.extend(data)
        # Verifica se há dados no buffer e tenta processar pacotes
        while buffer:
            # O header é o primeiro byte
            header = buffer[0]
            if header == 0x00:
                # Evento de joystick: precisa ter 6 bytes
                if len(buffer) < 6:
                    break
                if buffer[5] == 0xFF:
                    packet = buffer[:6]
                    consumed = process_packet(packet)
                    buffer = buffer[consumed:]
                else:
                    buffer.pop(0)
            elif header == 0x01:
                # Evento de botão: pacote de 3 bytes
                if len(buffer) < 3:
                    break
                if buffer[2] == 0xFF:
                    packet = buffer[:3]
                    consumed = process_packet(packet)
                    buffer = buffer[consumed:]
                else:
                    buffer.pop(0)
            else:
                buffer.pop(0)

if __name__ == '__main__':
    main()