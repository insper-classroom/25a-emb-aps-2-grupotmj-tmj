# import serial
# import pyautogui
# import time

# # Configure a porta serial – ajuste conforme necessário:
# # Em Mac, pode ser algo como '/dev/tty.usbserial-XXXXXXXX' ou se usar um adaptador Bluetooth virtual, algo como '/dev/rfcomm0'
# porta = '/dev/tty.usbmodem102'  # Substitua pelo nome da sua porta
# baud_rate = 115200

# try:
#     ser = serial.Serial(porta, baud_rate, timeout=1)
# except Exception as e:
#     print("Erro ao abrir a porta serial:", e)
#     exit(1)


# time.sleep(2)  # Aguarda a estabilização da conexão

# print("Conectado. Iniciando controle...")
# while True:
#     try:
#         linha = ser.readline().decode('utf-8').strip()
#         if linha.startswith("ANALOG,"):
#             partes = linha.split(',')
#             if len(partes) == 3:
#                 try:
#                     x = int(partes[1])
#                     y = int(partes[2])
#                     # Movimenta o mouse de forma relativa conforme os valores lidos.
#                     # Ajuste a velocidade (duration) se necessário.
#                     pyautogui.moveRel(x, y, duration=0.05)
#                 except ValueError:
#                     print("Erro ao converter os valores:", linha)
#     except Exception as e:
#         print("Erro na leitura:", e)

import serial
import pyautogui
import time
import re

# Configure a porta serial conforme sua configuração:
porta = '/dev/tty.usbmodem102'  # Atualize conforme necessário
baud_rate = 115200

try:
    ser = serial.Serial(porta, baud_rate, timeout=0.5)
except Exception as e:
    print("Erro ao abrir a porta serial:", e)
    exit(1)

time.sleep(2)
print("Conectado à porta serial. Aguardando dados...")

# Expressão regular para capturar mensagens do tipo "ANALOG,x,y"
pattern = re.compile(r'ANALOG,(-?\d+),(-?\d+)')
buffer = ""

while True:
    try:
        # Lê os dados disponíveis e adiciona ao buffer
        dados = ser.read(ser.in_waiting or 1).decode('utf-8', errors='replace')
        if dados:
            buffer += dados
            # Processa todas as linhas completas no buffer
            while "\n" in buffer:
                linha, buffer = buffer.split("\n", 1)
                linha = linha.strip()
                if linha:
                    # Procura pela mensagem com a regex
                    match = pattern.search(linha)
                    if match:
                        x_val = int(match.group(1))
                        y_val = int(match.group(2))
                        print(f"Recebido: x={x_val}, y={y_val}")
                        # Aqui você pode chamar pyautogui.moveRel(x_val, y_val) ou outra ação
                        # Exemplo:
                        pyautogui.moveRel(x_val, y_val, duration=0.05)
                    else:
                        # Caso a linha não contenha o padrão esperado, pode imprimir para depuração
                        print("Linha inválida:", linha)
        else:
            time.sleep(0.1)
    except serial.SerialException as e:
        print("Erro na leitura:", e)
        break