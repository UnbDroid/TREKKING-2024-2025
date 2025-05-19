import serial
import re
import matplotlib.pyplot as plt
from collections import deque

# Configurações da porta serial
SERIAL_PORT = '/dev/ttyUSB0'  # Altere para a porta correta (ex: '/dev/ttyUSB0' no Linux)
BAUD_RATE = 115200

# Expressão regular para extrair os valores de velocity e erro
pattern = re.compile(r'RPM: ([\d\.\-]+), Posi: ([\d\.\-]+) Target ([\d\.\-]+)')

# Filas para armazenar os dados
max_len = 100
velocity_data = deque(maxlen=max_len)
erro_data = deque(maxlen=max_len)

# Inicia a serial
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Configura o gráfico
plt.ion()
fig, ax = plt.subplots()
line1, = ax.plot([], [], label='Velocity (RPM)')
line2, = ax.plot([], [], label='Target')
ax.set_ylim(-100, 100)  # Ajuste conforme seus dados
ax.set_xlim(0, max_len)
ax.legend()
ax.set_title("Leitura Serial - Velocity & Erro")

try:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        match = pattern.search(line)
        if match:
            velocity = float(match.group(1))
            erro = float(match.group(3))
            velocity_data.append(velocity)
            erro_data.append(erro)

            # Atualiza o gráfico
            line1.set_ydata(velocity_data)
            line2.set_ydata(erro_data)
            line1.set_xdata(range(len(velocity_data)))
            line2.set_xdata(range(len(erro_data)))
            ax.relim()
            ax.autoscale_view()
            plt.pause(0.01)

except KeyboardInterrupt:
    print("Finalizado pelo usuário.")
    ser.close()

