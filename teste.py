import serial

ser = serial.Serial('/dev/ttyUSB0', 9600)

teste_python = 1

while True:
    if ser.in_waiting > 0:
        recebido = ser.readline().decode('utf-8')
        teste_python = recebido + 1
    
    ser.write(teste_python)
    print(teste_python)