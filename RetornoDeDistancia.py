import cv2
from ultralytics import YOLO
from collections import defaultdict
import numpy as np
import serial
import sys

# * Explicação do código, aqui vamos identificar um cone, com o modelo treinado.
# * Vanos identificar a posição do objeto em relação ao centro da tela.

cap = cv2.VideoCapture(2)
arduino = serial.Serial('/dev/ttyACM0',115200)
# Carregue o modelo YOLO
model = YOLO("/home/caldo/Documents/Droid/TREEKING2K24/V1.pt")

# Dicionário para rastrear IDs e histórico de posições
track_history = defaultdict(lambda: [])

# Variáveis de controle
seguir = True
deixar_rastro = True

while True:
    success, img = cap.read()
    if success:
        # Processamento com YOLO
        if seguir:
            results = model.track(
                img,
                persist=True,
                conf=0.35,
                imgsz=96,
                iou=0.3,
                max_det=2,
                stream_buffer=True,
                batch=-1,
                rect=True,
                # plots=True, # teste para plotar num gráfico a probabilidade de achar um cone :)
            )
        else:
            results = model(img)

        # Processamento dos resultados
        for result in results:
            # Visualização dos resultados na imagem
            #img = result.plot()

            # Rastreamento e estimativa de distância
            if seguir and deixar_rastro:
                try:
                    # Obter caixas
                    boxes = result.boxes.xywh.cpu()
                    track_id=[1]
                    # Traçar rastreamento
                    for box, track_id in zip(boxes,track_id):
                        x, y, w, h = box

                        # Calcular a posição do objeto em relação ao centro da tela
                        center_x = (x + w / 2) / img.shape[1]
                        center_y = (y + h / 2) / img.shape[0]
                        offset_x = (
                            center_x - 0.68
                        )  # Centro da tela na coordenada x é 0.68
                        offset_y = (
                            center_y - 0.8
                        )  # Centro da tela na coordenada y é 0.8

                        print(
                            f"Posição do objeto em relação ao centro da tela: (x={offset_x:.2f}, y={offset_y:.2f})"
                        )
                    sys.stdout.flush()
                    arduino.write(f'{offset_x:.2f},{offset_y:.2f}\n'.encode('utf-8'))
                except Exception as e:
                    print(f"Erro no rastreamento: {e}")
                    sys.stdout.flush()

    # Exibir imagem com resultados
    cv2.imshow("Tela", img)

    # Tecla 'q' para sair
    k = cv2.waitKey(1)
    if k == ord("q"):
        break

# Liberar recursos
cap.release()
cv2.destroyAllWindows()
arduino.close()
print("Desligando...")
