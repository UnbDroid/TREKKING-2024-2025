import cv2
import serial
import torch
import onnxruntime
import sys
from ultralytics import YOLO
from collections import defaultdict
import numpy as np
import time

#! Coisas da Comunicação Serial --------------------------------------------------

arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1, dsrdtr=True)

#! -------------------------------------------------------------------------------


# Parâmetros da câmera (preencha com os valores da sua câmera)
focal_length = 136  # Substitua com a distância focal da sua câmera (em pixels)
known_object_width = (
    28  # Substitua com a largura real do objeto em centímetros (ou outra unidade)
)

task = "track"  # Tarefa de rastreamento

sess_options = onnxruntime.SessionOptions()

sess_options.intra_op_num_threads = 0

sess = onnxruntime.InferenceSession("V6_128.onnx", sess_options)

cap = cv2.VideoCapture(2)

#132

# cap.open(2+cv2.CAP_DSHOW)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 128)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 128) 
# cap.set(cv2.CAP_PROP_FOCUS,60)
# time.sleep(.1)

# Carregue o modelo YOLO
model = YOLO("V6_128.onnx", task='detect')

# Dicionário para rastrear IDs e histórico de posições
track_history = defaultdict(lambda: [])

# def angulo(x,y):
#     cateto_oposto = x*100
#     cateto_adjacente = y
#     hipotenusa = np.sqrt(cateto_oposto**2 + cateto_adjacente**2)
#     angulo = np.arcsin(cateto_oposto/hipotenusa)
#     return angulo

# Variáveis de controle
seguir = True
deixar_rastro = True

# sys.stdout.flush()

# arduino.write(f"{0.00},{0.00}\n".encode('utf-8'))

while True:
    # Capture a imagem do vídeo
    success, img = cap.read()
    img = cv2.resize(img, (128, 128))
    # print("Li a camera")
    # img = cv2.resize(img, (128, 128))

    if success:
        # Processamento com YOLO
        if seguir:
            results = model.track(
                img,
                # verbose=False,
                persist=True,
                conf=0.8,
                imgsz=128,
                iou=0.3,
                max_det=2,
                stream_buffer=True,
                save_txt=False,
                stream=True,
            )
        else:
            results = model(img) #, verbose=False)
            
        # print("Fiz o processamento")

        # Processamento dos resultados
        for result in results:
            # Visualização dos resultados na imagem
            img = result.plot()
            # print("Fiz o plot")

            # Rastreamento e estimativa de distância
            if seguir and deixar_rastro:
                try:
                    # Obter caixas e IDs de rastreamento
                    boxes = result.boxes.xywh.cpu()
                    track_ids = result.boxes.id.int().cpu().tolist()
                    print("Obtive as caixas e IDs")

                    # Traçar rastreamento
                    for box, track_id in zip(boxes, track_ids):
                        x, y, w, h = box
                        track = track_history[track_id]
                        track.append((float(x), float(y)))  # Ponto central x, y
                        # Limitar histórico de rastreamento
                        if len(track) > 4:
                            track.pop(0)
                            
                        # print("Fiz o rastreamento")
                        
                        # Estimar distância usando largura do objeto e razão de triângulos similares
                        distance = (known_object_width * focal_length) / w
                        # distance -= 17
                        # print(f"Distância estimada para o objeto: {distance:.2f} cm")

                        # Calcular a posição do objeto em relação ao centro da tela
                        center_x = (x)/ img.shape[1]
                        
                        offset_x = (
                            center_x - 0.5
                        )  # Centro da tela na coordenada x é 0.68
                         # Centro da tela na coordenada y é 0.8
                        # angulo_cone = angulo(offset_x,distance)
                        # print(
                            # f"Posição do objeto em relação ao centro da tela: (x={offset_x:.2f}, distance={float(distance):.2f})"
                        # )
                        
                        print(f"Posição do objeto em relação ao centro da tela: (x={offset_x:.2f}, distance={float(distance):.2f})")
                        
                        
                        sys.stdout.flush()
                        
                        # print("Estou indo mandar")
                        
                        # Enviar posição do objeto para o Arduino via serial
                        arduino.write(f"{offset_x:.2f},{distance:.2f}\n".encode('utf-8'))
                        # print("Mandei via serial")

                except Exception as e:
                    sys.stdout.flush()
                    print(f"Erro no rastreamento: {e}")
                    arduino.write("Nada\n".encode('utf-8'))

    # Exibir imagem com resultados
    # img = cv2.resize(img,(96,96))
    # cv2.imshow("Tela", img)

    print("Exibi a tela")

    # Tecla 'q' para sair
    k = cv2.waitKey(1)
    if k == ord("q"):
        break

# Liberar recursos
cap.release()
cv2.destroyAllWindows()
arduino.close()  # Fecha a comunicação serial
print("Desligando...")