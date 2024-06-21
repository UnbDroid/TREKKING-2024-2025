import cv2
import serial
import sys
from ultralytics import YOLO
from collections import defaultdict
import numpy as np
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

# Coisas da Comunicação Serial --------------------------------------------------
# arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1, dsrdtr=True)
# -------------------------------------------------------------------------------

# Parâmetros da câmera (preencha com os valores da sua câmera)
focal_length = 640  # Substitua com a distância focal da sua câmera (em pixels)
known_object_width = 10  # Substitua com a largura real do objeto em centímetros (ou outra unidade)

cap = cv2.VideoCapture(0)

# Carregue o modelo YOLO
model = YOLO("/home/caldo/Documents/Droid/TREEKING2K24/V1.pt")

# Dicionário para rastrear IDs e histórico de posições
track_history = defaultdict(lambda: [])

# Variáveis de controle
seguir = True
deixar_rastro = True

sys.stdout.flush()

# arduino.write(f"{0.00},{0.00}\n".encode('utf-8'))

def process_frame(img):
    results = model.track(img, persist=True, conf=0.75, imgsz=128, iou=0.3, max_det=2, stream_buffer=True, save_txt=False) if seguir else model(img)
    for result in results:
        img = result.plot()
        if seguir and deixar_rastro:
            try:
                boxes = result.boxes.xywh.cpu()
                track_ids = result.boxes.id.int().cpu().tolist()
                for box, track_id in zip(boxes, track_ids):
                    x, y, w, h = box
                    track = track_history[track_id]
                    track.append((float(x), float(y)))
                    if len(track) > 4:
                        track.pop(0)
                    distance = (known_object_width * focal_length) / w
                    print(f"Distância estimada para o objeto: {distance:.2f} cm")
                    center_x = (x) / img.shape[1]
                    center_y = (y) / img.shape[0]
                    offset_x = (center_x - 0.5)
                    offset_y = (center_y - 0.5)
                    print(f"Posição do objeto em relação ao centro da tela: (x={offset_x:.2f}, distance={float(distance):.2f})")
                    sys.stdout.flush()
                    # arduino.write(f"{offset_x:.2f},{distance:.2f}\n".encode('utf-8'))
            except Exception as e:
                print(f"Erro no rastreamento: {e}")
                sys.stdout.flush()
    return img

with ThreadPoolExecutor(max_workers=4) as executor:
    futures = []
    while True:
        success, img = cap.read()
        if success:
            future = executor.submit(process_frame, img)
            futures.append(future)
            for future in as_completed(futures):
                img = future.result()
                cv2.imshow("Tela", img)
                futures.remove(future)
        k = cv2.waitKey(1)
        if k == ord("q"):
            break

cap.release()
cv2.destroyAllWindows()
# arduino.close()
print("Desligando...")
