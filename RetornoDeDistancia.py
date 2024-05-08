from ultralytics import YOLO
import cv2
from collections import defaultdict
import numpy as np

# Parâmetros da câmera (preencha com os valores da sua câmera)
focal_length = 640  # Substitua com a distância focal da sua câmera (em pixels)
known_object_width = (
    10  # Substitua com a largura real do objeto em centímetros (ou outra unidade)
)

cap = cv2.VideoCapture(0)

# Carregue o modelo YOLO
model = YOLO("runs/detect/train16/weights/best.pt")

# Dicionário para rastrear IDs e histórico de posições
track_history = defaultdict(lambda: [])

# Variáveis de controle
seguir = True
deixar_rastro = True

while True:
    # Capture a imagem do vídeo
    success, img = cap.read()

    if success:
        # Processamento com YOLO
        if seguir:
            results = model.track(
                img,
                persist=True,
                conf=0.35,
                imgsz=640,
                iou=0.3,
                max_det=2,
                stream_buffer=True,
                save_txt=False,
            )
        else:
            results = model(img)

        # Processamento dos resultados
        for result in results:
            # Visualização dos resultados na imagem
            img = result.plot()

            # Rastreamento e estimativa de distância
            if seguir and deixar_rastro:
                try:
                    # Obter caixas e IDs de rastreamento
                    boxes = result.boxes.xywh.cpu()
                    track_ids = result.boxes.id.int().cpu().tolist()

                    # Traçar rastreamento
                    for box, track_id in zip(boxes, track_ids):
                        x, y, w, h = box
                        track = track_history[track_id]
                        track.append((float(x), float(y)))  # Ponto central x, y

                        # Limitar histórico de rastreamento
                        if len(track) > 4:
                            track.pop(0)

                        # Estimar distância usando largura do objeto e razão de triângulos similares
                        distance = (known_object_width * focal_length) / w
                        print(f"Distância estimada para o objeto: {distance:.2f} cm")

                except Exception as e:
                    print(f"Erro no rastreamento: {e}")

        # Exibir imagem com resultados
        cv2.imshow("Tela", img)

    # Tecla 'q' para sair
    k = cv2.waitKey(1)
    if k == ord("q"):
        break

# Liberar recursos
cap.release()
cv2.destroyAllWindows()
print("Desligando...")
