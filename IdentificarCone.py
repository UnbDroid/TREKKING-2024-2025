from ultralytics import YOLO
import cv2
from collections import defaultdict
import numpy as np


cap = cv2.VideoCapture(0)
# ? usei esse site para me basear no model,track(https://docs.ultralytics.com/modes/predict/#inference-arguments)


#                           (ms)        (ms)
# YOLOv8n	640	    37.3	80.4	    0.99	    3.2	    8.7
# YOLOv8s	640	    44.9	128.4	    1.20	    11.2	28.6
# YOLOv8m	640	    50.2	234.7	    1.83	    25.9	78.9
# YOLOv8l	640	    52.9	375.2	    2.39	    43.7	165.2
# YOLOv8x	640	    53.9	479.1	    3.53	    68.2	257.8

model = YOLO("runs/detect/train16/weights/best.pt")

track_history = defaultdict(lambda: [])
seguir = True
deixar_rastro = False

while True:
    success, img = cap.read()

    if success:
        if seguir:
            results = model.track(
                img,
                persist=True,
                conf=0.35,
                imgsz=640,
                iou=0.3,
                max_det=2,
                stream_buffer=True,
                save_txt=False,  # seguindo o formato [class] [x_center] [y_center] [width] [height] [confidence]
            )
        else:
            results = model(img)

        # Process results list
        for result in results:
            # Visualize the results on the frame
            img = result.plot()

            if seguir and deixar_rastro:
                try:
                    # Get the boxes and track IDs
                    boxes = result.boxes.xywh.cpu()
                    track_ids = result.boxes.id.int().cpu().tolist()

                    # Plot the tracks
                    for box, track_id in zip(boxes, track_ids):
                        x, y, w, h = box
                        track = track_history[track_id]
                        track.append((float(x), float(y)))  # x, y center point
                        if len(track) > 30:  # retain 90 tracks for 90 frames
                            track.pop(0)

                        # Draw the tracking lines
                        points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
                        cv2.polylines(
                            img,
                            [points],
                            isClosed=False,
                            color=(230, 0, 0),
                            thickness=5,
                        )
                except:
                    pass

        cv2.imshow("Tela", img)

    k = cv2.waitKey(1)
    if k == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
print("desligando")
