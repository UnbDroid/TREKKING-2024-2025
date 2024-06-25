import cv2
import numpy as np
import ncnn
import serial
import sys
from collections import defaultdict

# Replace these with appropriate paths
param_path = "/home/caldo/Documents/Droid/TREEKING2K24/V1_ncnn_model/model.ncnn.param"
bin_path = "/home/caldo/Documents/Droid/TREEKING2K24/V1_ncnn_model/model.ncnn.bin"

# Initialize the NCNN model
net = ncnn.Net()
net.load_param(param_path)
net.load_model(bin_path)

# Initialize the video capture
cap = cv2.VideoCapture(0)

# Serial communication setup (uncomment for actual use)
# arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1, dsrdtr=True)

# Camera parameters
focal_length = 640
known_object_width = 10

# Dictionary to track IDs and position history
track_history = defaultdict(lambda: [])

# Control variables
seguir = True
deixar_rastro = True

while True:
    success, img = cap.read()
    if success:
        # Prepare the image for NCNN input
        input_img = cv2.resize(img, (128, 128))
        input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)
        input_img = input_img.astype(np.float32)
        input_img = input_img / 255.0
        input_img = input_img.transpose(2, 0, 1)
        input_img = np.expand_dims(input_img, axis=0)

        # Run inference with NCNN
        blob = ncnn.Mat(input_img)
        ex = net.create_extractor()
        ex.input("input", blob)
        ret, output = ex.extract("output")

        # Process the results
        if seguir:
            # Assuming the output is the bounding boxes, class ids, and scores
            # Adjust this part according to your model's output format
            boxes = output.reshape(-1, 6)
            for box in boxes:
                x, y, w, h, conf, cls_id = box
                if conf > 0.80:  # Confidence threshold
                    track_id = int(cls_id)  # Example of using class ID as track ID
                    x_center = x + w / 2
                    y_center = y + h / 2
                    track_history[track_id].append((x_center, y_center))

                    if len(track_history[track_id]) > 4:
                        track_history[track_id].pop(0)

                    # Estimate distance
                    distance = (known_object_width * focal_length) / w
                    print(f"Distância estimada para o objeto: {distance:.2f} cm")

                    center_x = x_center / img.shape[1]
                    center_y = y_center / img.shape[0]
                    offset_x = center_x - 0.5
                    offset_y = center_y - 0.5

                    print(f"Posição do objeto em relação ao centro da tela: (x={offset_x:.2f}, distance={float(distance):.2f})")

                    # Send position to Arduino
                    sys.stdout.flush()
                    print("estou indo mandar")
                    # arduino.write(f"{offset_x:.2f},{distance:.2f}\n".encode('utf-8'))
                    print("mandei via serial")

        # Display the image with results
        for box in boxes:
            x, y, w, h, conf, cls_id = box
            if conf > 0.80:
                cv2.rectangle(img, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 0), 2)
                cv2.putText(img, f"ID: {int(cls_id)}", (int(x), int(y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        cv2.imshow("Tela", img)

    # Key press to exit
    k = cv2.waitKey(1)
    if k == ord("q"):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
# arduino.close()
print("Desligando...")
