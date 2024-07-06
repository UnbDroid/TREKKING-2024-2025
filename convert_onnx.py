from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO("V6_128.pt")

# Export the model to NCNN format
model.export(format="ncnn")  # creates '/yolov8n_ncnn_model'

# Load the exported NCNN model
ncnn_model = YOLO("./V6_128_ncnn_model")