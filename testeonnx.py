from ultralytics import YOLO

# Load a YOLOv8n PyTorch model
model = YOLO("V1.pt")

# Export the model to NCNN format
model.export(format="ncnn")  # creates 'yolov8n_ncnn_model'

# Load the exported NCNN model
ncnn_model = YOLO("./V1_ncnn_model")

# Run inference
results = ncnn_model("https://ultralytics.com/images/bus.jpg")