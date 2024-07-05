from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO("V6_128.pt")

# Export the model to ONNX format
model.export(format="onnx")  # creates 'yolov8n.onnx'

# Load the exported ONNX model
onnx_model = YOLO("V6_128.onnx")