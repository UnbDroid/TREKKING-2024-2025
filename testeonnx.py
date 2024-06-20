from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO("/home/caldo/Documents/Droid/TREEKING2K24/best.pt")

# Export the model to ONNX format
model.export(format="onnx")  # creates 'yolov8n.onnx'

# Load the exported ONNX model
onnx_model = YOLO("best.onnx")

# When predicting, set the 'imgsz' parameter to 224x224
results = model.predict(source='/home/caldo/Documents/Droid/TREEKING2K24/bus.jpg', imgsz=(224, 224))