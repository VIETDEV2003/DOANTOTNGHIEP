from ultralytics import YOLO
model = YOLO("runs/detect/contrung_model/weights/best.pt")
results = model.predict(source="test.jpg")  
