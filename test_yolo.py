from ultralytics import YOLO

# Load a pretrained YOLO model
# model = YOLO("yolo11n.pt")
model = YOLO("yolo_button.pt")

# Perform object detection on an image
# results = model("bus.jpg")
# results = model("button1.jpg")
results = model("button2.jpg")



# Visualize the results
for result in results:
    result.show()