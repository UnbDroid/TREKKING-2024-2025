from ultralytics import YOLO


def main():

    model = YOLO("yolov8n.yaml")  # build a new model from scratch

    model.train(data="config.yaml", epochs=30, amp=False)  # train the model
    metrics = model.val()  # evaluate model performance on the validation set


if __name__ == "__main__":
    # freeze_support()
    main()
