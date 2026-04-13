import argparse
import cv2
from ultralytics import YOLO

MODEL = "models/best.pt"
CONF = 0.9
IOU = 0.6
TARGET = 2
RULES = {
    "shelf1": {"expected": "red box", "wrong": "can", "min": {"red box": 0.9, "can": 0.8}},
    "shelf2": {"expected": "can", "wrong": "red box", "min": {"red box": 0.9, "can": 0.8}},
}


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--shelf", choices=["shelf1", "shelf2"], default="shelf1")
    parser.add_argument("--camera-index", type=int, default=0)
    return parser.parse_args()


def stock_state(count):
    if count < TARGET:
        return "understocked"
    if count > TARGET:
        return "overstocked"
    return "balanced"


def detect(model, frame, shelf):
    result = model.predict(frame, conf=CONF, iou=IOU, verbose=False)[0]
    items = []
    if result.boxes is None:
        return items
    for i in range(len(result.boxes)):
        name = model.names[int(result.boxes.cls[i])]
        conf = float(result.boxes.conf[i])
        x1, y1, x2, y2 = [int(v) for v in result.boxes.xyxy[i].tolist()]
        ok = conf >= RULES[shelf]["min"].get(name, CONF)
        items.append((name, conf, ok, (x1, y1, x2, y2)))
    return items


def draw(frame, items):
    for name, conf, _, (x1, y1, x2, y2) in items:
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            frame,
            f"{name} {conf:.2f}",
            (x1, max(20, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
    return frame


def report(items, shelf):
    expected = RULES[shelf]["expected"]
    wrong = RULES[shelf]["wrong"]
    accepted = [(name, conf) for name, conf, ok, _ in items if ok]
    good = sum(1 for name, _ in accepted if name == expected)
    bad = sum(1 for name, _ in accepted if name == wrong)

    if not items:
        print("Vision result: no detections")
    else:
        for i, (name, conf, ok, _) in enumerate(items, 1):
            print(f"{i}. {name} | confidence={conf:.2f} | {'accepted' if ok else 'rejected'}")
    print(f"{shelf}: {expected}={good} {stock_state(good)}, misplaced {wrong}={bad}")


def main():
    args = get_args()
    model = YOLO(MODEL)
    camera = cv2.VideoCapture(args.camera_index)
    if not camera.isOpened():
        raise RuntimeError(f"Cannot open camera index {args.camera_index}")

    print("Press q to quit")
    while True:
        ok, frame = camera.read()
        if not ok:
            break
        items = detect(model, frame, args.shelf)
        report(items, args.shelf)
        cv2.imshow("vision", draw(frame, items))
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    camera.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
