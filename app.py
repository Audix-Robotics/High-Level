import argparse
import queue
import threading
from pathlib import Path

import cv2
from ultralytics import YOLO

BASE_DIR = Path(__file__).resolve().parent
MODEL_PATH = BASE_DIR / "models" / "best.pt"
DEFAULT_CONF = 0.25
DEFAULT_IOU = 0.60

PROFILES = {
    "indomie": {
        "expected": "Indomie",
        "wrong": ["beans_can", "fruit_rings_cereal"],
        "target": 2,
        "min": {"Indomie": 0.25, "beans_can": 0.25, "fruit_rings_cereal": 0.25},
    },
    "beans_can": {
        "expected": "beans_can",
        "wrong": ["Indomie", "fruit_rings_cereal"],
        "target": 2,
        "min": {"Indomie": 0.25, "beans_can": 0.25, "fruit_rings_cereal": 0.25},
    },
    "fruit_rings_cereal": {
        "expected": "fruit_rings_cereal",
        "wrong": ["Indomie", "beans_can"],
        "target": 2,
        "min": {"Indomie": 0.25, "beans_can": 0.25, "fruit_rings_cereal": 0.25},
    },
}


def get_args():
    parser = argparse.ArgumentParser(
        description="Trigger-only shelf inspector for the supermarket YOLOv8 model."
    )
    parser.add_argument("--profile", choices=sorted(PROFILES), default="indomie")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--model", type=Path, default=MODEL_PATH)
    parser.add_argument("--conf", type=float, default=DEFAULT_CONF)
    parser.add_argument("--iou", type=float, default=DEFAULT_IOU)
    return parser.parse_args()


def stock_state(count, target):
    if count < target:
        return "understocked"
    if count > target:
        return "overstocked"
    return "balanced"


def validate_profile(model, profile_name):
    names = set(model.names.values()) if isinstance(model.names, dict) else set(model.names)
    profile = PROFILES[profile_name]
    required = {profile["expected"], *profile["wrong"]}
    missing = sorted(required - names)
    if missing:
        raise ValueError(
            f"Profile '{profile_name}' references classes not found in the model: {', '.join(missing)}"
        )


def detect(model, frame, profile_name, conf, iou):
    profile = PROFILES[profile_name]
    result = model.predict(frame, conf=conf, iou=iou, verbose=False)[0]
    items = []

    if result.boxes is None:
        return items

    for i in range(len(result.boxes)):
        name = model.names[int(result.boxes.cls[i])]
        score = float(result.boxes.conf[i])
        x1, y1, x2, y2 = [int(v) for v in result.boxes.xyxy[i].tolist()]
        accepted = score >= profile["min"].get(name, conf)
        items.append((name, score, accepted, (x1, y1, x2, y2)))

    return items


def draw_preview(frame, profile_name):
    preview = frame.copy()
    cv2.putText(
        preview,
        f"Profile: {profile_name}",
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
    )
    cv2.putText(
        preview,
        "Type x + Enter to inspect | q + Enter to quit",
        (15, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )
    return preview


def draw_capture(frame, items, profile_name):
    profile = PROFILES[profile_name]
    expected = profile["expected"]

    for name, score, accepted, (x1, y1, x2, y2) in items:
        if not accepted:
            color = (0, 0, 255)
            label = "rejected"
        elif name == expected:
            color = (0, 255, 0)
            label = "expected"
        else:
            color = (0, 165, 255)
            label = "misplaced"

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            frame,
            f"{name} {score:.2f} {label}",
            (x1, max(20, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )

    return frame


def report(items, profile_name):
    profile = PROFILES[profile_name]
    expected = profile["expected"]
    wrong = set(profile["wrong"])
    target = profile["target"]

    accepted = [(name, score) for name, score, ok, _ in items if ok]
    expected_count = sum(1 for name, _ in accepted if name == expected)
    misplaced_count = sum(1 for name, _ in accepted if name in wrong)

    print(f"\nProfile: {profile_name}")
    print(f"Expected item: {expected} | Target count: {target}")

    if not items:
        print("Vision result: no detections")
    else:
        for i, (name, score, ok, _) in enumerate(items, 1):
            if not ok:
                verdict = "rejected"
            elif name == expected:
                verdict = "accepted-expected"
            elif name in wrong:
                verdict = "accepted-misplaced"
            else:
                verdict = "accepted-other"
            print(f"{i}. {name} | confidence={score:.2f} | {verdict}")

    print(f"Stock status: {expected_count} expected item(s) -> {stock_state(expected_count, target)}")
    print(f"Misplaced items: {misplaced_count}")


def run_trigger(model, profile_name, camera_index, conf, iou):
    camera = cv2.VideoCapture(camera_index)
    if not camera.isOpened():
        raise RuntimeError(f"Cannot open camera index {camera_index}")

    print("Triggered inspection mode")
    print("Live preview stays open while waiting")
    print("Type x then Enter to inspect the current frame")
    print("Type q then Enter to quit")

    commands = queue.Queue()
    stop_event = threading.Event()

    def read_commands():
        while not stop_event.is_set():
            try:
                command = input("\nCommand [x=inspect, q=quit]: ").strip().lower()
            except EOFError:
                command = "q"
            commands.put(command)
            if command == "q":
                break

    input_thread = threading.Thread(target=read_commands, daemon=True)
    input_thread.start()

    capture_id = 0
    last_frame = None
    while True:
        ok, frame = camera.read()
        if not ok:
            break

        last_frame = frame.copy()
        cv2.imshow("trigger_preview", draw_preview(frame, profile_name))

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        try:
            command = commands.get_nowait()
        except queue.Empty:
            continue

        if command == "q":
            break
        if command != "x":
            print("Unknown command. Type x to inspect or q to quit.")
            continue
        if last_frame is None:
            print("No camera frame available yet.")
            continue

        capture_id += 1
        items = detect(model, last_frame, profile_name, conf, iou)
        print(f"\nCapture #{capture_id}")
        report(items, profile_name)
        cv2.imshow("trigger_capture", draw_capture(last_frame.copy(), items, profile_name))
        cv2.waitKey(1)

    stop_event.set()
    camera.release()
    cv2.destroyAllWindows()


def main():
    args = get_args()
    model = YOLO(str(args.model))
    validate_profile(model, args.profile)
    run_trigger(model, args.profile, args.camera_index, args.conf, args.iou)


if __name__ == "__main__":
    main()
