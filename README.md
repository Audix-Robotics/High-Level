# Vision Branch for Raspberry Pi

This branch only contains the files needed to run the shelf vision model on the Raspberry Pi.

## Download As ZIP

```bash
Download the `vision` branch as a ZIP file from GitHub, extract it, then open a terminal inside the extracted folder.
```

## Files

- `vision.py` runs live camera detection
- `models/best.pt` is the trained YOLO model
- `vision_onnx.py` runs the ONNX version with lighter Pi settings
- `models/best.onnx` is the ONNX version of the model
- `requirements.txt` installs the Python packages
- `requirements_onnx.txt` installs the ONNX packages

## Setup on the Pi

```bash
sudo apt update
sudo apt install -y python3-pip python3-venv python3.12-venv
rm -rf .venv
python3 -m venv .venv
source .venv/bin/activate
python3 -m ensurepip --upgrade
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

If you renamed or moved the repo folder, recreate `.venv` with the commands above before running the project.
`vision.py` loads the model relative to its own folder, so it still works after extracting a ZIP to a different directory name.

## Run

```bash
python3 vision.py --shelf shelf1
```

Or:

```bash
python3 vision.py --shelf shelf2 --camera-index 0
```

If camera `0` does not work, try:

```bash
python3 vision.py --shelf shelf1 --camera-index 1
```

The terminal prints the detections and confidence.
The camera window shows only the object boxes and confidence labels.

## ONNX Version

Use this version if you want a separate model format option for the Pi.

### Setup

```bash
sudo apt update
sudo apt install -y python3-pip python3-venv python3.12-venv
rm -rf .venv
python3 -m venv .venv
source .venv/bin/activate
python3 -m ensurepip --upgrade
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements_onnx.txt
```

### Run

```bash
python3 vision_onnx.py --shelf shelf1 --camera-index 0
```

If camera `0` does not work, try:

```bash
python3 vision_onnx.py --shelf shelf1 --camera-index 1
```

This version uses:
- `models/best.onnx`
- camera resolution `640x480`
- `imgsz=320`
- detection every second frame for better FPS on the Pi
