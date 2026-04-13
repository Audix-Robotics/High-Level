# Vision Branch for Raspberry Pi

This branch only contains the files needed to run the shelf vision model on the Raspberry Pi.

## Clone

```bash
git clone --branch vision --single-branch git@github.com:Audix-Robotics/High-Level.git
cd High-Level
```

## Files

- `vision.py` runs live camera detection
- `models/best.pt` is the trained YOLO model
- `requirements.txt` installs the Python packages

## Setup on the Pi

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
python3 vision.py --shelf shelf1
```

Or:

```bash
python3 vision.py --shelf shelf2 --camera-index 0
```

The terminal prints the detections and confidence.
The camera window shows only the object boxes and confidence labels.
