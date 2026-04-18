# Supermarket Trigger Inspector

Small trigger-only webcam app for your trained supermarket YOLOv8 model.

## What is inside

- `app.py`: the trigger inspector
- `models/best.pt`: your trained YOLOv8 model
- `requirements.txt`: Python dependency list

The app keeps a live camera preview open, waits for keyboard commands in the terminal, and inspects the current frame only when you type `x` and press Enter.

## Supported product profiles

- `indomie`
- `beans_can`
- `fruit_rings_cereal`

Each profile checks:

- the expected product for that shelf
- misplaced products
- whether the shelf is `understocked`, `balanced`, or `overstocked`

## Recommended Raspberry Pi setup

- Raspberry Pi OS 64-bit
- Python 3.11 or newer
- USB webcam
- Desktop session, VNC session, or SSH with X forwarding if you want to see the OpenCV windows

Important:

- Plain headless SSH without GUI forwarding will not show the live camera windows.
- Cloning with SSH is fine, but you should run the app from the Pi desktop terminal or a GUI-capable remote session if you want the preview window.

## Clone option 1: GitHub interface / HTTPS

On the repository page:

1. Click `Code`.
2. Copy the `HTTPS` URL.

On the Raspberry Pi:

```bash
git clone https://github.com/YOUR-USERNAME/YOUR-REPO.git
cd YOUR-REPO/supermarket_trigger_app
```

If you prefer GitHub Desktop on another computer, GitHub's docs describe `Code` -> `Open with GitHub Desktop`.

## Clone option 2: SSH

Make sure your Raspberry Pi SSH key is already added to GitHub.

Then run:

```bash
git clone git@github.com:YOUR-USERNAME/YOUR-REPO.git
cd YOUR-REPO/supermarket_trigger_app
```

GitHub says SSH clone URLs require adding your public SSH key to your account first.

## Raspberry Pi setup

From inside `supermarket_trigger_app`:

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## Run the app

Default camera:

```bash
source .venv/bin/activate
python app.py --profile indomie
```

Another product profile:

```bash
python app.py --profile beans_can
python app.py --profile fruit_rings_cereal
```

Another camera index:

```bash
python app.py --profile indomie --camera-index 1
```

Custom confidence or IoU:

```bash
python app.py --profile indomie --conf 0.30 --iou 0.60
```

## How to use it

1. Start the script.
2. The live camera preview opens.
3. Type `x` and press Enter to inspect the current frame.
4. Type `q` and press Enter to quit.
5. You can also press `q` in the preview window to close it.

## Expected behavior

When you trigger a capture, the app:

1. runs YOLO detection on the current frame
2. prints each detected item in the terminal
3. counts expected vs misplaced products
4. reports `understocked`, `balanced`, or `overstocked`
5. opens an annotated capture window with boxes and labels

## Troubleshooting

If the camera does not open:

```bash
python app.py --profile indomie --camera-index 1
```

If the window does not appear:

- run it from the Raspberry Pi desktop terminal
- or use VNC
- or use SSH with X forwarding configured

If model loading fails, make sure this file exists:

```bash
models/best.pt
```

If `pip install -r requirements.txt` fails on the Pi, verify that you are using a 64-bit Raspberry Pi OS image and a supported Python version.

## Repository notes

This folder is meant to be the clean deployable app folder.
It intentionally does not include training datasets, experiment runs, or local virtual environments.

## References

- Ultralytics docs: `pip install -U ultralytics` and Python inference usage
  https://docs.ultralytics.com/
- GitHub docs: HTTPS and SSH clone URLs
  https://docs.github.com/en/get-started/git-basics/about-remote-repositories
- GitHub docs: `Code` -> `Open with GitHub Desktop`
  https://docs.github.com/en/enterprise-server%403.14/desktop/adding-and-cloning-repositories/cloning-a-repository-from-github-to-github-desktop
