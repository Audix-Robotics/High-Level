# Raspberry Pi SSH Setup

Use this guide if you want to clone the `final-vision` branch on the Raspberry Pi over SSH.

## Repo and branch

Repository:

```text
git@github.com:Audix-Robotics/High-Level.git
```

Branch:

```text
final-vision
```

## 1. Generate an SSH key on the Pi

Run:

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
```

Press Enter to accept the default path:

```text
/home/pi/.ssh/id_ed25519
```

You can set a passphrase or leave it empty.

## 2. Copy the public key

Run:

```bash
cat ~/.ssh/id_ed25519.pub
```

Copy the full output.

## 3. Add the key to GitHub

In GitHub:

1. Open `Settings`
2. Open `SSH and GPG keys`
3. Click `New SSH key`
4. Give it a name like `raspberry-pi`
5. Paste the public key
6. Save

## 4. Test SSH

Run:

```bash
ssh -T git@github.com
```

The first time, type `yes` when GitHub asks to trust the host.

If it works, GitHub should confirm authentication.

## 5. Clone this branch

Run:

```bash
git clone --branch final-vision git@github.com:Audix-Robotics/High-Level.git
cd High-Level
```

## 6. Set up Python and dependencies

Run:

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## 7. Run the app

Run:

```bash
source .venv/bin/activate
python app.py --profile indomie
```

You can also use:

```bash
python app.py --profile beans_can
python app.py --profile fruit_rings_cereal
```

## Notes

- If the default camera does not open, try `--camera-index 1`.
- If you connect to the Pi using plain SSH without GUI forwarding, the OpenCV preview window will not appear.
- For camera preview, use the Pi desktop, VNC, or SSH with X forwarding.
