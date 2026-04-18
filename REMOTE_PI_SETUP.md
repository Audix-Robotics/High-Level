# Remote Raspberry Pi Setup

Use this guide if you want to control the Raspberry Pi from your laptop and run the vision app remotely.

This is different from GitHub SSH.

- GitHub SSH: used for cloning the repository from GitHub to the Pi
- Raspberry Pi SSH: used for connecting from your laptop to the Pi terminal

## What this setup gives you

From your laptop, you will be able to:

- SSH into the Raspberry Pi
- start and stop the vision app
- pull code updates
- edit files from the terminal

Important:

- Plain SSH does not show the OpenCV camera window on your laptop.
- If you want to see the live camera preview, use VNC or Raspberry Pi Connect in addition to SSH.

## 1. Enable SSH on the Raspberry Pi

On the Pi, run:

```bash
sudo raspi-config
```

Then:

1. Open `Interface Options`
2. Open `SSH`
3. Choose `Yes`

You can verify SSH is enabled:

```bash
sudo systemctl status ssh
```

## 2. Find the Pi IP address

On the Pi, run:

```bash
hostname -I
```

Example:

```text
192.168.1.50
```

## 3. Connect from the Windows laptop

From Windows PowerShell:

```powershell
ssh your_pi_username@192.168.1.50
```

Example:

```powershell
ssh pi@192.168.1.50
```

## 4. Set up SSH keys from the laptop to the Pi

On the Windows laptop, create an SSH key if you do not already have one:

```powershell
ssh-keygen -t ed25519 -C "your_email@example.com"
```

Then send the public key to the Pi:

```powershell
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh your_pi_username@192.168.1.50 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"
```

After that, test login again:

```powershell
ssh your_pi_username@192.168.1.50
```

## 5. Clone the vision branch on the Pi

After logging into the Pi:

```bash
git clone --branch final-vision git@github.com:Audix-Robotics/High-Level.git
cd High-Level
```

If you prefer HTTPS:

```bash
git clone --branch final-vision https://github.com/Audix-Robotics/High-Level.git
cd High-Level
```

## 6. Install Python dependencies on the Pi

On the Pi:

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## 7. Run the vision app from your laptop through SSH

SSH into the Pi from the laptop:

```powershell
ssh your_pi_username@192.168.1.50
```

Then on the Pi:

```bash
cd High-Level
source .venv/bin/activate
python app.py --profile indomie
```

You can also run:

```bash
python app.py --profile beans_can
python app.py --profile fruit_rings_cereal
```

## 8. How to see the live camera window

If you run the script over plain SSH only:

- the app may run
- terminal output will appear
- but the OpenCV preview window will usually not show on your laptop

To see the live preview, use one of these:

- Raspberry Pi desktop directly
- VNC
- Raspberry Pi Connect
- SSH with X forwarding if your setup supports it

For most cases, VNC is the easiest option.

## 9. Recommended workflow

Use this combination:

- SSH for terminal control
- VNC or Pi desktop for the camera preview

Typical workflow:

1. From laptop, connect by SSH
2. From laptop or VNC, run the app on the Pi
3. Use the live preview to position the camera
4. Type `x` and Enter to inspect
5. Type `q` and Enter to quit

## 10. Common commands

Connect to Pi:

```powershell
ssh your_pi_username@192.168.1.50
```

Pull latest code on the Pi:

```bash
cd High-Level
git pull
```

Run app:

```bash
cd High-Level
source .venv/bin/activate
python app.py --profile indomie
```

Run with another camera:

```bash
python app.py --profile indomie --camera-index 1
```

## 11. Troubleshooting

If SSH does not connect:

- make sure the Pi and laptop are on the same network
- make sure SSH is enabled on the Pi
- make sure you are using the correct IP address

If the camera window does not show:

- this is expected on plain SSH
- use VNC or the Pi desktop session

If `git clone` with SSH fails:

- verify the Pi SSH key is added to GitHub
- see `SSH_SETUP.md`
