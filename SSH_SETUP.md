# SSH Setup

## Enable SSH on the Raspberry Pi

```bash
sudo apt update
sudo apt install -y openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh
sudo systemctl status ssh
hostname -I
```

## Create a GitHub SSH key on the Pi

```bash
ssh-keygen -t ed25519 -C "yousefkhaleddd34@gmail.com"
cat ~/.ssh/id_ed25519.pub
```

Copy the full `ssh-ed25519 ...` line and add it to:

- GitHub
- Settings
- SSH and GPG keys
- New SSH key

## Test GitHub SSH access

```bash
ssh -T git@github.com
```

If GitHub asks to trust the host, type:

```bash
yes
```

## Set the repo remote to SSH

Run these inside the repo:

```bash
git remote set-url origin git@github.com:Audix-Robotics/High-Level.git
git remote -v
```

## Push to the `vision` branch

```bash
git add .
git commit -m "Update vision branch"
git push origin vision
```
