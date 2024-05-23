#!/bin/bash
# Remove conflicts
for pkg in docker.io docker-doc docker-compose podman-docker containerd runc; do sudo apt-get remove $pkg; done

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
  bookworm stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Install latest docker with dependencies
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Test installation
sudo docker run hello-world

# Add autostart for docker demon
sudo systemctl enable docker.service
sudo systemctl enable containerd.service

# Clone repo
git clone https://github.com/NTDV/eurobot_2024.git
cd /home/pi/eurobot_2024
sudo chmod +x start.sh
sudo mv -f autocomposer.service /etc/systemd/system/
sudo systemctl enable autocomposer
sudo systemctl start autocomposer
