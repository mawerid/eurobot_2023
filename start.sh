#!/bin/bash
cd /home/pi/eurobot_2024
git reset --hard HEAD
git pull
sudo chmod +x start.sh
sudo docker compose up
