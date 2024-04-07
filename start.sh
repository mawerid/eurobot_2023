#!/bin/bash
cd /home/pi/eurobot_2024
git pull origin master
sudo chmod +x start.sh
xauth list
sudo docker compose up
