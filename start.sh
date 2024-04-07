#!/bin/bash
cd eurobot_2024
git pull origin master
sudo chmod +x ~/eurobot_2024/start.sh
xauth list
sudo docker compose up
