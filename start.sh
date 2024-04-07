#!/bin/bash
cd eurobot_2024
git pull origin master
xauth list
sudo docker compose up
