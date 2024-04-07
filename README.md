# Eurobot 2024
Files for EuroBot 2024 competition.
By MEPhI students

# Only for ARM64
## Installation:
`git clone https://github.com/mawerid/eurobot_2024.git`

`cd eurobot_2024`

`sudo bash init.sh`

## Open X11:
On local machine copy the output of `xauth list`.

Open container's terminal: `sudo docker exec -it eurobot_2024-base-1 bash`

Inside the container: `sudo xauth add <TOKEN LINE>`

Then you can start GUI apps inside the container, for example `rviz2`

## Optional:
Stop container: `sudo docker stop eurobot_2024-base-1`

Delete container: `sudo docker rm eurobot_2024-base-1`

Delete image: `sudo docker rmi eurobot_2024-base`

Check status of autostart service: `sudo systemctl status autocomposer`

Stop autostart service: `sudo systemctl stop autocomposer`

Disable autostart service: `sudo systemctl disable autocomposer`

## Troubleshooting:
If there is no container `eurobot_2024-base-1` check out all containers with `sudo docker ps`
