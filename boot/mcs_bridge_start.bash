#!/bin/bash

cd /home/pi/mcs_bridge/
docker run   --network host   --ipc host   --env-file .env   mcs-bridge   --bridge-type ros2
