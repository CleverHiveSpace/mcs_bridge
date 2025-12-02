#!/bin/bash

docker run -it   --network host   --ipc host   --env-file .env   mcs-bridge   --bridge-type ros2
