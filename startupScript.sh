#!/bin/bash
set -xo
sed -i "s/127.0.0.1/${ORION_IP}/g" catkin_ws/src/snm_test_github/firos/config/config.json
sed -i "s/1026/${ORION_PORT_EXT}/g" catkin_ws/src/snm_test_github/firos/config/config.json
cat catkin_ws/src/snm_test_github/firos/config/config.json

source /opt/ros/kinetic/setup.bash
cd catkin_ws
catkin_make
chmod -R 777 *
source devel/setup.bash

uuid=$(uuidgen)

sed -i "s/carob/carob-$uuid/g" src/snm_test_github/firos/config/robots.json

roslaunch carob_fieldnav carob_fieldnav.launch id:=$uuid &

sleep 10

curl --location --request POST "http://${ORION_IP}:${ORION_PORT_EXT}/v2/entities/carob-${uuid}/attrs" \
  --header 'Content-Type: application/json' \
  --data-raw '{
  "Domain": {"value": "'"${DOMAIN}"'"},
  "Mission": {"value": "'"${MISSION}"'"}
}'
