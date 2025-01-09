#!/bin/bash

printf "\n"
echo "--- help ---"
echo X-version with: './run.sh mas-x'
echo Change ros domain id with: 'ROS_DOMAIN_ID=1 ./run.sh'
printf "\n"

cd $(dirname $0)

# add proxy variables
if [[ -z "${https_proxy}" ]]; then
  echo "no proxy"
  export USE_PROXY=false
  export HTTPS_PROXY=""
else
  echo "set proxy to ${https_proxy}"
  export USE_PROXY=true
  export HTTPS_PROXY=${https_proxy}
fi

# set ROS_DOMAIN_ID
if [[ -z "${ROS_DOMAIN_ID}" ]]; then
  echo "use ROS_DOMAIN_ID=0"
  export MAS_DAEMON_PORT=35430
  export ROS_DOMAIN_ID=0
else
  echo "use ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  export MAS_DAEMON_PORT=$((35430+${ROS_DOMAIN_ID}))
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
fi
echo "use daemon port: ${MAS_DAEMON_PORT}"

xhost +local:
docker compose down
docker compose up --force-recreate --build $* # --build-arg USE_PROXY=${USE_PROXY} --build-arg HTTPS_PROXY=${HTTPS_PROXY}
docker compose down
xhost -local:
