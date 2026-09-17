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

docker build $* --progress=plain --tag fkie:jazzy_mas . --build-arg USE_PROXY=${USE_PROXY} --build-arg HTTPS_PROXY=${HTTPS_PROXY}
