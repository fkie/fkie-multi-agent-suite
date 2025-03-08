# sudo apt install python3-bloom python3-rosdep fakeroot debhelper dh-python
# sudo rosdep init
# rosdep update --include-eol-distros

usage() {
    echo "Parameters:"
    echo "  -c      remove all build files"
    echo "  -d      remove all debian files"
    echo "  -o      os version code"
    echo "  -r      ros distro code"
    echo "  -u      force update rosdep files"
    exit 1
}

while getopts hcdo:r:u  flag; do
    case "${flag}" in
        c) CLEAN=true;;
        d) CLEAN_DEB=true;;
        o) OS_VERSION=$OPTARG;;
        r) ROS_DISTRO=$OPTARG;;
        u) FORCE_UPDATE_ROSDEP=true;;
        h) usage ;;
        *) usage ;;
    esac
done

function clean () {
  echo "Remove debian build files"
  rm -fr fkie_mas_msgs/debian
  rm -fr fkie_mas_msgs/.obj-*
  rm -fr fkie_mas_pylib/debian
  rm -fr fkie_mas_pylib/.pybuild
  rm -fr fkie_mas_pylib/.pytest_cache
  rm -fr fkie_mas_pylib/.obj-*
  rm -fr fkie_mas_pylib/fkie_mas_pylib.egg-info
  rm -fr fkie_mas_daemon/debian
  rm -fr fkie_mas_daemon/.pybuild
  rm -fr fkie_mas_daemon/.pytest_cache
  rm -fr fkie_mas_daemon/.obj-*
  rm -fr fkie_mas_daemon/fkie_mas_daemon.egg-info
  rm -fr fkie_mas_discovery/debian
  rm -fr fkie_mas_discovery/.obj-*
  rm -fr fkie_mas_sync/debian
  rm -fr fkie_mas_sync/.obj-*
  rm -fr fkie_mas_meta/debian
  rm -fr fkie_mas_meta/.obj-*
  echo "Clean finished"
}

function clean-deb () {
  echo "Remove *.deb files"
  rm -fr ./*.deb
  rm -fr ./*.ddeb
  echo "Clean finished"
}

if [ ! -z "$CLEAN" ]; then
  clean
  if [ ! -z "$CLEAN_DEB" ]; then
    clean-deb
  fi
  exit 0
fi
if [ ! -z "$CLEAN_DEB" ]; then
  clean-deb
  exit 0
fi


DEP_INSTALLED=$(dpkg -l | grep debhelper)
if [ -z "$DEP_INSTALLED" ]; then
    echo "Install required dependencies"
    sudo apt install -y python3-bloom python3-rosdep fakeroot debhelper dh-python
fi

ROSDEP_REPO="/etc/ros/rosdep/sources.list.d/11-mas-$ROS_DISTRO.list"

if [ -z "$ROS_DISTRO" ]; then
  echo "unknown ROS_DISTRO, exit!"
fi

if [ ! -f $ROSDEP_REPO ] || [ ! -z $FORCE_UPDATE_ROSDEP ]; then
  echo "Create rosdep entries for mas packages:"
  sudo rm -fr /etc/ros/rosdep/sources.list.d/11-mas-*
  sudo touch $ROSDEP_REPO
  sudo chown $USER $ROSDEP_REPO
  echo "yaml https://raw.githubusercontent.com/fkie/fkie-multi-agent-suite/refs/heads/devel/rosdep/$ROS_DISTRO.yaml" > $ROSDEP_REPO
  rosdep update --include-eol-distros
fi

if [ -z "$OS_VERSION" ]; then
  OS_VERSION=`lsb_release -c -s`
fi

clean

echo -e "Create debian packages for --os-version \e[36m$OS_VERSION\e[0m --ros-distro \e[36m$ROS_DISTRO\e[0m"

cd fkie_mas_msgs && bloom-generate rosdebian --os-version $OS_VERSION --ros-distro $ROS_DISTRO && fakeroot debian/rules binary && cd ..
sudo apt install ./ros-$ROS_DISTRO-fkie-mas-msgs_*
cd fkie_mas_pylib && bloom-generate rosdebian --os-version $OS_VERSION --ros-distro $ROS_DISTRO && fakeroot debian/rules binary && cd ..
sudo apt install ./ros-$ROS_DISTRO-fkie-mas-pylib_*
cd fkie_mas_discovery && bloom-generate rosdebian --os-version $OS_VERSION --ros-distro $ROS_DISTRO && fakeroot debian/rules binary && cd ..
cd fkie_mas_daemon && bloom-generate rosdebian --os-version $OS_VERSION --ros-distro $ROS_DISTRO && fakeroot debian/rules binary && cd ..
# cd fkie_mas_meta && bloom-generate rosdebian --os-version $OS_VERSION --ros-distro $ROS_DISTRO && fakeroot debian/rules binary && cd ..
if [ "$ROS_DISTRO" == "noetic" ]; then
  sudo apt install ./ros-$ROS_DISTRO-fkie-mas-discovery_*
  cd fkie_mas_sync && bloom-generate rosdebian --os-version $OS_VERSION --ros-distro $ROS_DISTRO && fakeroot debian/rules binary && cd ..
fi

clean
rm -fr ./*-dbgsym_*.ddeb
sudo apt install -y ./ros-$ROS_DISTRO-fkie-mas-*
