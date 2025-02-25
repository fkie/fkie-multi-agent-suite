# sudo apt install python3-bloom python3-rosdep fakeroot debhelper dh-python
# sudo rosdep init
# rosdep update --include-eol-distros

if [ ! -f /etc/ros/rosdep/sources.list.d/10-mas.list ]; then
  echo "Create rosdep entries for mas packages:"
  sudo touch /etc/ros/rosdep/sources.list.d/10-mas.list
  sudo chown $USER /etc/ros/rosdep/sources.list.d/10-mas.list
  echo "yaml https://raw.githubusercontent.com/fkie/fkie-multi-agent-suite/refs/heads/devel/rosdep.yaml" > /etc/ros/rosdep/sources.list.d/10-mas.list
  rosdep update --include-eol-distros
  echo "Install required dependencies"
  sudo apt install -y python3-bloom python3-rosdep fakeroot debhelper dh-python
fi

function clean () {
  echo "Remove debian build files"
  rm -fr fkie_mas_msgs/debian
  rm -fr fkie_mas_msgs/.obj-*
  rm -fr fkie_mas_pylib/debian
  rm -fr fkie_mas_pylib/.pybuild
  rm -fr fkie_mas_pylib/.pytest_cache
  rm -fr fkie_mas_daemon/debian
  rm -fr fkie_mas_daemon/.pybuild
  rm -fr fkie_mas_daemon/.pytest_cache
  rm -fr fkie_mas_discovery/debian
  rm -fr fkie_mas_discovery/.obj-*
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

if [ "$1" == "clean" ]; then
  clean
  exit 0
fi
if [ "$1" == "clean-deb" ]; then
  clean-deb
  exit 0
fi


if [ ! -z "$1" ]; then
  ROS_DISTRO="$1"
fi

OS_VERSION=`lsb_release -c -s`
if [ "$ROS_DISTRO" == "galactic" ]; then
  OS_VERSION="focal"
elif [ "$ROS_DISTRO" == "jazzy" ]; then
  OS_VERSION="noble"
fi

clean

echo "Create debian packages for --os-version $OS_VERSION --ros-distro $ROS_DISTRO"

cd fkie_mas_msgs && bloom-generate rosdebian --os-version $OS_VERSION --ros-distro $ROS_DISTRO && fakeroot debian/rules binary && cd ..
cd fkie_mas_pylib && bloom-generate rosdebian --os-version $OS_VERSION --ros-distro $ROS_DISTRO && fakeroot debian/rules binary && cd ..
cd fkie_mas_discovery && bloom-generate rosdebian --os-version $OS_VERSION --ros-distro $ROS_DISTRO && fakeroot debian/rules binary && cd ..
cd fkie_mas_daemon && bloom-generate rosdebian --os-version $OS_VERSION --ros-distro $ROS_DISTRO && fakeroot debian/rules binary && cd ..
# cd fkie_mas_meta && bloom-generate rosdebian --os-version $OS_VERSION --ros-distro $ROS_DISTRO && fakeroot debian/rules binary && cd ..
if [ "$ROS_DISTRO" == "galactic" ]; then
  cd fkie_mas_sync && bloom-generate rosdebian --os-version $OS_VERSION --ros-distro $ROS_DISTRO && fakeroot debian/rules binary && cd ..
fi

rm -fr ./*-dbgsym_*.ddeb