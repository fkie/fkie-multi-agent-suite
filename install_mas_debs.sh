#!/bin/bash

if ! command -v jq &> /dev/null; then
    echo -e "\e[31mjq is not installed. Please install jq to run the script.\e[0m"
    echo "sudo apt install jq"
    read  -n 1 -p "Press <any key> to quit"
    exit 1
fi

usage() {
    echo "Parameters:"
    echo "  -g      install only GUI relevant packages"
    echo "  -r      do not install GUI packages"
    echo "  -p      install prerelease packages"
    echo "  -w      wait for key press to quit"
    echo "  -f      force the installation of debian packages, even if the packages have been cloned into the workspace."
    echo "  -e      removes fkie mas packages. ttyd and python3-websockets are not uninstalled."
    echo "  -d      delete downloaded files after installation"
    echo "  -s  <version> install selected release, e.g.: 3.8.0"
    echo "  -u  <user[:password]> user and password useful if API rate limit is exceeded"
    exit 1
}

while getopts grphwfes:u:  flag; do
    case "${flag}" in
        g) NO_ROS=true;;
        r) NO_GUI=true;;
        p) PRERELEASE=true;;
        w) WAIT_FOR_KEY=true;;
        f) FORCE_INSTALL=true;;
        e) UNINSTALL=true;;
        d) DELETE_DOWNLOADED=true;;
        s) FORCE_VERSION=$OPTARG;;
        u) FORCE_USER=$OPTARG;;
        h) usage ;;
        *) usage ;;
    esac
done

function exit_with_no() {
    if [[ ! -z $WAIT_FOR_KEY ]]; then
        read  -n 1 -p "Press <any key> to quit"
    fi
    exit $1
}

# check for OS and ROS_DISTRO
if [[ -z $NO_ROS ]]; then
    OS_CODENAME=$(env -i bash -c '. /etc/os-release; echo $VERSION_CODENAME')
    if [[ "$OS_CODENAME" == "focal"  || "$OS_CODENAME" == "jammy" || "$OS_CODENAME" == "noble" ]]; then
        echo -e "detected OS_CODENAME=\e[36m$OS_CODENAME\e[0m"
    else
        echo -e "\e[31mdebian packages are only available for focal, jammy and noble, exit\e[0m"
        exit_with_no 1
    fi

    if [[ "$ROS_DISTRO" == "noetic" || "$ROS_DISTRO" == "galactic" || "$ROS_DISTRO" == "humble" || "$ROS_DISTRO" == "jazzy" || "$ROS_DISTRO" == "kilted" ]]; then
        echo -e "detected ROS_DISTRO=\e[36m$ROS_DISTRO\e[0m"
    else
        if [ -z "$ROS_DISTRO" ]; then
            echo -e "\e[31mROS_DISTRO is not set, please set up your ROS environment first.\e[0m"
            exit_with_no 1
        else
            echo -e "\e[31mno debian packages for $ROS_DISTRO available.\e[0m"
            exit_with_no 1
        fi
    fi
fi

if [[ ! -z $UNINSTALL ]]; then
    OS_CODENAME=$(env -i bash -c '. /etc/os-release; echo $VERSION_CODENAME')
    echo "Uninstall packages of the Multi-Agent-Suite"
    echo -e "\033[36m🔐 sudo apt remove fkie-mas-gui *fkie-mas-*\e[0m"
    sudo apt remove fkie-mas-gui *fkie-mas-*
    echo -e "\e[32mfinished.\e[0m"
    exit_with_no 0
fi

if [[ "$ROS_DISTRO" != "noetic" ]]; then
    PACKAGE_PATH=$(ros2 pkg prefix fkie_mas_daemon)
    if [ ! -z "$PACKAGE_PATH" ]; then
        if [[ "$PACKAGE_PATH" == *"/install/"* ]]; then
            echo -e "\e[31mYou are using MAS packages in your workspace. Please update them with git and rebuild them.\e[0m"
            echo -e "\e[31mOr remove them from the workspace to install them as debian packages.\e[0m"
            if [ -z $FORCE_INSTALL ]; then
                exit_with_no 1
            fi
        fi
    fi
fi

TMP_DIR=/tmp/mas-debs

RELEASES_URL="https://api.github.com/repos/fkie/fkie-multi-agent-suite/releases"
if [ -z $FORCE_USER ]; then
    RELEASES=$(curl -s "$RELEASES_URL")
else
    RELEASES=$(curl -u $FORCE_USER -s "$RELEASES_URL")
fi

# Check whether the releases have been successfully retrieved
if [ $? -ne 0 ]; then
    echo -e "\e[31mError when retrieving the releases from github.com\e[0m"
    exit_with_no 1
fi

MESSAGE=$(echo "$RELEASES" | jq -r 'if type=="array" then false else has("message") end')
if [ "$MESSAGE" = true ] ; then
    echo -e "$RELEASES"
    exit_with_no 1
fi

if [ -z $FORCE_VERSION ]; then
    if [ -z "$PRERELEASE" ]; then
        # remove prereleases
        RELEASES=$(echo "$RELEASES" | jq -r '[.[] | select(.prerelease==false)'])
    else
        echo -e "\e[36mInstall also prereleases!\e[0m"
    fi
else
    echo -e "\e[36mSelect release for installation: $FORCE_VERSION\e[0m"
    RELEASES=$(echo "$RELEASES" | jq -r --arg FORCE_VERSION "$FORCE_VERSION" '[.[] | select(.name==$FORCE_VERSION)'])
fi


function download_and_verify_deb() {
  local deb_file="$1"
  local deb_url="$2"
  local expected_digest="$3"
  local tmp_dir="$4"

  if [ -z "$deb_file" ]; then
    echo -e "❌ No .deb file name provided."
    return 0
  fi

  mkdir -p "$tmp_dir"
  local deb_path="$tmp_dir/$deb_file"

  echo -e "Target path: $deb_path"

  if [ -f "$deb_path" ]; then
    echo -e "⬇️ File already exists at $deb_path - skipping download."
  else
    echo -e "⬇️ Downloading: $deb_url"
    cd "$tmp_dir" || return 1
    curl -L -O "$deb_url"
    cd - >/dev/null || return 1
  fi

  echo -n "📦 File: $deb_path"

  if [ -n "$expected_digest" ]; then
    if ! command -v sha256sum >/dev/null 2>&1; then
      echo -e "\n❌ 'sha256sum' is not installed."
      return 1
    fi

    if ! command -v awk >/dev/null 2>&1; then
      echo -e "\n❌ 'awk' is not installed."
      return 1
    fi

    actual_hash=$(sha256sum "$deb_path" | awk '{print $1}')

    if [ "$expected_digest" = "sha256:$actual_hash" ]; then
      echo -e "\n✅ SHA256 hash for $deb_path is valid."
    else
      echo -e "\n❌ SHA256 hash mismatch for $deb_path!"
      echo -e "Expected: $expected_digest"
      echo -e "Found:    sha256:$actual_hash"
      return 1
    fi
  fi

  echo -e "✅ Download and verification complete."
}


DEBS_TO_INSTALL=()

function get_package() {
    PACKAGE=$1
    OS_CODENAME=$2
    DEB_URL=""
    EXPECTED_DIGEST=""

    # Search for the file in the releases
    LATEST_VERSION=""
    while read -r ASSET_NAME FILE_URL DIGEST; do
        if [[ "$ASSET_NAME" == $PACKAGE*$OS_CODENAME*.deb ]]; then
            VERSION=$(echo "$ASSET_NAME" | grep -oP '\d+\.\d+\.\d+')
            if [[ -n "$VERSION" ]]; then
                if [[ ! -n "$LATEST_VERSION" ]] || dpkg --compare-versions $LATEST_VERSION lt $VERSION; then
                    LATEST_VERSION="$VERSION"
                    DEB_URL="$FILE_URL"
                    EXPECTED_DIGEST="$DIGEST"
                fi
            fi
        fi
    done < <(echo "$RELEASES" | jq -r '.[] | .assets[] | select((.name | endswith(".deb")) and (.digest)) | "\(.name) \(.browser_download_url) \(.digest)"')

    if [ ! -z "$DEB_URL" ]; then
        DEB_FILE=$(basename "$DEB_URL")

        # Check whether the latest version is already installed.
        if dpkg -l | grep -q "$PACKAGE"; then
            echo -e "\e[35m$PACKAGE\e[0m already installed."
            INSTALLED_VERSION=$(dpkg -l | grep "$PACKAGE" | grep -oP '\d+\.\d+.\d+' | sed -r "s/-/\./")
            echo "  Installed version: $INSTALLED_VERSION"

            # Extract latest version
            LATEST_VERSION=$(echo "$DEB_FILE" | grep -oP '\d+\.\d+\.\d+')

            if [ "$INSTALLED_VERSION" == "$LATEST_VERSION" ]; then
                echo "  The latest version is already installed, skip"
                DEB_FILE=
            else
                echo "  A new version is available: $LATEST_VERSION"
            fi
        else
            echo -e "\e[35m$PACKAGE\e[0m is not installed."
        fi

        if [ ! -z "$DEB_FILE" ]; then
            if ! download_and_verify_deb "$DEB_FILE" "$DEB_URL" "$EXPECTED_DIGEST" "$TMP_DIR"; then
                exit_with_no 1
            fi
            DEBS_TO_INSTALL+=("$TMP_DIR/$DEB_FILE")
        fi
    else
        echo -e "⚠️ \e[38;5;208m No .deb-file for '$PACKAGE' found.\e[0m"
    fi
}

if [[ -z $NO_GUI ]]; then
    echo "Get GUI package for Multi-Agent-Suite"
    get_package "fkie-mas-gui"
fi

if [[ -z $NO_ROS ]]; then
    echo "Get dependency packages for Multi-Agent-Suite"

    OS_CODENAME=$(env -i bash -c '. /etc/os-release; echo $VERSION_CODENAME')
    if [[ "$OS_CODENAME" == "focal"  || "$OS_CODENAME" == "jammy" || "$OS_CODENAME" == "noble" ]]; then
        echo -e "detected OS_CODENAME=\e[36m$OS_CODENAME\e[0m"
    else
        echo -e "\e[31mdebian packages are only available for focal, jammy and noble, exit\e[0m"
        exit_with_no 1
    fi

    get_package "python3-websockets" $OS_CODENAME

    echo "Get ROS packages for Multi-Agent-Suite"
    echo -e "detected ROS_DISTRO=\e[36m$ROS_DISTRO\e[0m"
    get_package "ros-$ROS_DISTRO-fkie-mas-msgs" $OS_CODENAME
    get_package "ros-$ROS_DISTRO-fkie-mas-pylib" $OS_CODENAME
    get_package "ros-$ROS_DISTRO-fkie-mas-discovery" $OS_CODENAME
    get_package "ros-$ROS_DISTRO-fkie-mas-daemon" $OS_CODENAME
    if [[ "$ROS_DISTRO" == "noetic" ]]; then
        get_package "ros-$ROS_DISTRO-fkie-mas-sync" $OS_CODENAME
    fi
fi


function restart_mas() {
    echo -e "\e[32m🔄 Restarting mas daemon nodes...\e[0m"
    echo -e "\033[36mros2 run fkie_mas_daemon mas-restart.py\e[0m"
    ros2 run fkie_mas_daemon mas-restart.py
    echo -e "\e[32m🔄 Restart mas gui? (Y/n)\e[0m"
    read yn
    case "$yn" in
        [yY]|"") 
            echo -e "\e[32mRestarting mas gui...\e[0m"
            echo -e "\033[36mscreen -dmS .mas-gui /bin/bash -c 'killall -q mas-gui && mas-gui'\e[0m"
            screen -dmS .mas-gui /bin/bash -c 'killall -q mas-gui && mas-gui'
            ;;
        [nN])
            ;;
    esac
}

if [ -d "$TMP_DIR" ]; then
    echo "Install packages"

    if [[ "$OS_CODENAME" == "focal" ]]; then
        # install ttyd using snap and all other packages using apt
        echo -e "\033[36m🔐 sudo apt install -y --allow-downgrades ${DEBS_TO_INSTALL[@]}\e[0m"
        if sudo apt install -y --allow-downgrades ${DEBS_TO_INSTALL[@]}; then
            echo -e "\e[33mno ttyd packages available for focal, installing using snap:\e[0m"
            sudo snap install ttyd --classic
            echo -e "\e[32m✅ Installation completed.\e[0m"
            restart_mas
        else
            echo -e "❌ \e[31mInstallation failed\e[0m"
        fi
    else
        # install all packages using apt
        echo -e "\033[36m🔐 sudo apt install -y --allow-downgrades ttyd ${DEBS_TO_INSTALL[@]}\e[0m"
        if sudo apt install -y --allow-downgrades ttyd ${DEBS_TO_INSTALL[@]}; then
            echo -e "\e[32m✅ Installation completed.\e[0m"
            restart_mas
        else
            echo -e "❌ \e[31mInstallation failed\e[0m"
        fi
    fi
    # Cleanup
    if [[ ! -z $DELETE_DOWNLOADED ]]; then
        rm -fr "$TMP_DIR"
    fi
else
    echo -e "Nothing to install."
fi

if [[ ! -z $WAIT_FOR_KEY ]]; then
    read  -n 1 -p "Press <any key> to quit"
fi
echo -e "\e[32mfinished.\e[0m"
