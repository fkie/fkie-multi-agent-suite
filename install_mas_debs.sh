#!/bin/bash

if ! command -v jq &> /dev/null; then
    echo -e "\e[31mjq is not installed. Please install jq to run the script.\e[0m"
    echo "sudo apt install jq"
    exit 1
fi

usage() {
    echo "Parameters:"
    echo "  -g      install only GUI relevant packages"
    echo "  -r      do not install GUI packages"
    echo "  -p      install prerelease packages"
    exit 1
}

while getopts grph  flag; do
    case "${flag}" in
        g) NO_ROS=true;;
        r) NO_GUI=true;;
        p) PRERELEASE=true;;
        h) usage ;;
        *) usage ;;
    esac
done

TMP_DIR=./tmp-mas

RELEASES_URL="https://api.github.com/repos/fkie/fkie-multi-agent-suite/releases"
RELEASES=$(curl -s "$RELEASES_URL")

# Check whether the releases have been successfully retrieved
if [ $? -ne 0 ]; then
    echo -e "\e[31mError when retrieving the releases from github.com\e[0m"
    exit 1
fi

if [ -z "$PRERELEASE" ]; then
    # remove prereleases
    RELEASES=$(echo "$RELEASES" | jq -r '[.[] | select(.prerelease==false)'])
else
    echo -e "\e[36mInstall also prereleases!\e[0m"
fi

function get_package() {
    PACKAGE=$1
    OS_CODENAME=$2
    DEB_URL=""

    # Search for the file in the releases
    LATEST_VERSION=""
    while read -r ASSET_NAME FILE_URL; do
        if [[ "$ASSET_NAME" == $PACKAGE*$OS_CODENAME*.deb ]]; then
            VERSION=$(echo "$ASSET_NAME" | grep -oP '\d+\.\d+\.\d+')
            FILE_FOUND=true
            if [[ -n "$VERSION" ]]; then
                if [[ "$LATEST_VERSION" < "$VERSION" ]]; then
                    LATEST_VERSION="$VERSION"
                    DEB_URL="$FILE_URL"
                fi
            fi
        fi
    done < <(echo "$RELEASES" | jq -r '.[] | .assets[] | select(.name | endswith(".deb")) | "\(.name) \(.browser_download_url)"')

    if [ ! -z "$DEB_URL" ]; then
        DEB_FILE=$(basename "$DEB_URL")

        # Check whether the latest version is already installed.
        if dpkg -l | grep -q "$PACKAGE"; then
            echo "$PACKAGE already installed."
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
            echo "$PACKAGE is not installed."
        fi

        if [ ! -z "$DEB_FILE" ]; then
            echo "Download: $DEB_URL"
            mkdir -p $TMP_DIR
            cd $TMP_DIR
            curl -L -O "$DEB_URL"
            echo -n " ./$DEB_FILE"
            cd ..
            echo -e "download finished"
        fi
    else
        echo -e "\e[31mNo .deb-file for '$PACKAGE' found.\e[0m"
    fi
}

if [[ -z $NO_GUI ]]; then
    echo "Get GUI package for Multi-Agent-Suite"
    get_package "fkie-mas-gui"
fi

if [[ -z $NO_ROS ]]; then
    echo "Get dependency packages for Multi-Agent-Suite"

    OS_CODENAME=$(env -i bash -c '. /etc/os-release; echo $VERSION_CODENAME')
    if [[ "$OS_CODENAME" == "focal" || "$OS_CODENAME" == "noble" ]]; then
        echo -e "detected OS_CODENAME=\e[36m$OS_CODENAME\e[0m"
    else
        echo -e "\e[31mdebian packages are only available for focal and noble, exit\e[0m"
        exit 1
    fi

    get_package "python3-websockets" $OS_CODENAME
    get_package "ttyd" $OS_CODENAME

    echo "Get ROS packages for Multi-Agent-Suite"

    if [[ "$ROS_DISTRO" == "noetic" || "$ROS_DISTRO" == "galactic" || "$ROS_DISTRO" == "jazzy" ]]; then
        echo -e "detected ROS_DISTRO=\e[36m$ROS_DISTRO\e[0m"
        get_package "ros-$ROS_DISTRO-fkie-mas-msgs" $OS_CODENAME
        get_package "ros-$ROS_DISTRO-fkie-mas-pylib" $OS_CODENAME
        get_package "ros-$ROS_DISTRO-fkie-mas-discovery" $OS_CODENAME
        get_package "ros-$ROS_DISTRO-fkie-mas-daemon" $OS_CODENAME
        if [[ "$ROS_DISTRO" == "noetic" ]]; then
            get_package "ros-$ROS_DISTRO-fkie-mas-sync" $OS_CODENAME
        fi
    else
        if [ -z "$ROS_DISTRO" ]; then
            echo -e "\e[31mROS_DISTRO is not set\e[0m"
        else
            echo -e "\e[31mno debian packages for $ROS_DISTRO available.\e[0m"
        fi
    fi
fi

if [ -d "$TMP_DIR" ]; then
    echo "Install packages"

    # Fix dependencies
    if sudo apt install $TMP_DIR/*; then
        echo -e "\e[32mInstallation completed.\e[0m"
        echo -e "\e[32mRestart mas nodes, please!.\e[0m"
    else
        echo -e "\e[31mInstallation failed\e[0m"
    fi
    # Cleanup
    rm -fr "$TMP_DIR"
else
    echo -e "Nothing to install."
fi
read  -n 1 -p "Press <any key> to quit"
