#!/bin/bash

if ! command -v jq &> /dev/null; then
    echo "jq is not installed. Please install jq to run the script."
    echo "sudo apt install jq"
    exit 1
fi

TMP_DIR=./tmp-mas

RELEASES_URL="https://api.github.com/repos/fkie/fkie-multi-agent-suite/releases"
RELEASES=$(curl -s "$RELEASES_URL")

# Check whether the releases have been successfully retrieved
if [ $? -ne 0 ]; then
    echo "Error when retrieving the releases."
    exit 1
fi

function get_package() {
    PACKAGE=$1
    DEB_URL=""

    # Search for the file in the releases
    LATEST_VERSION=""

    while read -r ASSET_NAME FILE_URL; do
        if [[ "$ASSET_NAME" == $PACKAGE*.deb ]]; then
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
            INSTALLED_VERSION=$(dpkg -l | grep "$PACKAGE" | grep -oP '\d+\.\d+\.\d+')
            echo "  Installed version: $INSTALLED_VERSION"

            # Extract latest version
            LATEST_VERSION=$(echo "$DEB_FILE" | grep -oP '\d+\.\d+\.\d+')

            if [ "$INSTALLED_VERSION" == "$LATEST_VERSION" ]; then
                echo "  The latest version is already installed, skip"
                DEB_FILE=
            else
                echo "  A new version is available: $LATEST_VERSION."
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
            echo "download finished"
        fi
    else
        echo "No .deb-file found."
    fi
}

if [[ "$1" != "no-gui" ]]; then
    get_package "fkie-mas-gui"
    # get_package "python3-websockets"
    # get_package "ttyd"
fi
if [[ "$ROS_DISTRO" == "noetic" || "$ROS_DISTRO" == "jazzy" ]]; then
    get_package "ros-$ROS_DISTRO-fkie-mas-msgs"
    get_package "ros-$ROS_DISTRO-fkie-mas-pylib"
    get_package "ros-$ROS_DISTRO-fkie-mas-discovery"
    get_package "ros-$ROS_DISTRO-fkie-mas-daemon"
    if [[ "$ROS_DISTRO" == "noetic" ]]; then
        get_package "ros-$ROS_DISTRO-fkie-mas-sync"
    fi
else
    echo "no valid ROS_DISTRO (noetic or jazzy) detected."
fi

if [ ! -z "$DEB_FILE" ]; then
    echo "install packages"

    # Install the debian package
    sudo dpkg -i -R $TMP_DIR

    # Fix dependencies
    sudo apt-get install -f
    # Cleanup
    rm -fr "$TMP_DIR"
    echo "Installation completed."
else
    echo "Nothing to do."
fi
