import xml.etree.ElementTree as ET
import argparse
from datetime import datetime
import subprocess
import os
import json


def get_git_log(since_version):
    """Get the git log since the last version."""
    try:
        # Perform a git pull to ensure the latest changes are fetched
        subprocess.check_call(["git", "pull"], stderr=subprocess.STDOUT)

        # Prepend 'v' to the since_version
        since_version = f"v{since_version}"

        log = subprocess.check_output(
            ["git", "log", f"{since_version}..HEAD", "--oneline"],
            stderr=subprocess.STDOUT
        ).decode('utf-8').strip()
        return log
    except subprocess.CalledProcessError as e:
        print("Error retrieving git log:", e.output.decode())
        return ""


def update_package_json(new_version, current_version):
    """Update the version in package.json if it exists, preserving formatting."""
    package_json_path = 'package.json'
    if os.path.exists(package_json_path):
        with open(package_json_path, 'r') as json_file:
            package_data = json.load(json_file)

        package_data['version'] = new_version

        # Write back with pretty formatting
        with open(package_json_path, 'w') as json_file:
            json.dump(package_data, json_file, indent=2, separators=(
                ',', ': '))  # Preserve original formatting
            # append new line at the end
            json_file.write('\n')
        print(f"Version updated in {package_json_path}.")
        return True
    elif os.getcwd().endswith('fkie_mas_daemon'):
        gui_file_path = '../fkie_mas_gui/src/renderer/context/SettingsContext.tsx'
        content = ''
        with open(gui_file_path, 'r') as gui_file:
            content = gui_file.read()
            content = content.replace(
                f'MIN_VERSION_DAEMON: "{current_version}"', f'MIN_VERSION_DAEMON: "{new_version}"')
        if content:
            with open(gui_file_path, 'w') as gui_file:
                gui_file.write(content)
                print(f"MIN_VERSION_DAEMON updated in {gui_file_path}.")
    return False


def bump_version(package_path, version_part):
    """Increase the version in package.xml and update CHANGELOG.md."""
    os.chdir(package_path)  # Change to the specified directory

    tree = ET.parse('package.xml')
    root = tree.getroot()

    version_tag = root.find('version')
    if version_tag is None:
        print("No <version> tag found.")
        return

    current_version = version_tag.text
    print(f"Current version: {current_version}")

    try:
        major, minor, patch = map(int, current_version.split('.'))

        if version_part == 'major':
            major += 1
            minor = 0
            patch = 0
        elif version_part == 'minor':
            minor += 1
            patch = 0
        elif version_part == 'patch':
            patch += 1
        else:
            print("Invalid version part. Please specify 'major', 'minor', or 'patch'.")
            return

        new_version = f"{major}.{minor}.{patch}"
        version_tag.text = new_version
        print(f"New version: {new_version}")

        # Save changes to package.xml, preserving formatting
        tree.write('package.xml', encoding='utf-8', xml_declaration=True)
        # append new line at the end
        with open('package.xml', 'a') as xml_file:
            xml_file.write('\n')
        print(f"Version updated in package.xml.")

        # Update version in package.json if it exists
        json_updated = update_package_json(new_version, current_version)

        if json_updated:
            # Get git log since the last version
            log_since_last_release = get_git_log(current_version)
            changelog_entry = f"## {new_version} - {datetime.now().strftime('%d.%m.%Y')}\n\n{log_since_last_release}\n\n"

            # Insert entry in the 3rd line of CHANGELOG.md
            changelog_path = 'CHANGELOG.md'
            with open(changelog_path, 'r') as changelog_file:
                lines = changelog_file.readlines()

            lines.insert(2, changelog_entry)  # Insert in the 3rd line
            with open(changelog_path, 'w') as changelog_file:
                changelog_file.writelines(lines)

            print(f"Entry added to {changelog_path}.")

    except ValueError:
        print("Error parsing the version.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Bump the version of a ROS package and update CHANGELOG.md.')
    parser.add_argument('package_path', type=str, help='Path to the package')
    parser.add_argument('version_part', type=str, choices=['major', 'minor', 'patch'],
                        help="Part of the version to increase: 'major', 'minor', or 'patch'")

    args = parser.parse_args()
    bump_version(args.package_path, args.version_part)
