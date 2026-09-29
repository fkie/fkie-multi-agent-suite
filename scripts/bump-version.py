import argparse
import json
import os
import re
import subprocess
import xml.etree.ElementTree as ET
from datetime import datetime

# packages that collect all repository changes (prefixed with the package name)
ALL_CHANGES_PACKAGES = {"fkie_mas_gui"}

# tags that look like a version: 1.2.3 / v1.2.3 / v1.2 / 1.2.3-rc1
VERSION_TAG_RE = re.compile(r"^v?\d+(\.\d+)+([.\-+~][0-9A-Za-z.\-]+)?$")

RECORD_SEP = "\x1e"
FIELD_SEP = "\x1f"


# --------------------------------------------------------------------------
# git helpers
# --------------------------------------------------------------------------
def run_git(args, cwd=None, check=True):
    """Run a git command and return stdout (or None on error)."""
    try:
        return subprocess.check_output(["git"] + args, stderr=subprocess.STDOUT, cwd=cwd).decode("utf-8").strip()
    except subprocess.CalledProcessError as e:
        if check:
            print(f"git {' '.join(args)} failed:", e.output.decode().strip())
        return None


def get_repo_root():
    """Return the git repository root."""
    return run_git(["rev-parse", "--show-toplevel"])


def git_pull(repo_root):
    """Try to pull, but never abort the bump if it fails."""
    if run_git(["pull", "--ff-only"], cwd=repo_root, check=False) is None:
        print("Warning: 'git pull' failed or not possible, continuing with local state.")


def get_last_version_tag(repo_root):
    """Return the most recent tag reachable from HEAD that looks like a version."""
    out = run_git(["tag", "--merged", "HEAD", "--sort=-creatordate"], cwd=repo_root, check=False)
    if not out:
        print("No tags found, using complete history.")
        return None

    for tag in out.splitlines():
        tag = tag.strip()
        if tag and VERSION_TAG_RE.match(tag):
            print(f"Last version tag: {tag}")
            return tag

    print("No version-like tag found, using complete history.")
    return None


def get_package_dirs(repo_root):
    """Map repository-relative package directories to package names."""
    out = run_git(["ls-files", "*package.xml"], cwd=repo_root, check=False)
    packages = {}
    if not out:
        return packages

    for path in out.splitlines():
        path = path.strip()
        if not path:
            continue
        rel_dir = os.path.dirname(path)
        name = os.path.basename(rel_dir) or "root"
        try:
            tree = ET.parse(os.path.join(repo_root, path))
            name_tag = tree.getroot().find("name")
            if name_tag is not None and name_tag.text:
                name = name_tag.text.strip()
        except ET.ParseError:
            pass
        packages[rel_dir] = name
    return packages


def package_for_file(file_path, package_dirs):
    """Return the package name owning the given file (longest matching path)."""
    best_dir = None
    for pkg_dir in package_dirs:
        if pkg_dir == "" or file_path == pkg_dir or file_path.startswith(pkg_dir + "/"):
            if best_dir is None or len(pkg_dir) > len(best_dir):
                best_dir = pkg_dir
    return package_dirs[best_dir] if best_dir is not None else None


def is_ignored_subject(subject):
    """Filter out release housekeeping commits."""
    lower = subject.lower()
    return lower in ("prepare for release", "bump version") or lower.startswith("merge branch")


def collect_package_changes(since_ref, repo_root, rel_path):
    """Unique commit subjects since 'since_ref' limited to the package path."""
    args = ["log", "--no-merges", "--pretty=format:%s"]
    if since_ref:
        args.append(f"{since_ref}..HEAD")
    args += ["--", rel_path.replace(os.sep, "/")]

    log = run_git(args, cwd=repo_root)
    if not log:
        return []

    changes, seen = [], set()
    for line in log.splitlines():
        msg = line.strip()
        if not msg or is_ignored_subject(msg):
            continue
        if msg not in seen:
            seen.add(msg)
            changes.append(msg)
    return changes


def collect_all_changes(since_ref, repo_root, package_dirs, skip_name=None):
    """
    Unique commit subjects since 'since_ref' for the whole repo.
    Entries are prefixed with the affected package names, except for 'skip_name'.
    """
    args = [
        "log",
        "--no-merges",
        "--name-only",
        f"--pretty=format:{RECORD_SEP}%s{FIELD_SEP}",
    ]
    if since_ref:
        args.append(f"{since_ref}..HEAD")

    log = run_git(args, cwd=repo_root)
    if not log:
        return []

    changes, seen = [], set()
    for record in log.split(RECORD_SEP):
        if not record.strip():
            continue
        subject, _, files_block = record.partition(FIELD_SEP)
        subject = subject.strip()
        if not subject or is_ignored_subject(subject):
            continue

        names = []
        for file_path in files_block.splitlines():
            file_path = file_path.strip()
            if not file_path:
                continue
            name = package_for_file(file_path, package_dirs)
            # own package changes are not marked
            if name and name != skip_name and name not in names:
                names.append(name)

        entry = f"[{', '.join(sorted(names))}] {subject}" if names else subject
        if entry not in seen:
            seen.add(entry)
            changes.append(entry)
    return changes


# --------------------------------------------------------------------------
# file updates
# --------------------------------------------------------------------------
def update_package_json(new_version, current_version):
    """Update the version in package.json if it exists, preserving formatting."""
    package_json_path = "package.json"
    if os.path.exists(package_json_path):
        with open(package_json_path, encoding="utf-8") as json_file:
            package_data = json.load(json_file)

        package_data["version"] = new_version

        with open(package_json_path, "w", encoding="utf-8") as json_file:
            json.dump(package_data, json_file, indent=2, separators=(",", ": "))
            json_file.write("\n")

        print(f"Version updated in {package_json_path}.")
        return True

    elif os.getcwd().endswith("fkie_mas_daemon"):
        gui_file_path = "../fkie_mas_gui/src/renderer/context/SettingsContext.tsx"
        content = ""
        with open(gui_file_path, encoding="utf-8") as gui_file:
            content = gui_file.read()
            content = content.replace(
                f'MIN_VERSION_DAEMON = "{current_version}"', f'MIN_VERSION_DAEMON = "{new_version}"'
            )
        if content:
            with open(gui_file_path, "w", encoding="utf-8") as gui_file:
                gui_file.write(content)
                print(f"MIN_VERSION_DAEMON updated in {gui_file_path}.")
    return False


def update_changelog_md(new_version, changes):
    """Update CHANGELOG.md if it exists."""
    changelog_path = "CHANGELOG.md"
    if not os.path.exists(changelog_path):
        return

    if changes:
        bullet_list = "\n".join(f"- {change}" for change in changes)
    else:
        bullet_list = "- No changes found"

    changelog_entry = f"## {new_version} - {datetime.now().strftime('%d.%m.%Y')}\n\n{bullet_list}\n\n"

    with open(changelog_path, encoding="utf-8") as changelog_file:
        lines = changelog_file.readlines()

    insert_index = 2 if len(lines) >= 2 else len(lines)
    lines.insert(insert_index, changelog_entry)

    with open(changelog_path, "w", encoding="utf-8") as changelog_file:
        changelog_file.writelines(lines)

    print(f"Entry added to {changelog_path}.")


def update_changelog_rst(new_version, changes):
    """Update CHANGELOG.rst if it exists, inserting after the package title block."""
    changelog_path = "CHANGELOG.rst"
    if not os.path.exists(changelog_path):
        return

    title = f"{new_version} ({datetime.now().strftime('%d.%m.%Y')})"
    underline = "-" * len(title)

    if changes:
        bullet_list = "\n".join(f"* {change}" for change in changes)
    else:
        bullet_list = "* No changes found"

    changelog_entry = f"{title}\n{underline}\n{bullet_list}\n\n"

    with open(changelog_path, encoding="utf-8") as changelog_file:
        lines = changelog_file.readlines()

    insert_index = 0
    if len(lines) >= 4 and lines[3].strip() == "":
        insert_index = 4
    else:
        for i, line in enumerate(lines):
            if line.strip() == "":
                insert_index = i + 1
                break

    lines.insert(insert_index, changelog_entry)

    with open(changelog_path, "w", encoding="utf-8") as changelog_file:
        changelog_file.writelines(lines)

    print(f"Entry added to {changelog_path}.")


# --------------------------------------------------------------------------
# main logic
# --------------------------------------------------------------------------
def bump_version(package_path, version_part):
    """Increase the version in package.xml and update changelog files."""
    original_cwd = os.getcwd()
    abs_package_path = os.path.abspath(package_path)

    repo_root = get_repo_root()
    if not repo_root:
        print("Not inside a git repository.")
        return

    rel_package_path = os.path.relpath(abs_package_path, repo_root)
    git_pull(repo_root)

    since_ref = get_last_version_tag(repo_root)
    package_dirs = get_package_dirs(repo_root)

    os.chdir(abs_package_path)
    try:
        tree = ET.parse("package.xml")
        root = tree.getroot()

        version_tag = root.find("version")
        name_tag = root.find("name")
        if version_tag is None:
            print("No <version> tag found.")
            return

        current_version = version_tag.text.strip()
        package_name = (
            name_tag.text.strip() if name_tag is not None and name_tag.text else os.path.basename(abs_package_path)
        )
        print(f"Package: {package_name}, current version: {current_version}")

        try:
            major, minor, patch = map(int, current_version.split("."))
        except ValueError:
            print("Error parsing the version.")
            return

        if version_part == "major":
            major, minor, patch = major + 1, 0, 0
        elif version_part == "minor":
            minor, patch = minor + 1, 0
        else:
            patch += 1

        if package_name in ALL_CHANGES_PACKAGES:
            changes = collect_all_changes(since_ref, repo_root, package_dirs, skip_name=package_name)
            print(f"Collected {len(changes)} repository-wide changes.")
        else:
            changes = collect_package_changes(since_ref, repo_root, rel_package_path)
            print(f"Collected {len(changes)} changes for {package_name}.")

        new_version = f"{major}.{minor}.{patch}"
        version_tag.text = new_version
        print(f"New version: {new_version}")

        tree.write("package.xml", encoding="utf-8", xml_declaration=True)
        with open("package.xml", "a", encoding="utf-8") as xml_file:
            xml_file.write("\n")
        print("Version updated in package.xml.")

        update_package_json(new_version, current_version)
        update_changelog_md(new_version, changes)
        update_changelog_rst(new_version, changes)

    finally:
        os.chdir(original_cwd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Bump the version of a ROS package and update changelog files.")
    parser.add_argument("package_path", type=str, help="Path to the package")
    parser.add_argument(
        "version_part",
        type=str,
        choices=["major", "minor", "patch"],
        help="Part of the version to increase: 'major', 'minor', or 'patch'",
    )

    args = parser.parse_args()
    bump_version(args.package_path, args.version_part)
