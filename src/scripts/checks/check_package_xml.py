import os
import sys
import xml.etree.ElementTree as ET

errors = []

BASE_DIR = (
    os.path.abspath(sys.argv[1])
    if len(sys.argv) > 1
    else os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
)

IGNORE_FOLDERS = {".git", ".github", ".vscode", "__pycache__", "scripts"}


def validate_package(pkg_path, pkg_name):
    path = os.path.join(pkg_path, "package.xml")
    if not os.path.exists(path):
        errors.append(f"Missing package.xml in {pkg_path}")
        return
    try:
        tree = ET.parse(path)
        root = tree.getroot()
        xml_name = root.find("name").text
        if xml_name != pkg_name:
            errors.append(
                f"package.xml name '{xml_name}' does not match directory name '{pkg_name}'"
            )
    except Exception as e:
        errors.append(f"Error parsing package.xml in {pkg_path}: {str(e)}")


def should_check_package(pkg_path, pkg_name):
    if pkg_name in IGNORE_FOLDERS:
        return False
    if not os.path.isdir(pkg_path):
        return False
    contents = set(os.listdir(pkg_path))
    if contents == {"CMakeLists.txt"}:
        return False
    return True


def main():
    for pkg_name in os.listdir(BASE_DIR):
        pkg_path = os.path.join(BASE_DIR, pkg_name)
        if should_check_package(pkg_path, pkg_name):
            validate_package(pkg_path, pkg_name)

    if errors:
        for e in errors:
            print(f"{e}")
        sys.exit(1)

    print("package.xml validation passed.")


if __name__ == "__main__":
    main()
