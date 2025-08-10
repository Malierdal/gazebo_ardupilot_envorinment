import os
import sys

errors = []

BASE_DIR = (
    os.path.abspath(sys.argv[1])
    if len(sys.argv) > 1
    else os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
)

IGNORE_FOLDERS = {"scripts", ".git", ".github", ".vscode", "__pycache__"}


def validate_launch_file(pkg_path, pkg_name):
    launch_dir = os.path.join(pkg_path, "launch")
    expected_launch_file = os.path.join(launch_dir, f"{pkg_name}.launch")

    if not os.path.isdir(launch_dir):
        errors.append(f"Missing 'launch/' directory in package '{pkg_name}'")
        return

    if not os.path.isfile(expected_launch_file):
        errors.append(
            f"Missing launch file: '{pkg_name}.launch' in 'launch/' directory of package '{pkg_name}'"
        )


def main():
    for pkg in os.listdir(BASE_DIR):
        if pkg in IGNORE_FOLDERS:
            continue

        pkg_path = os.path.join(BASE_DIR, pkg)
        if os.path.isdir(pkg_path):
            validate_launch_file(pkg_path, pkg)

    if errors:
        for e in errors:
            print(f"{e}")
        sys.exit(1)

    print("All launch file validations passed.")


if __name__ == "__main__":
    main()
