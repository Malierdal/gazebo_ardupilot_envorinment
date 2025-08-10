import os
import sys

errors = []

BASE_DIR = (
    os.path.abspath(sys.argv[1])
    if len(sys.argv) > 1
    else os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
)

IGNORE_FOLDERS = {"scripts", ".git", ".github", ".vscode", "__pycache__"}


def validate_python(pkg_path):
    scripts_dir = os.path.join(pkg_path, "scripts")
    for root, _, files in os.walk(pkg_path):
        for f in files:
            if f.endswith(".py") and not root.startswith(scripts_dir):
                errors.append(f"{f} should be in {scripts_dir}")


def main():
    for pkg in os.listdir(BASE_DIR):
        if pkg in IGNORE_FOLDERS:
            continue

        pkg_path = os.path.join(BASE_DIR, pkg)
        if os.path.isdir(pkg_path):
            validate_python(pkg_path)

    if errors:
        for e in errors:
            print(f"{e}")
        sys.exit(1)

    print("Python structure passed.")


if __name__ == "__main__":
    main()
