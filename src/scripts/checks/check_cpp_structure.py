import os
import sys

errors = []

BASE_DIR = (
    os.path.abspath(sys.argv[1])
    if len(sys.argv) > 1
    else os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
)


def validate_cpp(pkg_path, pkg_name):
    src_dir = os.path.join(pkg_path, "src")
    include_dir = os.path.join(pkg_path, "include", pkg_name)

    for root, _, files in os.walk(pkg_path):
        for file in files:
            if file.endswith(".cpp"):
                if not root.startswith(src_dir):
                    errors.append(f"{file} should be in {src_dir}")
            elif file.endswith(".hpp"):
                if not root.startswith(include_dir):
                    errors.append(f"{file} should be in {include_dir}")


def main():
    for pkg_name in os.listdir(BASE_DIR):
        pkg_path = os.path.join(BASE_DIR, pkg_name)
        if os.path.isdir(pkg_path):
            validate_cpp(pkg_path, pkg_name)

    if errors:
        for e in errors:
            print(f"{e}")
        sys.exit(1)
    print("C++ structure passed.")


if __name__ == "__main__":
    main()
