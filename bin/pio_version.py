import shutil
import subprocess

from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()


def has_define(name):
    for flag in env.get("BUILD_FLAGS", []):
        for token in str(flag).split():
            if token == f"-D{name}" or token.startswith(f"-D{name}="):
                return True

    for item in env.get("CPPDEFINES", []):
        key = item[0] if isinstance(item, (list, tuple)) else item
        if str(key) == name:
            return True

    return False


def git_output(*args):
    executable = shutil.which("git")
    if not executable:
        return None

    command = [executable, "-C", env.subst("$PROJECT_DIR")] + list(args)
    try:
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=10, check=False
        )
    except Exception as e:  # noqa: BLE001
        print(f"Failed to run git: {e}")
        return None

    if result.returncode != 0:
        return None

    return result.stdout.strip() or None


def apply_define(name, *args):
    if has_define(name):
        return

    value = git_output(*args)
    if not value:
        print(f"{name} not detected, using default")
        return

    env.Append(CPPDEFINES=[(name, value)])
    print(f"{name}={value}")


apply_define("ESPFC_REVISION", "rev-parse", "--short=7", "HEAD")
# apply_define("ESPFC_VERSION", "describe", "--tags", "--abbrev=0", "--match", "v*")
