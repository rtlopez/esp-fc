import os
import subprocess
import sys
import shutil
from os.path import join

Import("env")

def run_clang_format_base(check_only=False):
    # 1. Look for the binary in system PATH (handles .exe on Windows automatically)
    executable = shutil.which("clang-format-20") or shutil.which("clang-format")
    
    if not executable:
        print("\n[CLANG-FORMAT] ERROR: 'clang-format' not found in PATH.")
        print("Please install LLVM and ensure it is added to your system variables.")
        # Return without error to avoid breaking the PIO build process
        return 

    # 2. Define directories and extensions to process
    proj_dir = env.get("PROJECT_DIR")
    source_dirs = ["src", "lib/Espfc/src"]
    extensions = (".cpp", ".c", ".h", ".hpp", ".ipp")
    
    files_to_format = []
    for d in source_dirs:
        full_path = join(proj_dir, d)
        if not os.path.exists(full_path):
            continue
            
        for root, _, files in os.walk(full_path):
            for f in files:
                if f.endswith(extensions):
                    files_to_format.append(join(root, f))

    if not files_to_format:
        print("[CLANG-FORMAT] No source files found to process.")
        return

    # 3. Build the command
    # Uses the .clang-format file found in the project root
    command = [executable, "-style=file"]
    
    if check_only:
        # CI Mode: check without applying changes
        # --Werror returns non-zero exit code if formatting is required
        command += ["--dry-run", "--Werror"]
        print(f"[CLANG-FORMAT] Checking formatting for {len(files_to_format)} files...")
    else:
        # Local Mode: format files in-place
        command.append("-i")
        print(f"[CLANG-FORMAT] Formatting {len(files_to_format)} files...")

    command += files_to_format

    # 4. Execute the process
    try:
        # We use capture_output to see why it failed in the terminal/CI logs
        result = subprocess.run(command, capture_output=True, text=True)
        
        if result.returncode != 0:
            print("\n[CLANG-FORMAT] Issues detected:")
            print(result.stderr)
            if check_only:
                # In CI mode, we want to stop the pipeline
                sys.exit(1)
        else:
            print("[CLANG-FORMAT] Success.")
            
    except Exception as e:
        print(f"[CLANG-FORMAT] An unexpected error occurred: {e}")

def run_clang_format(*args, **kwargs):
    run_clang_format_base(check_only=False)

def run_check_format(*args, **kwargs):
    run_clang_format_base(check_only=True)

# Register PlatformIO Custom Targets
env.AddCustomTarget(
    name="format",
    dependencies=None,
    actions=[run_clang_format],
    title="Clang Format",
    description="Format source files in-place (needs clang-format in PATH)"
)

env.AddCustomTarget(
    name="check_format",
    dependencies=None,
    actions=[run_check_format],
    title="Check Format",
    description="Verify formatting without applying changes (for CI)"
)