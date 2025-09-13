# gitv.py
import subprocess, datetime
from SCons.Script import Import
Import("env")

# 1) verbose flag set to true
verbose = True

error = False  # flips to True if any git call fails

def _run(cmd, default="unknown"):
    global error
    try:
        out = subprocess.check_output(cmd, stderr=subprocess.DEVNULL)
        return out.decode().strip()
    except Exception:
        error = True
        return default

def _call_ok(cmd) -> bool:
    global error
    try:
        rc = subprocess.call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return rc == 0
    except Exception:
        error = True
        return False

def _has_unstaged_changes() -> bool:
    return not _call_ok(["git", "diff", "--quiet"])

def _has_staged_changes() -> bool:
    return not _call_ok(["git", "diff", "--cached", "--quiet"])

def _has_untracked_files() -> bool:
    out = _run(["git", "ls-files", "--others", "--exclude-standard"], default="")
    # if _run failed, error is already flagged; treat as "no untracked" to avoid extra noise
    return out != ""

def _is_dirty() -> bool:
    return _has_unstaged_changes() or _has_staged_changes() or _has_untracked_files()

# gather values
sha        = _run(["git", "rev-parse", "--short=8", "HEAD"])
branch     = _run(["git", "rev-parse", "--abbrev-ref", "HEAD"])
dirty_flag = "1" if _is_dirty() else "0"
build_time = datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")

# finalize error flag
git_error = "1" if error else "0"

# helper to safely wrap as a C string literal
def cstr(s: str) -> str:
    # escape backslashes and double quotes, then wrap in quotes
    return '"' + s.replace('\\', '\\\\').replace('"', '\\"') + '"'

# inject preprocessor defines
env.Append(
    CPPDEFINES=[
        ("GIT_ENABLED",    1),                 # numeric 0/1     # did this script run
        ("GIT_SHA",        cstr(sha)),        # -> -DGIT_SHA="abc12345"     # short head sha or "unknown"
        ("GIT_BRANCH",     cstr(branch)),     # -> -DGIT_BRANCH="soren42"   # branch name or "unknown"
        ("BUILD_TIME_UTC", cstr(build_time)), # -> -DBUILD_TIME_UTC="2025-09-13T19:05:22Z"
        ("GIT_DIRTY",      1 if dirty_flag == "1" else 0),  # numeric 0/1
        ("GIT_ERROR",      1 if git_error == "1" else 0),   # numeric 0/1
    ]
)

if verbose:
    if git_error == "1":
        print(f"[gitv.py] git_error=1 (values may be unreliable); sha={sha} branch={branch}")
    else:
        print(f"[gitv.py] sha={sha} dirty={dirty_flag} branch={branch} built={build_time} error={git_error}")