# after_build.py
from SCons.Script import Import
Import("env")
import subprocess, os, datetime

def run(cmd, check=True, capture=True):
    return subprocess.run(
        cmd,
        check=check,
        text=True,
        stdout=(subprocess.PIPE if capture else None),
        stderr=subprocess.STDOUT,
    )

def after_build(source, target, env_):
    try:
        # ensure we’re at the repo root (important if pio runs from .pio/build/…)
        toplevel = run(["git", "rev-parse", "--show-toplevel"]).stdout.strip()
        os.chdir(toplevel)

        # if nothing changed, skip
        dirty = run(["git", "status", "--porcelain"]).stdout.strip()
        if not dirty:
            print("pio_auto_commit: nothing to commit")
            return

        # stage everything that isn’t ignored (lean on .gitignore to exclude builds)
        run(["git", "add", "-A"])

        # if still nothing staged, skip
        staged_check = subprocess.run(["git", "diff", "--cached", "--quiet"])
        if staged_check.returncode == 0:
            print("pio_auto_commit: nothing staged after gitignore filters")
            return

        # build a helpful commit message
        ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        try:
            base = run(["git", "rev-parse", "--short", "HEAD"]).stdout.strip()
        except subprocess.CalledProcessError:
            base = "no-head"
        try:
            branch = run(["git", "rev-parse", "--abbrev-ref", "HEAD"]).stdout.strip()
        except subprocess.CalledProcessError:
            branch = "(detached)"

        pioenv = env_.get("PIOENV", "?")
        board = env_.GetProjectOption("board") or "?"

        msg = f"build: auto-commit after successful platformio compile @ {ts} (env {pioenv}, board {board}, base {base}, branch {branch})"
        run(["git", "commit", "-m", msg], capture=False)
        print("pio_auto_commit: committed changes")

        # optional lightweight tag per build (uncomment to use)
        # tag = f"pio-build-{datetime.datetime.now().strftime('%Y%m%d-%H%M%S')}"
        # run(["git", "tag", tag, "-m", tag], capture=False)
        # print(f"pio_auto_commit: tagged {tag}")

    except subprocess.CalledProcessError as e:
        # don’t fail the build if git misbehaves
        print("pio_auto_commit: git step failed\n", (e.stdout or "").strip())

# hook runs only after the firmware target is produced (i.e., successful build)
env.AddPostAction("$PROGNAME", after_build)
