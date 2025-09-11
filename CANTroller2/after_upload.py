# after_upload.py (place in CANTroller2/, alongside platformio.ini)
from SCons.Script import Import
Import("env")

import os, shutil, subprocess, datetime

def run(cmd, check=True, capture=True):
    return subprocess.run(
        cmd, check=check, text=True,
        stdout=(subprocess.PIPE if capture else None),
        stderr=subprocess.STDOUT,
    )

def file_tracked(path):
    p = subprocess.run(["git", "ls-files", "--error-unmatch", path],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return p.returncode == 0

def file_changed_vs_index(path):
    if not os.path.exists(path):
        return False
    if not file_tracked(path):
        return True
    # returncode 1 => different; 0 => same
    p = subprocess.run(["git", "diff", "--quiet", "--", path])
    return p.returncode == 1

def after_upload(source, target, env_):
    try:
        # resolve important paths before we chdir
        build_dir   = env_.subst("$BUILD_DIR")         # absolute
        progname    = env_.subst("$PROGNAME")          # usually "firmware"
        src_bin     = os.path.join(build_dir, f"{progname}.bin")
        project_dir = env_.subst("$PROJECT_DIR")       # .../CANTroller2 (absolute)
        pioenv      = env_.get("PIOENV", "unknown-env")

        if not os.path.exists(src_bin):
            print(f"pio_post_upload: built binary not found: {src_bin}")
            return

        # artifact lives under the project dir so it stays with this pio project
        out_dir = os.path.join(project_dir, "firmware_artifacts", pioenv)
        os.makedirs(out_dir, exist_ok=True)
        dst_bin = os.path.join(out_dir, "firmware.bin")  # stable name

        # copy/overwrite the just-built file
        shutil.copy2(src_bin, dst_bin)

        # compute repo-root-relative path for nicer git output
        repo_root = run(["git", "rev-parse", "--show-toplevel"]).stdout.strip()
        rel_dst   = os.path.relpath(dst_bin, repo_root)

        # only commit if executable changed
        if not file_changed_vs_index(rel_dst):
            print("pio_post_upload: executable unchanged; skipping commit")
            return

        # commit from repo root so history is correct even with subfolders
        os.chdir(repo_root)

        # stage the binary (guarantee it’s included)
        run(["git", "add", "--", rel_dst], capture=False)

        # optionally: also stage code changes; comment out to make it bin-only
        run(["git", "add", "-A"], capture=False)

        # ensure there’s actually something to commit
        if subprocess.run(["git", "diff", "--cached", "--quiet"]).returncode == 0:
            print("pio_post_upload: nothing staged after add; skipping commit")
            return

        ts     = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        branch = run(["git", "rev-parse", "--abbrev-ref", "HEAD"]).stdout.strip()
        base   = run(["git", "rev-parse", "--short", "HEAD"]).stdout.strip()
        msg = f"build: commit after successful upload (env {pioenv}) @ {ts} [base {base}, branch {branch}]"

        run(["git", "commit", "-m", msg], capture=False)
        print(f"pio_post_upload: committed (includes {rel_dst})")

        # optional: keep timestamped copies too
        # stamped = os.path.join(out_dir, f"firmware-{datetime.datetime.now().strftime('%Y%m%d-%H%M%S')}.bin")
        # shutil.copy2(dst_bin, stamped)
        # run(["git", "add", "--", os.path.relpath(stamped, repo_root)], capture=False)
        # run(["git", "commit", "--amend", "--no-edit"], capture=False)

    except subprocess.CalledProcessError as e:
        print("pio_post_upload: git step failed\n", (e.stdout or "").strip())
        # don’t fail the upload due to git issues

env.AddPostAction("upload", after_upload)
