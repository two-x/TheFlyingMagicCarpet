#!/bin/bash
# Applies tools/wasm/patches/*.patch (unified diffs) to a freshly-copied
# FastLED source tree. Called from build.sh; see tools/wasm/README.md for
# what each patch does and why.
set -euo pipefail

TARGET_DIR="$1"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PATCHES_DIR="$SCRIPT_DIR/patches"

find "$PATCHES_DIR" -name '*.patch' | while read -r patch_file; do
  rel_path="${patch_file#"$PATCHES_DIR"/}"
  rel_path="${rel_path%.patch}"
  target_file="$TARGET_DIR/$rel_path"
  if [ ! -f "$target_file" ]; then
    echo "warning: patch target $target_file not found, skipping $patch_file (FastLED layout may have changed -- check tools/wasm/README.md)" >&2
    continue
  fi
  patch -s "$target_file" "$patch_file"
done
