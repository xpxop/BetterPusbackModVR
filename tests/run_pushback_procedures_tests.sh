#!/bin/sh
set -eu

test_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_dir=$(CDPATH= cd -- "$test_dir/.." && pwd)
work_dir=$(mktemp -d)
trap 'rm -rf "$work_dir"' EXIT HUP INT TERM

cc -std=c99 -Wall -Wextra -Werror \
    -I"$repo_dir/src" \
    "$repo_dir/src/pushback_procedures.c" \
    "$test_dir/pushback_procedures_test.c" \
    -lm -o "$work_dir/pushback_procedures_test"

"$work_dir/pushback_procedures_test" "$work_dir"
