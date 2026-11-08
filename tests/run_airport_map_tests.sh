#!/bin/sh
set -eu

test_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_dir=$(CDPATH= cd -- "$test_dir/.." && pwd)
test_bin="${TMPDIR:-/tmp}/betterpushback-airport-map-test"

cc -std=c99 -Wall -Wextra -Werror \
    -I"$repo_dir/src" \
    "$repo_dir/src/airport_map.c" "$test_dir/airport_map_test.c" \
    -lm -o "$test_bin"

"$test_bin" "$test_dir/data/airport_map_sample.dat"
rm -f "$test_bin"
