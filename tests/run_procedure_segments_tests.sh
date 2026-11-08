#!/bin/sh
set -eu

test_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_dir=$(CDPATH= cd -- "$test_dir/.." && pwd)
lacf="$repo_dir/../libacfutils"
sdk="$lacf/SDK/CHeaders"

if [ ! -f "$lacf/src/geom.c" ] || [ ! -d "$sdk/XPLM" ]; then
    printf 'Skipping procedure_segments test: libacfutils sources not found at %s\n' "$lacf"
    exit 0
fi

case "$(uname -s)" in
Darwin) undefined_flag="-Wl,-undefined,dynamic_lookup" ;;
*) undefined_flag="-Wl,--unresolved-symbols=ignore-all" ;;
esac

work_dir=$(mktemp -d)
trap 'rm -rf "$work_dir"' EXIT HUP INT TERM

# driving.c references XPLM probe/coordinate APIs that this test never
# calls, so unresolved XPLM symbols are tolerated at link time.
cc -std=gnu11 -w -DIBM=0 -DLIN=1 -DAPL=0 -DXPLM200=1 -DXPLM210=1 \
    -DXPLM300=1 -DXPLM301=1 -DXPLM303=1 -DXPLM400=1 \
    -I"$repo_dir/src" -I"$lacf/src" -I"$sdk/XPLM" -I"$sdk/Widgets" \
    "$test_dir/procedure_segments_test.c" \
    "$repo_dir/src/driving.c" "$repo_dir/src/pushback_procedures.c" \
    "$lacf/src/geom.c" "$lacf/src/list.c" "$lacf/src/math.c" \
    "$lacf/src/avl.c" \
    -lm "$undefined_flag" \
    -o "$work_dir/procedure_segments_test"

"$work_dir/procedure_segments_test"
