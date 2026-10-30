#!/bin/sh
# Host tests for obj_dict (no ESP-IDF needed). Also checks the frames in vectors.json,
# which the app's tests (HelloBalls-Remote test/od_vectors.json) mirror.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$(mktemp -d)
cc -std=c11 -Wall -Wextra -Werror -I"$HERE/../include" \
   "$HERE/../src/obj_dict.c" "$HERE/test_obj_dict.c" -o "$OUT/test_obj_dict"
"$OUT/test_obj_dict" "$HERE/vectors.json"
