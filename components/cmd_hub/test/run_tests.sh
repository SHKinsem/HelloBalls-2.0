#!/bin/sh
# Host tests for cmd_hub (no ESP-IDF needed)
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$(mktemp -d)
cc -Wall -Wextra -Werror -I"$HERE/stubs" -I"$HERE/../include" \
   "$HERE/../src/cmd_hub.c" "$HERE/test_cmd_hub.c" -o "$OUT/test_cmd_hub"
"$OUT/test_cmd_hub"
