#!/usr/bin/env bash
# Render src/chladni's engine natively — no hardware, no flash cycle.
# Same idea as tools/handpan_render.
#
#   tools/chladni_render/run.sh            # every scene
#   tools/chladni_render/run.sh bow        # one scene
#
# Scenes: contact strike bow position couple stress bench.
# WAVs land in tools/chladni_render/out/. Exits non-zero if a check fails.
set -euo pipefail
cd "$(dirname "$0")"

CXX=${CXX:-c++}
mkdir -p out

# Compiled straight from src/, never copied.
$CXX -std=gnu++17 -O2 -Wall -Wextra ${EXTRA:-} \
     -I../../src/chladni \
     -o render render.cpp ../../src/chladni/chladni_dsp.cpp

./render "${1:-all}"
