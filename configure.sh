#!/bin/bash

# This is probably only useful on my system..
#

set -x

ARGS_PREFIX_PATH="-DCMAKE_PREFIX_PATH=`pwd`/deps/build/destdir/usr/local"
ARGS_CCACHE=""
if [ -f /usr/bin/ccache ]; then
	ARGS_CCACHE="-DCMAKE_CXX_COMPILER_LAUNCHER=ccache"
fi
ARGS_STATIC="-DSLIC3R_STATIC=ON"
ARGS="${ARGS_PREFIX_PATH} ${ARGS_CCACHE} ${ARGS_STATIC}"
if [ -f /usr/bin/mold ]; then
	ARGS="${ARGS} -DCMAKE_CXX_FLAGS=-fuse-ld=mold"
	ARGS="${ARGS} -DCMAKE_C_FLAGS=-fuse-ld=mold"
fi
ARGS="${ARGS} -DCMAKE_COLOR_DIAGNOSTICS=ON"

cmake ${ARGS} -DSLIC3R_ENABLE_FORMAT_STEP=false -B build-debug -DCMAKE_BUILD_TYPE=Debug &&
cmake ${ARGS} -DSLIC3R_ENABLE_FORMAT_STEP=false -B build-release -DCMAKE_BUILD_TYPE=RelWithDebugInfo
