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
ARGS="${ARGS} -DCMAKE_COLOR_DIAGNOSTICS=ON"

cmake ${ARGS} --preset no-occt -B build-debug -DCMAKE_BUILD_TYPE=Debug &&
cmake ${ARGS} --preset no-occt -B build-release -DCMAKE_BUILD_TYPE=RelWithDebugInfo
