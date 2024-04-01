REM This is probably only useful on my system...
REM

set "args=-DSLIC3R_STATIC=ON -DCMAKE_PREFIX_PATH=C:/src/PrusaSlicer-deps/usr/local -G Ninja"

cmake %args% --preset no-occt -B build-release -DCMAKE_BUILD_TYPE=Release