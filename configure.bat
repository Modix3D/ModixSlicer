REM This is probably only useful on my system...
REM

set "args=-DSLIC3R_STATIC=ON -DCMAKE_PREFIX_PATH=C:/src/ModixSlicer/deps/build/destdir/usr/local"

cmake %args% --preset no-occt -Bbuild -H. -DCMAKE_BUILD_TYPE=Release -G "Ninja"
