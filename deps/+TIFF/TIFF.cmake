add_cmake_project(TIFF
    #URL https://gitlab.com/libtiff/libtiff/-/archive/v4.6.0/libtiff-v4.6.0.zip
    #URL_HASH SHA256=5d652432123223338a6ee642a6499d98ebc5a702f8a065571e1001d4c08c37e6
	GIT_REPOSITORY https://gitlab.com/libtiff/libtiff/
	GIT_TAG cmake-cmath-avoid-imported-target
    CMAKE_ARGS
        -Dtiff-tools:BOOL=OFF
        -Dtiff-tests:BOOL=OFF
        -Dlzma:BOOL=OFF
        -Dwebp:BOOL=OFF
        -Djbig:BOOL=OFF
        -Dzstd:BOOL=OFF
        -Dpixarlog:BOOL=OFF
        -Dlibdeflate:BOOL=OFF
        -DCMAKE_POLICY_DEFAULT_CMP0057=NEW
        -Dtiff-contrib:BOOL=OFF
)

set(DEP_TIFF_DEPENDS ZLIB PNG JPEG OpenGL)
