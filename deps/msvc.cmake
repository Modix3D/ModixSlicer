set(CMAKE_SYSTEM_NAME      Windows)
set(CMAKE_SYSTEM_PROCESSOR AMD64)

set(CMAKE_C_COMPILER     /usr/lib/llvm/16/bin/clang-cl)
set(CMAKE_CXX_COMPILER   /usr/lib/llvm/16/bin/clang-cl)
set(CMAKE_LINKER         /usr/lib/llvm/16/bin/lld-link)

set(CMAKE_CXX_FLAGS "--target=x86_64-windows-msvc /EHa")
set(CMAKE_C_FLAGS   "--target=x86_64-windows-msvc /EHa")

add_definitions(-DWIN32=1)
set(CMAKE_FIND_USE_CMAKE_SYSTEM_PATH OFF)
