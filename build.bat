cd /d c:\trae\GasDispersionDesktop
if exist build-mingw rmdir /s /q build-mingw
cmake -G "MinGW Makefiles" -B build-mingw -DCMAKE_BUILD_TYPE=Release
cmake --build build-mingw -- -j4
