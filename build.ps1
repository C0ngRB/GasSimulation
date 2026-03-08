Remove-Item -Recurse -Force "c:/trae/GasDispersionDesktop/build-mingw" -ErrorAction SilentlyContinue
cmake -G "MinGW Makefiles" -B "c:/trae/GasDispersionDesktop/build-mingw" -DCMAKE_BUILD_TYPE=Release
