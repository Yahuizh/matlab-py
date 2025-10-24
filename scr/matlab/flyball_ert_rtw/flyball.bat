set CMAKE_EXPORT_COMPILE_COMMANDS=TRUE
"E:\BaiduNetdiskDownload\matlab24\bin\win64\cmake\bin\cmake.exe" -S . -B build -G "Visual Studio 16 2019" -A x64 -DCMAKE_TOOLCHAIN_FILE="E:\BaiduNetdiskDownload\matlab24/toolbox/coder/compile/cmake/windows_msvc_toolchain.cmake"  -DCMAKE_PLATFORM_INFO_INITIALIZED:INTERNAL=1 -DCMAKE_INSTALL_PREFIX=".." --no-warn-unused-cli
@if errorlevel 1 (
    @echo The cmake command returned an error of %errorlevel% 1>&2
    @exit /B 1
)

"E:\BaiduNetdiskDownload\matlab24\bin\win64\cmake\bin\cmake.exe" --build build --config Release
@if errorlevel 1 (
    @echo The cmake command returned an error of %errorlevel% 1>&2
    @exit /B 1
)

"E:\BaiduNetdiskDownload\matlab24\bin\win64\cmake\bin\cmake.exe" --install build --prefix .. --config Release
@if errorlevel 1 (
    @echo The cmake command returned an error of %errorlevel% 1>&2
    @exit /B 1
)

