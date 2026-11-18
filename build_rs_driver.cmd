@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
set CMAKE="C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
set NINJA=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe
cd /d C:\Users\JamesVong\LiDAR\rs_driver
echo ===== CONFIGURE
%CMAKE% -S . -B build -G Ninja "-DCMAKE_MAKE_PROGRAM=%NINJA%" -DCMAKE_BUILD_TYPE=Release -DENABLE_USB=ON -DCOMPILE_DEMOS=ON -DDISABLE_PCAP_PARSE=ON
if errorlevel 1 ( echo ===== CONFIGURE FAILED & exit /b 1 )
echo ===== BUILD
%CMAKE% --build build --config Release -j 16
if errorlevel 1 ( echo ===== BUILD FAILED & exit /b 1 )
echo ===== BUILD OK
dir /s /b build\*.exe
