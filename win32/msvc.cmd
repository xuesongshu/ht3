@echo off
title Visual Studio 2019 Development Kit
set WORKDIR=%cd%
call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\VC\Auxiliary\Build\vcvarsall.bat" x64
call "C:\Qt\Qt5.14.0\5.14.0\msvc2017_64\bin\qtenv2.bat"
cd /d "%WORKDIR%"
cmd /k code .
