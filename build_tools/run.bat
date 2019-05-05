cd %~dp0\..
mkdir out
cd out
cmake -G "Visual Studio 16 2019" ..\src
cd %~dp0
