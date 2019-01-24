NextSoftware/VVCSoftware_VTM build howto:

The software uses cmake to create the needed build files. 
Download cmake: http://www.cmake.org/ and install it.

=================== Windows only =======================
Python and gnuwin32 are not mandatory, but they simplifiy the build process for the user.
python:    https://www.python.org/downloads/release/python-371/
gnuwin32:  https://sourceforge.net/projects/getgnuwin32/files/getgnuwin32/0.6.30/GetGnuWin32-0.6.3.exe/download

To use MinGW, install MSYS2:
http://repo.msys2.org/distrib/msys2-x86_64-latest.exe

Installation instructions:
https://www.msys2.org/

and install the needed toolchains.
pacman -S --needed base-devel mingw-w64-i686-toolchain mingw-w64-x86_64-toolchain git subversion mingw-w64-i686-cmake mingw-w64-x86_64-cmake
========================================================

========= Build instructions for plain cmake ===========
Open a command prompt on your system and change into the root directory
of this project (location of README.txt).

Create a build directory in the root directory:
mkdir build 

After that use one of the following cmake commands. Feel free to change the 
commands to satisfy your needs.

Windows sample for Visual Studio 2015 64 Bit:
cd build
cmake .. -G "Visual Studio 14 2015 Win64"

Linux Release Makefile sample:
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release

Linux Debug Makefile sample:
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug

MACOSX Xcode sample:
cd build
cmake .. -G "Xcode"
========================================================

============= Build instructions for make ==============
remark:
If you installed python and gnuwin32 on Windows operating systems, 
you will be able to use make.

Open a command prompt on your system and change into the root directory
of this project (location of README.txt).

to use the default system compiler simply call:
make all

Using MSYS2 and MinGW:
Open an MSYS MinGW 64-Bit terminal and change into the root directory
of this project (location of README.txt).

Call:
make all toolset=gcc
========================================================
