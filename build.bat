@REM cmake .\Games101\HW1\src\ -B ./build/Games101/HW1 -GNinja -DCMAKE_BUILD_TYPE="Debug"
@REM cmake .\Games101\HW2\src\ -B ./build/Games101/HW2 -GNinja -DCMAKE_BUILD_TYPE="Debug"
@REM cmake .\Games101\HW3\src\ -B ./build/Games101/HW3 -GNinja -DCMAKE_BUILD_TYPE="Debug"
@REM cmake .\Games101\HW4\src\ -B ./build/Games101/HW4 -GNinja -DCMAKE_BUILD_TYPE="Debug"
@REM cmake .\Games101\HW5\src\ -B ./build/Games101/HW5 -GNinja -DCMAKE_BUILD_TYPE="Debug"

cmake .\Games101\HW6\src\ -B ./build/Games101/HW6 -GNinja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER=clang

@REM echo off
@REM rm ./compile_commands.json -r
@REM ln -s build/Games101/HW6/compile_commands.json ./