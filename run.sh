# #! /bin/bash

# # Make sure executable tag is passed
# if [[ $# == 0 ]]; then
#     cd src
#     options=$(find . -maxdepth 1 -type f -name '*.cpp')
#     echo "Must pass C++ file name without extension (bash run.sh <filename>). Here are your file options:"
#     while IFS= read -r file; do
#         file_name=$(basename "$file")
#         echo " - ${file_name%.*}"  # remove file extension to display available files
#     done <<< "$options"
#     exit 1
# fi

# cpp_file="src/$1.cpp"

# if [ ! -f "$cpp_file" ]; then
#     echo "C++ file '$1.cpp' not found in the 'src' directory."
#     exit 1
# fi

# shift

# # Run CMake and build the project
# mkdir -p build
# cd build
# cmake ..
# make

# if [ $? == 0 ]; then
#     executable="${cpp_file%.*}"
#     "./$executable" "$@"  # execute the compiled executable with additional arguments if provided
# else
#     echo "Build Failed!"
# fi

# debug_flag=""
# while [[ $# -gt 0 ]]; do 
#     case "$2" in 
#         -r|--rebuild)
#             echo "Rebuilding from scratch..."
#             rm -rf build            
#             shift 1
#             ;;
#         -d|--debug)
#             debug_flag="-DCMAKE_BUILD_TYPE=Debug"
#             shift 1
#             ;;
#         *)
#             echo "Unknown arg '$1'"
#             exit 1
#             ;;
#     esac
# done

# cmake .
# make

# if [ $? == 0 ]; then
#     executable="$1"
#     "./$executable" "$@"  # execute the compiled executable with additional arguments if provided
# else
#     echo "Build Failed!"
# fi


#!/bin/bash

rebuild_flag=false
debug_flag=""

# Process command-line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -r|--rebuild)
            echo "Rebuilding from scratch..."
            rebuild_flag=true
            shift 1
            ;;
        -d|--debug)
            debug_flag="-DCMAKE_BUILD_TYPE=Debug"
            shift 1
            ;;
        *)
            if [ -z "$executable" ]; then
                executable="$1"
            else
                echo "Unknown arg '$1'"
                exit 1
            fi
            shift 1
            ;;
    esac
done

# Rebuild if the -r flag was used
if [ "$rebuild_flag" = true ]; then
    rm -rf CMakeFiles
fi

# Run CMake and Make
# mkdir -p build && cd build
cmake $debug_flag .
make

if [ $? == 0 ]; then
    if [ -n "$executable" ]; then
        "./$executable" "${@:2}"  # execute the compiled executable with additional arguments if provided
    else
        echo "Executable not specified."
    fi
else
    echo "Build Failed!"
fi
