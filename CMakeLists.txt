cmake_minimum_required(VERSION 3.28)
project(RV_FSTSP_heuristic)
set(CMAKE_CXX_FLAGS_RELEASE "-O3  -march=native -fsanitize=address -g")
# -ffast-math -funsafe-math-optimizations
#-Wall -Wextra
set(CMAKE_CXX_STANDARD 26)
SET(CMAKE_CXX_FLAGS_PERFORMANCE
        "${CMAKE_CXX_FLAGS_RELEASE} -march=native"
        CACHE STRING "Flags used by the C++ compiler during performance builds."
        FORCE )
set(CMAKE_CONFIGURATION_TYPES "Debug;Release;Performance" CACHE STRING "" FORCE)
mark_as_advanced(CMAKE_CONFIGURATION_TYPES)
add_executable(RV_FSTSP_heuristic main.cpp
        instance.h
        instance.cpp
        parameters.h
        no_loop/solution.h
        no_loop/solution.cpp
        no_loop/alns_ops.h
        no_loop/alns_ops.cpp
        with_loop/solutionl.h
        with_loop/solutionl.cpp
        with_loop/alnsl_ops.h
        with_loop/alnsl_ops.cpp
)

set_target_properties(RV_FSTSP_heuristic PROPERTIES OUTPUT_NAME "RV-FSTSP_heuristic_250k_75k")