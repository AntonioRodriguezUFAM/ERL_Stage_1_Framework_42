cmake_minimum_required(VERSION 3.10)

# Force CUDA to use GCC 8
#set(CMAKE_CUDA_HOST_COMPILER /usr/bin/gcc-8)

# ----------------------------
# CUDA Configuration
# ----------------------------
set(CMAKE_CUDA_HOST_COMPILER /usr/bin/gcc-8)  # Keep this from previous fix


# Set the project name and enable CUDA
project(MySystem LANGUAGES CXX CUDA)  # Add CUDA as a language



# Set the project name
#project(MySystem)

# Enable CUDA language
#enable_language(CUDA)

# Use C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



# ----------------------------------------------------------------------------------
# CUDA-Specific Configuration
# ----------------------------------------------------------------------------------
# Set CUDA standard to 14 instead of 17 (CUDA 10.2 doesn't support C++17)
set(CMAKE_CUDA_ARCHITECTURES 53)  #Corresponds to Maxwell architecture (sm_53), Jetson Nano's compute capability (5.3)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# Add CUDA-specific compiler flags
#set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler -Wall,-Wextra,-Wpedantic")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -g -G -Xcompiler -Wall,-Wextra,-Wpedantic -Xcudafe --display_error_number")

# ARM64 atomic operations fix
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8-a")

# ARM Architecture Flags (Jetson Nano: ARMv8-A Cortex-A57)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -march=armv8-a -mno-outline-atomics")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8-a -mno-outline-atomics")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a -mtune=cortex-a57 -O0")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8-a -mtune=cortex-a57 -O0")

# Compiler flags for ARM architecture
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a -mtune=cortex-a57 -O0")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8-a -mtune=cortex-a57 -O0")

# Suppress CMP0074 warning
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()


# ----------------------------------------------------------------------------------
# Enforce debug build by default if nothing else specified. 
# This ensures we get debug symbols for gdb or other debuggers.
# ----------------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    #set(CMAKE_BUILD_TYPE Debug)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Optionally add strong warnings for debugging
add_compile_options(
    "$<$<C_COMPILER_ID:GNU>:-Wall;-Wextra;-pedantic>"
    "$<$<CXX_COMPILER_ID:GNU>:-Wall;-Wextra;-pedantic>"
    "$<$<C_COMPILER_ID:Clang>:-Wall;-Wextra;-pedantic>"
    "$<$<CXX_COMPILER_ID:Clang>:-Wall;-Wextra;-pedantic>"
)

# For threads (pthread)
find_package(Threads REQUIRED)

# Add link directories for libraries
link_directories(/usr/lib/aarch64-linux-gnu/8)

# Find the libusb-1.0 package
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)


# Lynsyn Library
# ------------------------------------------------
# Search for "lynsyn" in standard paths AND the specific Lynsyn location within the build directory
find_library(LYNSYN_LIB
    NAMES lynsyn liblynsyn
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/build/lynsyn-host-software/liblynsyn
)

if(NOT LYNSYN_LIB)
    message(FATAL_ERROR "Lynsyn library not found! Adjust PATHS in find_library. Current path tried: ${CMAKE_CURRENT_SOURCE_DIR}/build/lynsyn-host-software/liblynsyn")
endif()
# ------------------------------------------------

# Look for SDL2 library
find_package(SDL2 REQUIRED)
if(NOT SDL2_FOUND)
    message(FATAL_ERROR "SDL2 not found! Please install libsdl2-dev.")
endif()

# Look for the libv4l2 library
find_library(V4L2_LIB v4l2)
if(NOT V4L2_LIB)
    message(FATAL_ERROR "libv4l2 (v4l2) not found. Please install libv4l-dev or set V4L2_LIB manually.")
endif()

# Find the fmt library
find_package(fmt REQUIRED)
if(NOT fmt_FOUND)
    message(FATAL_ERROR "fmt library not found! Please install fmt or ensure it is available in the project.")
endif()

# Find CUDA librariescmake
find_package(CUDA REQUIRED)


# Debug info
message(STATUS "------------------------------------------------------")
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "CMake version: ${CMAKE_VERSION}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "------------------------ CUDA ------------------------------")
message(STATUS "CUDA Compiler: ${CMAKE_CUDA_COMPILER}")
message(STATUS "CUDA Architecture: ${CMAKE_CUDA_ARCHITECTURES}")
message(STATUS "------------------------ CUDA -----------------------------")
message(STATUS "SDL2 include dirs: ${SDL2_INCLUDE_DIRS}")
message(STATUS "V4L2 library: ${V4L2_LIB}")
message(STATUS "fmt library: ${fmt_LIBRARIES}")
message(STATUS "Lynsyn library: ${LYNSYN_LIB}")  # Verify path
message(STATUS "Libusb library: ${LIBUSB}")  # Verify Libusb path
message(STATUS "------------------------------------------------------")


# ----------------------------------------------------------------------------------
# Executable with CUDA Support
# ----------------------------------------------------------------------------------
# Add CUDA source files (adjust with your actual CUDA files)
file(GLOB CUDA_SOURCES 
   # "Stage_01/Concretes/*.cu"
   # "Stage_01/SharedStructures/*.cu"
)

# Add the executable (which main .cpp do you want? Adjust as needed)
# Example: we use SampleTestIntegrationCode.cpp as your main:
add_executable(MySystem 
    #SampleTestIntegrationCode_v12.cpp
    #main-camera.cpp
    SampleTestIntegrationCode_v17.cpp
   # main.
   # LynsynMonitorConcreteTesting.cpp
    # Add CUDA source files here
    #cuda_kernel.cu
    Stage_01/Concretes/AlgorithmConcreteKernels.cu
    ${CUDA_SOURCES}  # Add CUDA source files
     )

# Ensure the compiler sees the local include folders:
target_include_directories(MySystem
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Stage_01
        ${CMAKE_CURRENT_SOURCE_DIR}/Stage_01/Factories
        ${CMAKE_CURRENT_SOURCE_DIR}/Stage_01/Concretes
        ${CMAKE_CURRENT_SOURCE_DIR}/Stage_01/SharedStructures
        ${CMAKE_CURRENT_SOURCE_DIR}/Stage_02
        ${CMAKE_CURRENT_SOURCE_DIR}/Stage_02/Factories
        ${SDL2_INCLUDE_DIRS}
        ${LIBUSB_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/Stage_01/Includes # Assuming lynsyn.h is here
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}  # CUDA headers

)

# Link libraries: threads + v4l2 + SDL2 + optionally SDL2_ttf + fmt
target_link_libraries(MySystem
    PRIVATE
	Threads::Threads
        ${CMAKE_THREAD_LIBS_INIT}
        ${V4L2_LIB}
        ${SDL2_LIBRARIES}
        ${LYNSYN_LIB}
        ${LIBUSB_LIBRARIES}
    

        # If you actually need SDL2_ttf, uncomment or ensure installed:
        #-lSDL2_ttf
        fmt::fmt
        # Add CUDA libraries if needed
        #CUDA::cudart  # CUDA runtime library
        #CUDA::nvToolsExt  # Optional: For profiling tools

        ${CUDA_LIBRARIES}           # Changed from CUDA::cudart
        ${CUDA_nvToolsExt_LIBRARY}  # Changed from CUDA::nvToolsExt
       # atomic # Link against atomic library
        # Add any other libraries you need
        stdc++fs # Add filesystem library
)


# Set CUDA specific flags
set_target_properties(MySystem PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
)
