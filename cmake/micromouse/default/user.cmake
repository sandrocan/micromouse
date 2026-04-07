# user.cmake – Custom include paths for the layered source directory structure.
# This file is included by the generated CMakeLists.txt and is safe to commit.
# See: https://onlinedocs.microchip.com/oxy/GUID-ED8B541E-5E3C-4029-8AB1-E8B73B87E536-en-US-1/GUID-E8B0B3A0-D10E-4C1E-B3E0-B8E6B3E3E3E0.html

# Add the src/ sub-directories so that flat #include "header.h" statements
# continue to work after the files have been moved into the layered structure.
target_include_directories(micromouse_default_default_XC16_compile PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/hal"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/drivers"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/control"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/app"
)
