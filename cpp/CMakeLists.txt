cmake_minimum_required(VERSION 2.8.12)

set(PCK OusterDecoder)
project("${PCK}.u")

message(STATUS "Source Dir: ${CMAKE_CURRENT_SOURCE_DIR}")

## IMPORTANT: Set the same C++ standard for the whole project ######################################

set(CMAKE_CXX_STANDARD          17)   # Proper C++ >= 11 is required
# NOTE: This computer has gcc 9.4, which doesn't fully support c++20, so we use 17
set(CMAKE_CXX_STANDARD_REQUIRED ON)   # Enforce the C++ standard requirement
set(CMAKE_CXX_EXTENSIONS        OFF)  # Non-portable compiler-specific extensions are NOT required

## IMPORTANT: Locate the RTMaps SDK ################################################################
if (MSVC)
    set(RTMAPS_SDKDIR "C:/Program Files/Intempora/RTMaps 4" CACHE PATH "")
else()
    set(RTMAPS_SDKDIR "/opt/rtmaps" CACHE PATH "")
endif()

# CONFIGURE YOUR VCPKG LOCATION HERE:
#set(VCPKG_PATH "/home/sam/Desktop/Data/Data/software/libraries/cpp/vcpkg" CACHE PATH "")
set(VCPKG_PATH "C:/Users/smryk/Desktop/EcoCAR/CAVs/vcpkg" CACHE PATH "")

set(CAV_DATATYPES_LOC "../../../cav_autera_software/cav_datatypes" CACHE PATH "")
set(CMAKE_BUILD_TYPE "Debug" CACHE PATH "")
set(CMAKE_PREFIX_PATH "${VCPKG_PATH}/packages/;" ${CMAKE_PREFIX_PATH})

if (NOT DEFINED RTMAPS_SDKDIR)
    if (DEFINED ENV{RTMAPS_SDKDIR})
        set(RTMAPS_SDKDIR "$ENV{RTMAPS_SDKDIR}" CACHE PATH "")  # Use the default RTMaps installation
    else()
        message(FATAL_ERROR "RTMAPS_SDKDIR not defined. Either pass -D\"RTMAPS_SDKDIR=...\" to CMake or define an RTMAPS_SDKDIR environment variable")
    endif()
endif()

include("${RTMAPS_SDKDIR}/templates.u/rtmaps.cmake")

## IMPORTANT: Locate the input reader library ######################################################

if (NOT TARGET rtmaps_input_reader)
    set(rtmaps_input_reader_DIR "${RTMAPS_SDKDIR}/packages/rtmaps_input_reader/lib/cmake/rtmaps_input_reader" CACHE PATH "")
    find_package(rtmaps_input_reader CONFIG REQUIRED NO_DEFAULT_PATH)
endif()

find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(CURL REQUIRED)
find_package(spdlog 1.8 REQUIRED)

# REQUIRED: Create the pck target
# Note that the following folders will be automatically added to the list of include directories
# * local_interfaces
# * protected_interfaces
# * public_interfaces

add_rtmaps_package(${PCK} PCKINFO "${PCK}.pckinfo"
    # add sources, headers and/or directories

    "local_interfaces"                    # NB: if you add and/or remove files to this directory, you must re-run the CMake generation command
    "src"                                 # NB: if you add and/or remove files to this directory, you must re-run the CMake generation command
    "ouster_client/src"

)

target_link_libraries(${PCK} rtmaps_input_reader)

target_include_directories(${PCK} PRIVATE

    ouster_client/include
    ouster_client/include/optional-lite
    ouster_client/include/ouster

)
target_link_libraries(${PCK}
    Eigen3::Eigen
    CURL::libcurl
    jsoncpp_lib
    spdlog::spdlog
)

# RECOMMENDED: Good compiler options ##############################################################

if (MSVC)
   # MSVC Toolset >= 140 (i.e. 2015) is required
   target_compile_options(${PCK} PRIVATE /MP)
else()
   # https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
   # target_compile_options(${PCK} PRIVATE -fno-strict-aliasing -fdiagnostics-show-option -Wall -Wextra -pedantic -Wstrict-aliasing -Wshadow -Wtrigraphs -Wno-deprecated-copy)
   target_compile_options(${PCK} PRIVATE -fno-strict-aliasing -fdiagnostics-show-option)
endif()
