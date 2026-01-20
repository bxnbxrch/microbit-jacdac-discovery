cmake_minimum_required(VERSION 3.6)
project(codal-microbit-v2 C CXX)

# Include utility scripts
include(utils/cmake/JSONParser.cmake)
include(utils/cmake/util.cmake)
include(utils/cmake/colours.cmake)

if(NOT "${BUILD_TOOL}" STRGREATER "")
    set(BUILD_TOOL "CODAL")
endif()

# Supress unnecessary compiler checks
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

# Read codal.json
file(READ "./codal.json" codal_json)
sbeParseJson(codal codal_json)

set(CODAL_APP_OUTPUT_DIR ".")
set(CODAL_APP_SOURCE_DIR "source")
if("${codal.application}" STRGREATER "")
    set(CODAL_APP_SOURCE_DIR "${codal.application}")
endif()
if("${codal.output_folder}" STRGREATER "")
    set(CODAL_APP_OUTPUT_DIR "${codal.output_folder}")
endif()

# Validate target
if(NOT "${codal.target.name}" STRGREATER "")
    message(FATAL_ERROR "${BoldRed}INVALID TARGET.${ColourReset}")
endif()

set(LIB_DEST "libraries")
set(CODAL_DEPS "")

# Install main target
INSTALL_DEPENDENCY(${LIB_DEST} ${codal.target.name} ${codal.target.url} ${codal.target.branch} ${codal.target.type})
list(APPEND CODAL_DEPS ${codal.target.name})
message("${BoldMagenta}Set target: ${codal.target.name}${ColourReset}")

# Parse target JSON
if("${codal.target.dev}" STRGREATER "")
    file(READ "./${LIB_DEST}/${codal.target.name}/target.json" device_json)
else()
    file(READ "./${LIB_DEST}/${codal.target.name}/target-locked.json" device_json)
endif()
sbeParseJson(device device_json)

# -------------------------
# ===== Config Merge ======
# -------------------------
# Merge device defaults with user overrides from codal.json
set(DEVICE_FIELDS "")
set(DEVICE_VALUES "")
foreach(var ${device})
    if("${var}" MATCHES "^device\\.config\\.")
        string(REPLACE "device.config." "" key "${var}")
        list(APPEND DEVICE_FIELDS "${key}")
        list(APPEND DEVICE_VALUES "${${var}}")
    endif()
endforeach()

set(CODAL_FIELDS "")
set(CODAL_VALUES "")
foreach(var ${codal})
    if("${var}" MATCHES "^codal\\.config\\.")
        string(REPLACE "codal.config." "" key "${var}")
        list(APPEND CODAL_FIELDS "${key}")
        list(APPEND CODAL_VALUES "${${var}}")
    endif()
endforeach()

UNIQUE_JSON_KEYS(CODAL_FIELDS CODAL_VALUES DEVICE_FIELDS DEVICE_VALUES FINAL_FIELDS FINAL_VALUES)

# Emit merged config header to be force-included for all targets
set(EXTRA_INCLUDES_PATH "${PROJECT_SOURCE_DIR}/build/codal_extra_definitions.h")
FORM_DEFINITIONS(FINAL_FIELDS FINAL_VALUES EXTRA_DEFINITIONS)
file(WRITE "${EXTRA_INCLUDES_PATH}" "${EXTRA_DEFINITIONS}")
add_compile_options(-include ${EXTRA_INCLUDES_PATH})

# Toolchain setup
SET(TOOLCHAIN ${device.toolchain})
SET(TOOLCHAIN_FOLDER "./utils/cmake/toolchains/${device.toolchain}")
set(CMAKE_TOOLCHAIN_FILE "${TOOLCHAIN_FOLDER}/toolchain.cmake" CACHE PATH "toolchain file")
enable_language(ASM)
include(${TOOLCHAIN_FOLDER}/compiler-flags.cmake)

# Apply device-specific compile and link flags/defines
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${device.cpu_opts} ${device.c_flags} ${device.definitions}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${device.cpu_opts} ${device.cpp_flags} ${device.definitions}")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${device.cpu_opts} ${device.asm_flags} ${device.definitions}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${device.linker_flags}")

# Add libraries folder for includes
include_directories(${PROJECT_SOURCE_DIR}/${LIB_DEST})

# Toolchain platform includes (for platform_includes.h)
include_directories(${TOOLCHAIN_FOLDER})

# -------------------------
# ===== Jacdac Setup ======
# -------------------------
set(JACDAC_LIB "codal-jacdac")
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/${LIB_DEST}/${JACDAC_LIB}/CMakeLists.txt")
    message(FATAL_ERROR "Jacdac library not found! Clone it here: git clone https://github.com/microsoft/codal-jacdac.git libraries/codal-jacdac")
endif()
list(APPEND CODAL_DEPS "${JACDAC_LIB}")

# Add other device libraries if any
if("${device.libraries}" STRGREATER "")
    foreach(i ${device.libraries})
        INSTALL_DEPENDENCY(${LIB_DEST} ${device.libraries_${i}.name} ${device.libraries_${i}.url} ${device.libraries_${i}.branch} ${device.libraries_${i}.type})
        list(APPEND CODAL_DEPS "${device.libraries_${i}.name}")
    endforeach()
endif()

# Add all dependencies (target + jacdac + device libs)
foreach(dep ${CODAL_DEPS})
    add_subdirectory("${PROJECT_SOURCE_DIR}/${LIB_DEST}/${dep}")
endforeach()

# -------------------------
# ===== Application =======
# -------------------------
# Collect sources (limit to main.cpp to keep RAM usage low)
RECURSIVE_FIND_DIR(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/${CODAL_APP_SOURCE_DIR}" "*.h")
RECURSIVE_FIND_DIR(HPP_DIRS "${PROJECT_SOURCE_DIR}/${CODAL_APP_SOURCE_DIR}" "*.hpp")
list(APPEND INCLUDE_DIRS ${HPP_DIRS})
set(SOURCE_FILES "${PROJECT_SOURCE_DIR}/${CODAL_APP_SOURCE_DIR}/main.cpp")

if("${SOURCE_FILES}" STREQUAL "")
    message(FATAL_ERROR "${BoldRed}No application to build! Add main.cpp to ${PROJECT_SOURCE_DIR}/${CODAL_APP_SOURCE_DIR}${ColourReset}")
endif()

# Add Jacdac headers to the include list used by the build tool.
list(APPEND INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/${LIB_DEST}/${JACDAC_LIB}/inc")

# -------------------------
# ===== Build Tool ========
# -------------------------
if("${BUILD_TOOL}" STRGREATER "")
    string(COMPARE EQUAL "${BUILD_TOOL}" "CODAL" CODAL_BUILD)
    if(${CODAL_BUILD})
        include("${PROJECT_SOURCE_DIR}/utils/cmake/buildtools/codal.cmake")
    endif()
endif()
