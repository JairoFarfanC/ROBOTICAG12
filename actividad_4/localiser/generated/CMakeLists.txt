cmake_minimum_required(VERSION 3.10)

# Force output binaries to project's bin folder (fixes "Permission denied" in /bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../bin)

# Select the compiler standard
if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 11)
    message(FATAL_ERROR "g++ version must be at least 11.")
elseif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 12)
    message(STATUS "Using g++ 11 → C++20")
    set(CXX_STANDARD 20)
elseif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 13)
    message(STATUS "Using g++ 12 → C++23")
    set(CXX_STANDARD 23)
else()
    message(STATUS "Using g++ 13+ → C++23")
    set(CXX_STANDARD 23)
endif()

# Check RoboComp installation
if(NOT EXISTS $ENV{ROBOCOMP}/cmake)
    message(FATAL_ERROR "RoboComp not found at $ENV{ROBOCOMP}. Please set ROBOCOMP environment variable.")
endif()

# Include RoboComp modules
include($ENV{ROBOCOMP}/cmake/robocomp.cmake)
include($ENV{ROBOCOMP}/cmake/modules/qt.cmake)
include(../src/CMakeLists.txt)  # Include user-defined source rules (if any)

# Ice configuration
add_definitions("-DICE_CPP11_MAPPING")
find_package(Ice REQUIRED COMPONENTS Ice++11 IceStorm++11)

# Source files
set(SOURCES
    $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
    $ENV{ROBOCOMP}/classes/sigwatch/sigwatch.cpp
    $ENV{ROBOCOMP}/classes/grafcetStep/GRAFCETStep.cpp
    $ENV{ROBOCOMP}/classes/ConfigLoader/ConfigLoader.cpp
    main.cpp
    genericworker.cpp
    MNIST.cpp  # ← Added for MNIST proxy
)

# RoboComp initialization
robocomp_initialize($ENV{ROBOCOMP}/)
robocomp_idsl_to_ice(Lidar3D Camera360RGB GenericBase OmniRobot MNIST)
robocomp_ice_to_src(Lidar3D Camera360RGB GenericBase OmniRobot MNIST)

# Output directory (still used for install, but binary goes to CMAKE_RUNTIME_OUTPUT_DIRECTORY)
set(EXECUTABLE_OUTPUT_PATH ${RC_COMPONENT_DEVEL_PATH}/bin)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../src)

# Build executable
add_executable(localiser
    ${SOURCES}
    ${MOC_SOURCES}
    ${RC_SOURCES}
    ${UI_HEADERS}
)

target_link_libraries(localiser
    ${LIBS}
    ${STATIC_LIBS}
    ${SPECIFIC_LIBS}
    ${QT_LIBRARIES}
    ${Ice_LIBRARIES}
)

set_target_properties(localiser PROPERTIES
    CXX_STANDARD ${CXX_STANDARD}
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# Install rule
install(
    FILES ${EXECUTABLE_OUTPUT_PATH}/localiser
    DESTINATION ${RC_COMPONENT_INSTALL_PATH}/bin/
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                GROUP_READ GROUP_EXECUTE
                WORLD_READ WORLD_EXECUTE
)
