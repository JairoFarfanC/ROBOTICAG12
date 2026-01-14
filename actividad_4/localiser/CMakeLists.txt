cmake_minimum_required(VERSION 3.10)
project(localiser)

if("$ENV{ROBOCOMP}" STREQUAL "")
    message(WARNING "ROBOCOMP variable not set. Using the default value: /home/robocomp/robocomp")
    set(ENV{ROBOCOMP} "/home/robocomp/robocomp/")
endif()

set(RC_COMPONENT_PATH ${CMAKE_CURRENT_SOURCE_DIR} CACHE STRING "component base path" FORCE)
set(RC_COMPONENT_DEVEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "component binary path" FORCE)
set(RC_COMPONENT_INSTALL_PATH $ENV{ROBOCOMP} CACHE STRING "component install path")

# Incluir headers
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/generated
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${RC_COMPONENT_INSTALL_PATH}/classes
)

# Encontrar Ice y Qt
find_package(Ice REQUIRED COMPONENTS Ice++11)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Añadir subdirectorio generado
add_subdirectory(generated)

# Fuentes del src
set(SRC_SOURCES
    src/specificworker.cpp
)

# Executable
add_executable(localiser
    ${SRC_SOURCES}
    $<TARGET_OBJECTS:localiser_generated>
)

# Enlazar librerías
target_link_libraries(localiser
    ${Ice_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    ${RC_COMPONENT_INSTALL_PATH}/lib/librapplication.so
    ${RC_COMPONENT_INSTALL_PATH}/lib/libConfigLoader.so
    ${RC_COMPONENT_INSTALL_PATH}/lib/libsigwatch.so
)

# Instalación
install(TARGETS localiser DESTINATION ${RC_COMPONENT_INSTALL_PATH}/bin)
install(FILES etc/config DESTINATION ${RC_COMPONENT_INSTALL_PATH}/etc-default/ RENAME localiser.conf)
