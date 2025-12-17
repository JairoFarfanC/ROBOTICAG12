###############################################
# CMakeLists.txt for src/
###############################################

# Incluir OpenCV requerido por ImageProcessor
INCLUDE($ENV{ROBOCOMP}/cmake/modules/opencv4.cmake)

# Añadir librerías extra necesarias para Task 3
SET(LIBS ${LIBS} tbb Qt6PrintSupport)

# ============================
# SOURCES
# ============================
LIST(APPEND SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/specificworker.cpp
        ${CMAKE_CURRENT_LIST_DIR}/room_detector.cpp
        ${CMAKE_CURRENT_LIST_DIR}/ransac_line_detector.cpp
        ${CMAKE_CURRENT_LIST_DIR}/hungarian.cpp
        ${CMAKE_CURRENT_LIST_DIR}/door_detector.cpp
        ${CMAKE_CURRENT_LIST_DIR}/qcustomplot.cpp
        ${CMAKE_CURRENT_LIST_DIR}/time_series_plotter.cpp

)

# ============================
# HEADERS
# ============================
LIST(APPEND HEADERS
        ${CMAKE_CURRENT_LIST_DIR}/specificworker.h
        ${CMAKE_CURRENT_LIST_DIR}/room_detector.h
        ${CMAKE_CURRENT_LIST_DIR}/ransac_line_detector.h
        ${CMAKE_CURRENT_LIST_DIR}/hungarian.h
        ${CMAKE_CURRENT_LIST_DIR}/door_detector.h
        ${CMAKE_CURRENT_LIST_DIR}/munkres.hpp
        ${CMAKE_CURRENT_LIST_DIR}/common_types.h
        ${CMAKE_CURRENT_LIST_DIR}/image_processor.h
        ${CMAKE_CURRENT_LIST_DIR}/nominal_room.h
        ${CMAKE_CURRENT_LIST_DIR}/qcustomplot.h
        ${CMAKE_CURRENT_LIST_DIR}/time_series_plotter.h
)

# ============================
# UI FILES
# ============================
LIST(APPEND UI_HEADERS
        ${CMAKE_CURRENT_LIST_DIR}/mainUI.ui
)