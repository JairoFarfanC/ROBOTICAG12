set(SOURCES
        ../src/specificworker.cpp
        ../src/door_detector.cpp
        ../src/hungarian.cpp
        ../src/ransac_line_detector.cpp
        ../src/room_detector.cpp
        ../src/time_series_plotter.cpp
        ../src/qcustomplot.cpp
)

set(HEADERS
        ../src/specificworker.h
        ../src/door_detector.h
        ../src/hungarian.h
        ../src/ransac_line_detector.h
        ../src/room_detector.h
        ../src/time_series_plotter.h
        ../src/qcustomplot.h
)

list(APPEND LIBS ${LIBS})

add_definitions(-fmax-errors=1 -fno-char8_t)