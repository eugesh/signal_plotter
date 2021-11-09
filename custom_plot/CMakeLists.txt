set (HEADERS
	 qcustomplot.h)

include_directories ("${CMAKE_SOURCE_DIR}/custom_plot")
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library (custom_plot SHARED qcustomplot.cpp ${MOCSrcs})

target_link_libraries (custom_plot ${QTLIBS})
