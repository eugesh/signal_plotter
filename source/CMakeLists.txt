include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)

include_directories ("${CMAKE_SOURCE_DIR}/source")
include_directories ("${CMAKE_SOURCE_DIR}/custom_plot")

set (HEADERS
	 main_window.h)

QT5_WRAP_UI(UISrcs ${CMAKE_CURRENT_SOURCE_DIR}/../form_ui/mainwindow.ui)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable (scanline_viewer
	            main_window.cpp
	            main.cpp
	            ${MOCSrcs} ${UISrcs})

target_link_libraries (scanline_viewer ${QTLIBS} custom_plot)
