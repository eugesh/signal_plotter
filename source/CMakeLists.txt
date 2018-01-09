include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)

include_directories ("${CMAKE_CURRENT_SOURCE_DIR}/source")
include_directories ("${CMAKE_CURRENT_SOURCE_DIR}/custom_plot")

set (HEADERS
	 main_window.h)

QT4_WRAP_UI(UISrcs ${CMAKE_CURRENT_SOURCE_DIR}/../form_ui/mainwindow.ui)

# QT4_WRAP_CPP(MOCSrcs HEADERS)
QT4_WRAP_CPP(MOCSrcs main_window.h)


add_executable (scanline_viewer
	            main_window.cpp
	            main.cpp
	            signals_eval.cpp
	            #main_module.cpp
	            ${MOCSrcs} ${UISrcs})

target_link_libraries (scanline_viewer ${ADDON_LIBRARIES} ${QTLIBS} ${QT_LIBRARIES} custom_plot) #${OpenCV_LIBS} 

