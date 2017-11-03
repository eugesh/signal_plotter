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
	            ${MOCSrcs} ${UISrcs})

target_link_libraries (scanline_viewer ${ADDON_LIBRARIES} ${GLIB2_LIBRARIES} ${HYSCAN_LIBRARIES} ${QTLIBS} ${QT_LIBRARIES} ${GDAL_LIBRARY} custom_plot)

# set_target_properties (scanline_viewer PROPERTIES DEFINE_SYMBOL "HYSCAN_API_EXPORTS")
