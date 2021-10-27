include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_SOURCE_DIR}/form_ui
)

include_directories ("${CMAKE_SOURCE_DIR}/source")
include_directories ("${CMAKE_SOURCE_DIR}/custom_plot")
link_directories ("${CMAKE_SOURCE_DIR}/bin")

set (HEADERS main_window.h)
QT5_WRAP_UI(UISrcs ${CMAKE_CURRENT_SOURCE_DIR}/../form_ui/mainwindow.ui)
#QT4_WRAP_UI(UISrcs ${CMAKE_CURRENT_SOURCE_DIR}/../form_ui/mainwindow.ui)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# QT4_WRAP_CPP(MOCSrcs HEADERS)
#QT4_WRAP_CPP(MOCSrcs main_window.h)


add_executable (scanline_viewer
	            main_window.cpp
	            main.cpp
	            signals_eval.cpp
                    ../custom_plot/qcustomplot.cpp
	            ${MOCSrcs} ${UISrcs})

if(CMAKE_VERSION VERSION_LESS "3.0")
    target_link_libraries (scanline_viewer ${ADDON_LIBRARIES} ${QTLIBS} ${QT_LIBRARIES} custom_plot -lfftw3f) # Eigen3::Eigen ${OpenCV_LIBS}
else()
    target_link_libraries (scanline_viewer ${ADDON_LIBRARIES} ${QTLIBS} ${QT_LIBRARIES} -lcustom_plot -lfftw3f)# Eigen3::Eigen)
endif() 

