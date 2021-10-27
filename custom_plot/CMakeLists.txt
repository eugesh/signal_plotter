set (HEADERS
	 qcustomplot.h)

#QT4_WRAP_CPP(MOCSrcs qcustomplot.h)

set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
#set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library (custom_plot SHARED qcustomplot.cpp ${MOCSrcs})

target_link_libraries (custom_plot ${ADDON_LIBRARIES} ${QTLIBS})# ${QT_LIBRARIES})
