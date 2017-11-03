set (HEADERS
	 qcustomplot.h)

QT4_WRAP_CPP(MOCSrcs qcustomplot.h)

add_library (custom_plot SHARED qcustomplot.cpp ${MOCSrcs})

target_link_libraries (custom_plot ${ADDON_LIBRARIES} ${QT_LIBRARIES})
