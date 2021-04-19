QT += core gui widgets printsupport

TARGET = png_2Dviewer
TEMPLATE = app

INCLUDEPATH += custom_plot/
INCLUDEPATH += source/

SOURCES += \
    source/main.cpp \
    source/main_window.cpp \
    custom_plot/qcustomplot.cpp

HEADERS += \
    source/main_window.h \
    custom_plot/qcustomplot.h

FORMS += form_ui/mainwindow.ui


#unix: CONFIG += link_pkgconfig
#unix: PKGCONFIG += eigen3
