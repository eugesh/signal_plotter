QT += core gui widgets printsupport

TARGET = scanline_viewer
TEMPLATE = app

INCLUDEPATH += custom_plot/
INCLUDEPATH += source/
windows: INCLUDEPATH += D:\soft\developerTools\github\eigen

SOURCES += \
    nivisadevice.cpp \
    nivisadeviceinfo.cpp \
    nivisagenerator.cpp \
    nivisaoscilloscope.cpp \
    source/main.cpp \
    source/main_window.cpp \
    source/signals_eval.cpp \
    custom_plot/qcustomplot.cpp

HEADERS += \
    nivisadevice.h \
    nivisadeviceinfo.h \
    nivisagenerator.h \
    nivisaoscilloscope.h \
    source/main_window.h \
    source/signals_eval.h \
    custom_plot/qcustomplot.h

FORMS += form_ui/mainwindow.ui


unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += eigen3
