TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    SendSerial.cpp \
    Camera.cpp \
    controller.cpp

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += opencv

HEADERS += \
    SendSerial.h \
    Camera.h \
    controller.h
