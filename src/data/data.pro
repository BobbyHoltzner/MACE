#-------------------------------------------------
#
# Project created by QtCreator 2016-12-08T21:09:09
#
#-------------------------------------------------

QT       -= core gui

TARGET = data
TEMPLATE = lib

DEFINES += DATA_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES += \
    global_position.cpp \
    local_position.cpp \
    positional_aid.cpp \
    data_aid.cpp

HEADERS += data_global.h \
    global_position.h \
    local_position.h \
    positional_aid.h \
    data_aid.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/data.lib release/data.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/data.lib debug/data.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/data
headers.files   += $$HEADERS
INSTALLS       += headers

INCLUDEPATH += $$PWD/../


unix{
    EigenInclude = $$system(pkg-config --cflags eigen3)
    EigenInclude = $$replace(EigenInclude, "-I", "")/eigen3
    INCLUDEPATH += $$EigenInclude
}
win32{
    INCLUDEPATH += "C:\Program Files (x86)\Eigen\include\eigen3"
}
