#-------------------------------------------------
#
# Project created by QtCreator 2017-02-08T23:17:41
#
#-------------------------------------------------

QT       -= core gui

TARGET = TestCodeHolder
TEMPLATE = lib

DEFINES += TESTCODEHOLDER_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += testcodeholder.cpp \
    missionupload.cpp

HEADERS += testcodeholder.h\
        testcodeholder_global.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}
