#-------------------------------------------------
#
# Project created by QtCreator 2016-08-24T11:42:27
#
#-------------------------------------------------

QT       -= core gui

TARGET = module_path_planning_NASAPhase2
TEMPLATE = lib

DEFINES += MODULE_PATH_PLANNING_NASAPHASE2_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES += module_path_planning_nasaphase2.cpp

HEADERS += module_path_planning_nasaphase2.h \
    module_path_planning_nasaphase2_global.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

INCLUDEPATH += $$PWD/../
DEPENDPATH += $$PWD/../
