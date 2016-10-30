#-------------------------------------------------
#
# Project created by QtCreator 2016-08-24T12:57:31
#
#-------------------------------------------------

QT       -= core gui

TARGET = module_RTA_NASAPhase2
TEMPLATE = lib

DEFINES += MODULE_RTA_NASAPHASE2_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES += module_rta_nasaphase2.cpp

HEADERS += module_rta_nasaphase2.h\
        module_rta_nasaphase2_global.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/module_RTA_NASAPhase2.lib release/module_RTA_NASAPhase2.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/module_RTA_NASAPhase2.lib debug/module_RTA_NASAPhase2.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/module_RTA_NASAPhase2
headers.files   += $$HEADERS
INSTALLS       += headers

INCLUDEPATH += $$(MACE_ROOT)/include

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core


unix{
    EigenInclude = $$system(pkg-config --cflags eigen3)
    EigenInclude = $$replace(EigenInclude, "-I", "")/eigen3
    INCLUDEPATH += $$EigenInclude
}
win32{
    INCLUDEPATH += "C:\Program Files (x86)\Eigen\include\eigen3"
}


INCLUDEPATH += $$PWD/../
DEPENDPATH += $$PWD/../
