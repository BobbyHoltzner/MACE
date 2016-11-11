#-------------------------------------------------
#
# Project created by QtCreator 2016-11-09T11:52:32
#
#-------------------------------------------------

QT       -= core gui

TARGET = data_ardupilot
TEMPLATE = lib

DEFINES += DATA_ARDUPILOT_LIBRARY

SOURCES += data_ardupilot.cpp

HEADERS += data_ardupilot.h\
        data_ardupilot_global.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/data_ardupilot.lib release/data_ardupilot.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/data_ardupilot.lib debug/data_ardupilot.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/data_ardupilot
headers.files   += $$HEADERS
INSTALLS       += headers

INCLUDEPATH += $$(MACE_ROOT)/include


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

INCLUDEPATH += $$PWD/../../mavlink_cpp/V2/ardupilotmega


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

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_MAVLINK/release/ -lmodule_vehicle_MAVLINK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_MAVLINK/debug/ -lmodule_vehicle_MAVLINK
else:unix: LIBS += -L$$OUT_PWD/../module_vehicle_MAVLINK/ -lmodule_vehicle_MAVLINK

INCLUDEPATH += $$PWD/../module_vehicle_MAVLINK
DEPENDPATH += $$PWD/../module_vehicle_MAVLINK
