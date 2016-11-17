#-------------------------------------------------
#
# Project created by QtCreator 2016-11-09T11:52:32
#
#-------------------------------------------------
QT += core
QT       = core gui

TARGET = data_ardupilot
TEMPLATE = lib

DEFINES += DATA_ARDUPILOT_LIBRARY

SOURCES += data_ardupilot.cpp \
    ardupilot_flightmode.cpp \
    ardupilot_status.cpp \
    ardupilot_attitude.cpp \
    ardupilot_gps_status.cpp \
    ardupilot_position.cpp

HEADERS += data_ardupilot.h\
        data_ardupilot_global.h \
    ardupilot_flightmode.h \
    ardupilot_status.h \
    ardupilot_attitude.h \
    ardupilot_gps_status.h \
    ardupilot_position.h

INCLUDEPATH += $$PWD/../../mavlink_cpp/V2/ardupilotmega

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

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_MAVLINK/release/ -lmodule_vehicle_MAVLINK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_MAVLINK/debug/ -lmodule_vehicle_MAVLINK
else:unix: LIBS += -L$$OUT_PWD/../module_vehicle_MAVLINK/ -lmodule_vehicle_MAVLINK

unix{
    EigenInclude = $$system(pkg-config --cflags eigen3)
    EigenInclude = $$replace(EigenInclude, "-I", "")/eigen3
    INCLUDEPATH += $$EigenInclude
}
win32{
    INCLUDEPATH += "C:\Program Files (x86)\Eigen\include\eigen3"
}



