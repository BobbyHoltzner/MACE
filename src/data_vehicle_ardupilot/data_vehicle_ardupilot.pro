#-------------------------------------------------
#
# Project created by QtCreator 2017-01-12T13:32:38
#
#-------------------------------------------------

QT       -= core gui

TARGET = data_vehicle_ardupilot
TEMPLATE = lib

DEFINES += DATA_VEHICLE_ARDUPILOT_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES += \
    components/vehicle_operating_parameters.cpp \
    components/vehicle_operating_status.cpp \
    components/vehicle_operating_attitude.cpp

HEADERS +=\
        data_vehicle_ardupilot_global.h \
    mavlink_parser_ardupilot.h \
    ardu_platforms.h \
    components/vehicle_operating_parameters.h \
    components.h \
    components/vehicle_operating_status.h \
    components/vehicle_operating_attitude.h

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/data_vehicle_ardupilot.lib release/data_vehicle_ardupilot.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/data_vehicle_ardupilot.lib debug/data_vehicle_ardupilot.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/data_vehicle_ardupilot
headers.files   += \
        ardu_platforms.h \
        components.h \
        mavlink_parser_ardupilot.h \
        data_vehicle_ardupilot_global.h
INSTALLS       += headers

#Header file copy
headers_Components.path    = $$(MACE_ROOT)/include/data_vehicle_ardupilot/Components
headers_Components.files   += \
        components/vehicle_operating_parameters.h \
        components/vehicle_operating_status.h \
        components/vehicle_operating_attitude.h

INSTALLS       += headers_Components



INCLUDEPATH += $$PWD/../../mavlink_cpp/V2/ardupilotmega

INCLUDEPATH += $$PWD/../

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_generic/release/ -ldata_vehicle_generic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_generic/debug/ -ldata_vehicle_generic
else:unix: LIBS += -L$$OUT_PWD/../data_vehicle_generic/ -ldata_vehicle_generic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_MAVLINK/release/ -ldata_vehicle_MAVLINK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_MAVLINK/debug/ -ldata_vehicle_MAVLINK
else:unix: LIBS += -L$$OUT_PWD/../data_vehicle_MAVLINK/ -ldata_vehicle_MAVLINK
