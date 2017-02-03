#-------------------------------------------------
#
# Project created by QtCreator 2017-01-10T14:37:24
#
#-------------------------------------------------

QT       -= core gui

TARGET = data_vehicle_MAVLINK
TEMPLATE = lib

DEFINES += DATA_VEHICLE_MAVLINK_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES += \
    mavlink_parser.cpp \
    Components/gps_status.cpp

HEADERS +=\
    altitude_reference_frames.h \
    mavlink_parser.h \
    Components/gps_status.h \
    components.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/data_vehicle_MAVLINK.lib release/data_vehicle_MAVLINK.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/data_vehicle_MAVLINK.lib debug/data_vehicle_MAVLINK.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/data_vehicle_MAVLINK
headers.files   += \
        mavlink_parser.h \
        components.h \
        altitude_reference_frames.h
INSTALLS       += headers

#Header file copy
headers_Components.path    = $$(MACE_ROOT)/include/data_vehicle_MAVLINK/Components
headers_Components.files   += \
        Components/gps_status.h
INSTALLS       += headers_Components


INCLUDEPATH += $$PWD/../../mavlink_cpp/V2/common/

INCLUDEPATH += $$PWD/../

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_topics/release/ -ldata_generic_state_topics
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_topics/debug/ -ldata_generic_state_topics
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_state_topics/ -ldata_generic_state_topics



