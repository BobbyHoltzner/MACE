TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
QT += serialport
QT += network

TARGET = MACE
QMAKE_CXXFLAGS += -std=c++11


SOURCES += main.cpp \
    data_interpolation.cpp \
    configuration_reader_xml.cpp \
    pugixml.cpp


HEADERS += \
    data_interpolation.h \
    configuration_reader_xml.h \
    pugixml.hpp \
    pugiconfig.hpp \
    module_collection.h


# Copy Files
target.path = $$(MACE_ROOT)/bin
INSTALLS += target

INCLUDEPATH += $$(MACE_ROOT)/include

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_path_planning_NASAPhase2/release/ -lmodule_path_planning_NASAPhase2
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_path_planning_NASAPhase2/debug/ -lmodule_path_planning_NASAPhase2
else:unix: LIBS += -L$$OUT_PWD/../module_path_planning_NASAPhase2/ -lmodule_path_planning_NASAPhase2


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_MAVLINK/release/ -lmodule_vehicle_MAVLINK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_MAVLINK/debug/ -lmodule_vehicle_MAVLINK
else:unix: LIBS += -L$$OUT_PWD/../module_vehicle_MAVLINK/ -lmodule_vehicle_MAVLINK


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_RTA_NASAPhase2/release/ -lmodule_RTA_NASAPhase2
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_RTA_NASAPhase2/debug/ -lmodule_RTA_NASAPhase2
else:unix: LIBS += -L$$OUT_PWD/../module_RTA_NASAPhase2/ -lmodule_RTA_NASAPhase2

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../comms/release/ -lcomms
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../comms/debug/ -lcomms
else:unix: LIBS += -L$$OUT_PWD/../comms/ -lcomms


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_ground_station/release/ -lmodule_ground_station
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_ground_station/debug/ -lmodule_ground_station
else:unix:!macx: LIBS += -L$$OUT_PWD/../module_ground_station/ -lmodule_ground_station


INCLUDEPATH += $$PWD/../

INCLUDEPATH += $$PWD/../../mavlink_cpp/V2/ardupilotmega


unix{
    EigenInclude = $$system(pkg-config --cflags eigen3)
    EigenInclude = $$replace(EigenInclude, "-I", "")/eigen3
    INCLUDEPATH += $$EigenInclude
}
win32{
    INCLUDEPATH += "C:\Program Files (x86)\Eigen\include\eigen3"
}
