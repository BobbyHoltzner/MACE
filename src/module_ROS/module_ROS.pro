#-------------------------------------------------
#
# Project created by QtCreator 2016-08-24T11:42:27
#
#-------------------------------------------------

QT       -= core gui

TARGET = module_ROS
TEMPLATE = lib

DEFINES += MODULE_ROS_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES += module_ROS.cpp

HEADERS += module_ROS.h \
    module_ROS_global.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/module_ROS.lib release/module_ROS.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/module_ROS.lib debug/module_ROS.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/module_ROS
headers.files   += $$HEADERS
INSTALLS       += headers

INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/
INCLUDEPATH += $$(MACE_ROOT)/include
INCLUDEPATH += $$PWD/../
DEPENDPATH += $$PWD/../

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3

unix {
exists(/opt/ros/indigo/lib/) {
    DEFINES += ROS_EXISTS
    INCLUDEPATH += /opt/ros/indigo/include
    INCLUDEPATH += /opt/ros/indigo/lib

        LIBS += -L/opt/ros/indigo/lib -lroscpp
        LIBS += -L/opt/ros/indigo/lib -lroscpp_serialization
        LIBS += -L/opt/ros/indigo/lib -lrostime
        LIBS += -L/opt/ros/indigo/lib -lxmlrpcpp
        LIBS += -L/opt/ros/indigo/lib -lcpp_common
        LIBS += -L/opt/ros/indigo/lib -lrosconsole_log4cxx
        LIBS += -L/opt/ros/indigo/lib -lrosconsole_backend_interface
        LIBS += -L/opt/ros/indigo/lib -lroslib
        LIBS += -L/opt/ros/indigo/lib -lrospack
        LIBS += -L/opt/ros/indigo/lib -lmessage_filters
        LIBS += -L/opt/ros/indigo/lib -lclass_loader
        LIBS += -L/opt/ros/indigo/lib -lconsole_bridge
        LIBS += -L/opt/ros/indigo/lib -lrosconsole
        LIBS += -L/opt/ros/indigo/lib -limage_transport
        LIBS += -L/opt/ros/indigo/lib -lcv_bridge

}
}
