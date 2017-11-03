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

SOURCES += module_ROS.cpp \
    rosTimer.cpp

HEADERS += module_ROS.h \
    module_ROS_global.h \
    rosTimer.h

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

INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/include
INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
DEPENDPATH += $$PWD/../

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base_topic/release/ -lbase_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base_topic/debug/ -lbase_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../base_topic/ -lbase_topic

unix {
exists(/opt/ros/kinetic/lib/) {
    DEFINES += ROS_EXISTS
    INCLUDEPATH += /opt/ros/kinetic/include
    INCLUDEPATH += /opt/ros/kinetic/lib

        LIBS += -L/opt/ros/kinetic/lib -lroscpp
        LIBS += -L/opt/ros/kinetic/lib -lroscpp_serialization
        LIBS += -L/opt/ros/kinetic/lib -lrostime
        LIBS += -L/opt/ros/kinetic/lib -lxmlrpcpp
        LIBS += -L/opt/ros/kinetic/lib -lcpp_common
        LIBS += -L/opt/ros/kinetic/lib -lrosconsole_log4cxx
        LIBS += -L/opt/ros/kinetic/lib -lrosconsole_backend_interface
        LIBS += -L/opt/ros/kinetic/lib -lroslib
        LIBS += -L/opt/ros/kinetic/lib -lrospack
        LIBS += -L/opt/ros/kinetic/lib -lmessage_filters
        LIBS += -L/opt/ros/kinetic/lib -lclass_loader
        LIBS += -L/opt/ros/kinetic/lib -lconsole_bridge
        LIBS += -L/opt/ros/kinetic/lib -lrosconsole
        LIBS += -L/opt/ros/kinetic/lib -limage_transport
        LIBS += -L/opt/ros/kinetic/lib -lcv_bridge
}
}

