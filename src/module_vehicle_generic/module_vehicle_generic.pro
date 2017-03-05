#-------------------------------------------------
#
# Project created by QtCreator 2017-01-09T17:42:07
#
#-------------------------------------------------

QT       -= core gui

TARGET = module_vehicle_generic
TEMPLATE = lib

DEFINES += MODULE_VEHICLE_GENERIC_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES += module_vehicle_generic.cpp

HEADERS += module_vehicle_generic.h\
        module_vehicle_generic_global.h

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/module_vehicle_generic.lib release/module_vehicle_generic.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/module_vehicle_generic.lib debug/module_vehicle_generic.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/module_vehicle_generic
headers.files   += \
        module_vehicle_generic.h \
        module_vehicle_generic_global.h
INSTALLS       += headers

INCLUDEPATH += $$PWD/../

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../comms/release/ -lcomms
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../comms/debug/ -lcomms
else:unix: LIBS += -L$$OUT_PWD/../comms/ -lcomms

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item_topic/release/ -ldata_generic_state_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item_topic/debug/ -ldata_generic_state_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_state_item_topic/ -ldata_generic_state_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item/release/ -ldata_generic_state_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item/debug/ -ldata_generic_state_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_state_item/ -ldata_generic_state_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_generic_topic/release/ -ldata_vehicle_generic_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_generic_topic/debug/ -ldata_vehicle_generic_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_vehicle_generic_topic/ -ldata_vehicle_generic_topic


unix{
    EigenInclude = $$system(pkg-config --cflags eigen3)
    EigenInclude = $$replace(EigenInclude, "-I", "")/eigen3
    INCLUDEPATH += $$EigenInclude
}
win32{
    INCLUDEPATH += "C:\Program Files (x86)\Eigen\include\eigen3"
}
