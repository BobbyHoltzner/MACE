#-------------------------------------------------
#
# Project created by QtCreator 2017-07-23T07:59:51
#
#-------------------------------------------------

QT       -= core gui

TARGET = data_interface_MACE
TEMPLATE = lib

DEFINES += DATA_INTERFACE_MACE_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += data_interface_mace.cpp \
    MACE_to_COMMS/helper_mission_mace_to_comms.cpp \
    COMMS_to_MACE/helper_mission_comms_to_mace.cpp

HEADERS += data_interface_mace.h\
        data_interface_mace_global.h \
    generic/helper_previous_command_mace.h \
    generic/helper_previous_guided_mace.h \
    generic/helper_previous_transmission_mace.h \
    generic/helper_previous_transmission_base_mace.h \
    MACE_to_COMMS/helper_mission_mace_to_comms.h \
    COMMS_to_MACE/helper_mission_comms_to_mace.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/data_interface_MACE.lib release/data_interface_MACE.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/data_interface_MACE.lib debug/data_interface_MACE.dll
INSTALLS += lib


#Header file copy
headers.path    = $$(MACE_ROOT)/include/data_interface_MACE
headers.files   += \
INSTALLS       += headers

#Header file copy
headers_generic.path    = $$(MACE_ROOT)/include/data_interface_MACE/generic
headers_generic.files   += \
    generic/helper_previous_command_mace.h \
    generic/helper_previous_guided_mace.h \
    generic/helper_previous_transmission_mace.h \
    generic/helper_previous_transmission_base_mace.h
INSTALLS       += headers_generic

#Header file copy
headers_MACE_to_COMMS.path    = $$(MACE_ROOT)/include/data_interface_MACE/MACE_to_COMMS
headers_MACE_to_COMMS.files   += \
    MACE_to_COMMS/helper_mission_mace_to_comms.h
INSTALLS       += headers_MACE_to_COMMS

#Header file copy
headers_COMMS_to_MACE.path    = $$(MACE_ROOT)/include/data_interface_MACE/COMMS_to_MACE
headers_COMMS_to_MACE.files   += \
    COMMS_to_MACE/helper_mission_comms_to_mace.h
INSTALLS       += headers_COMMS_to_MACE

INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$PWD/../../speedLog/
INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../comms/release/ -lcomms
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../comms/debug/ -lcomms
else:unix:!macx: LIBS += -L$$OUT_PWD/../comms/ -lcomms

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item/release/ -ldata_generic_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item/debug/ -ldata_generic_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_item/ -ldata_generic_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item_topic/release/ -ldata_generic_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item_topic/debug/ -ldata_generic_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_item_topic/ -ldata_generic_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item/release/ -ldata_generic_state_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item/debug/ -ldata_generic_state_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_state_item/ -ldata_generic_state_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item_topic/release/ -ldata_generic_state_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item_topic/debug/ -ldata_generic_state_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_state_item_topic/ -ldata_generic_state_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item/release/ -ldata_generic_command_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item/debug/ -ldata_generic_command_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_command_item/ -ldata_generic_command_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item_topic/release/ -ldata_generic_command_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item_topic/debug/ -ldata_generic_command_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_command_item_topic/ -ldata_generic_command_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/release/ -ldata_generic_mission_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/debug/ -ldata_generic_mission_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/ -ldata_generic_mission_item_topic
