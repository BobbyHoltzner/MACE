#-------------------------------------------------
#
# Project created by QtCreator 2016-09-02T13:57:03
#
#-------------------------------------------------

QT       -= core gui

TARGET = common
TEMPLATE = lib

DEFINES += COMMON_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

SOURCES +=

HEADERS += common.h\
    enum_class_hash.h \
    publisher.h \
    class_forward.h \
    optional_parameter.h \
    fsm.h \
    pointer_collection.h

#Header file copy
headers.path    = $$(MACE_ROOT)/include/common
headers.files   += $$HEADERS
INSTALLS       += headers

INCLUDEPATH += $$(MACE_ROOT)/include

