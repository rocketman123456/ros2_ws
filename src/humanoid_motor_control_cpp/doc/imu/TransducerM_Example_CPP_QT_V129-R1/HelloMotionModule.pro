#-------------------------------------------------
#
# Project created by QtCreator 2016-07-26T16:36:39
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

QT += serialport
QT += testlib

TARGET = HelloMotionModule
TEMPLATE = app



SOURCES += main.cpp\
        HelloMotionModule.cpp \
    EasyProfile/EasyObjectDictionary.cpp \
    EasyProfile/EasyProfile.cpp \
    EasyProfile/EasyProtocol.cpp

HEADERS  += HelloMotionModule.h \
    EasyProfile/EasyObjectDictionary.h \
    EasyProfile/EasyProfile.h \
    EasyProfile/EasyProtocol.h \
    EasyProfile/EasyQueue.h \
    EasyProfile/BasicTypes.h

FORMS    += HelloMotionModule.ui
