#-------------------------------------------------
#
# Project created by QtCreator 2016-10-04T03:38:02
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = LineFollowerSimulator
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    arduino.cpp \
    route.cpp \
    robot.cpp \
    utils.cpp \
    robotitem.cpp \
    pid.cpp

HEADERS  += mainwindow.h \
    arduino.h \
    route.h \
    robot.h \
    utils.h \
    robotitem.h \
    pid.h

FORMS    += mainwindow.ui

CONFIG += c++11
