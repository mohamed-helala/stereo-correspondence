#-------------------------------------------------
#
# Project created by QtCreator 2014-05-05T21:17:52
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = Optical-Flow
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app



SOURCES += \
    kdtree.c \
    SLIC.cpp \
    writeMat.cpp \
    util.cpp \
    volanalysis.cpp \
    main.cpp \
    guidedfilter.cpp

HEADERS += \
    kdtree.h \
    params.h \
    SLIC.h \
    writeMat.h \
    util.h \
    volanalysis.h \
    main.h \
    guidedfilter.h

win32:CONFIG(debug, debug): LIBS += -L/usr/local/lib//release/  -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_ts -lopencv_video -lopencv_videostab -ldl
else:win32:CONFIG(debug, debug): LIBS += -L/usr/local/lib/  -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_ts -lopencv_video -lopencv_videostab -ldl
else:symbian: LIBS += -lopencv_calib3d -lopencv_contrib  -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_ts -lopencv_video -lopencv_videostab -ldl
else:unix:CONFIG(debug, debug):: LIBS += -L/usr/local/lib/ -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_ts -lopencv_video -lopencv_videostab -ldl
