LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE := FCWSDetector
LOCAL_SRC_FILES := $(NDK_PROJECT_PATH)/obj/local/armeabi/libFCWSDetector.a

include $(PREBUILT_STATIC_LIBRARY)