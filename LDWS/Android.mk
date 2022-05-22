LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_CFLAGS  := -O3 -mfloat-abi=softfp -ftree-vectorize -mfpu=neon -ffast-math -funroll-all-loops
LOCAL_LDLIBS  += -llog -lz -lm -lc

ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
	LOCAL_CFLAGS += -march=armv7-a -mtune=cortex-a7
    LOCAL_ARM_NEON := true
endif

ifeq ($(TARGET_ARCH_ABI),arm64-v8a)
	LOCAL_CFLAGS += -march=armv8-a -mtune=cortex-a53
    LOCAL_ARM_NEON := true
endif

LOCAL_MODULE    := LDWS

LOCAL_SRC_FILES :=Sauvegarde.c \
				   Road_Tracker.c \
				   Recherche.c \
				   Points.c \
				   Mise_A_Jour.c \
				   Median.c \
				   Matrice.c \
				   LDWS_Interface.c \
				   Init_Struct.c \
				   Initialisation.c \
				   Detection_Zone.c \
				   Classement_Zone.c \
				   Caractere.c \
				   Bspline.c \
				   AMF.c
LOCAL_C_INCLUDES := \
$(LOCAL_PATH) \
$(APP_MODULE_LIB_PATH)/LDWS \
$(APP_MODULE_LIB_PATH)/NE10/inc \
$(APP_MODULE_LIB_PATH)/NE10/common \

LOCAL_STATIC_LIBRARIES := cpufeatures NE10
include $(BUILD_STATIC_LIBRARY)

$(call import-module,android/cpufeatures)
