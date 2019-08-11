################################################################################
## ESP32
ifndef IDF_PATH
	$(error If building for ESP32, you must supply the IDF_PATH variable.)
endif

BUILD_ROOT           := $(shell pwd)

MANUVR_OPTIONS += -D__MANUVR_ESP32

# TODO: Doesn't work. Never did. Fix.
# Debugging options...
#ifeq ($(DEBUG),1)
#  MANUVR_OPTIONS += -DMANUVR_DEBUG
#  #MANUVR_OPTIONS += -DMANUVR_PIPE_DEBUG
#  MANUVR_OPTIONS += -DMANUVR_IMU_DEBUG
#  MANUVR_OPTIONS += -DMANUVR_EVENT_PROFILER
#endif
#export MANUVR_OPTIONS

export MANUVR_PLATFORM  = ESP32
export OUTPUT_PATH      = $(BUILD_ROOT)/build

CXXFLAGS += $(MANUVR_OPTIONS)

PROJECT_NAME         := radio-relay
BUILD_DIR_BASE       := $(OUTPUT_PATH)
EXTRA_COMPONENT_DIRS := $(BUILD_ROOT)/lib/ManuvrOS/ManuvrOS
COMPONENT_EXTRA_INCLUDES := $(PROJECT_PATH)/lib


# Pull in the esp-idf...
include $(IDF_PATH)/make/project.mk
