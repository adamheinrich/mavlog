# Set to @ if you want to suppress command echo
CMD_ECHO = @

# Program name
BIN = mavlog

# Important directories
MAVLINK_DIR = ../c_library_v1
BUILD_DIR = ../build

# Include paths
INC = -I. \
      -I$(MAVLINK_DIR)/common

# C ource files outside src directory
SRC_C =

# Defines required by included libraries
DEF =
#DEF += -DDEBUG

# Compiler and linker flags
ARCHFLAGS =
OPTFLAGS = -O3 -flto
DBGFLAGS = -ggdb

# CC: Place functions and data into separate sections to allow dead code removal
# by the linker (-f*-sections).
CFLAGS = $(ARCHFLAGS) $(DBGFLAGS) $(OPTFLAGS) -std=gnu99 -Wall -Wno-format \
         -Wno-error=unused-function -Wno-error=unused-variable \
         -ffunction-sections -fdata-sections

# LD: Remove unused sections, link with MMAL, generate map
LDFLAGS = $(ARCHFLAGS) $(DBGFLAGS) $(OPTFLAGS) -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(BIN).map
LDLIBFLAGS = -Wl,--start-group -lpthread -Wl,--end-group

# Generate object list from source files and add their dirs to search path
SRC_C += $(wildcard *.c)
FILENAMES_C = $(notdir $(SRC_C))
OBJS_C = $(addprefix $(BUILD_DIR)/, $(FILENAMES_C:.c=.o))
vpath %.c $(dir $(SRC_C))

# Tool selection
CC = gcc
LD = gcc
NM = nm
OBJCOPY = objcopy
OBJDUMP = objdump
SIZE = size

all: $(BUILD_DIR) $(BUILD_DIR)/$(BIN)
	@echo ""
	$(CMD_ECHO) @$(SIZE) $(BUILD_DIR)/$(BIN)

$(BUILD_DIR):
	$(CMD_ECHO) mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: %.c
	@echo "Compiling C file: $(notdir $<)"
	$(CMD_ECHO) $(CC) $(CFLAGS) $(DEF) $(INC) -c -o $@ $<

$(BUILD_DIR)/$(BIN): $(OBJS_C)
	@echo "Linking binary: $(notdir $@)"
	$(CMD_ECHO) $(LD) $(LDFLAGS) -o $@ $^ $(LDLIBFLAGS)

clean:
	rm -f $(BUILD_DIR)/$(BIN) $(BUILD_DIR)/*.map $(BUILD_DIR)/*.o
