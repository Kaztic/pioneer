# Webots Makefile for pioneer_controller
# Copyright 1996-2024 Cyberbotics Ltd.

### Generic Makefile for Webots controllers, libraries, physics plugins and robot windows.

# WEBOTS_HOME can be specified as an environment variable or as a Make argument
# Example: make WEBOTS_HOME=/usr/local/webots
ifndef WEBOTS_HOME
 ifeq ($(shell uname -s),Linux)
  # Try to find Webots in standard locations
  ifneq ($(shell which webots 2> /dev/null),)
   # Get the directory containing the webots binary, then go up one level
   WEBOTS_HOME=$(shell dirname $$(readlink -f $$(which webots)))
  else
   WEBOTS_HOME=/usr/local/webots
  endif
 endif
 ifeq ($(shell uname -s),Darwin)
  WEBOTS_HOME=/Applications/Webots.app
 endif
endif

### Controller name
TARGET = controller

### Source files
SOURCES = controller.c

### Compiler settings
CC = gcc
CFLAGS = -Wall -I"$(WEBOTS_HOME)/include/controller/c"

### Linker settings
LDFLAGS = -L"$(WEBOTS_HOME)/lib/controller" -lController -lm

### Platform-specific settings
ifeq ($(shell uname -s),Linux)
 LDFLAGS += -Wl,-rpath,"$(WEBOTS_HOME)/lib/controller"
endif

ifeq ($(shell uname -s),Darwin)
 LDFLAGS += -Wl,-rpath,@executable_path/../lib/controller
endif

### Build rules
.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CC) $(CFLAGS) -o $(TARGET) $(SOURCES) $(LDFLAGS)

clean:
	rm -f $(TARGET) *.o *~

# Help target
help:
	@echo "Makefile for Pioneer Controller"
	@echo ""
	@echo "Available targets:"
	@echo "  all    - Build the controller (default)"
	@echo "  clean  - Remove compiled files"
	@echo "  help   - Show this help message"
	@echo ""
	@echo "Usage: make [WEBOTS_HOME=/path/to/webots]"

