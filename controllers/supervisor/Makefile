# Makefile for Supervisor Controller

WEBOTS_HOME ?= /usr/local/webots
TARGET = supervisor

# Auto-detect Webots location if not set
ifeq ($(shell uname -s),Linux)
 ifndef WEBOTS_HOME
  ifneq ($(shell which webots 2> /dev/null),)
   WEBOTS_HOME=$(shell dirname $$(readlink -f $$(which webots)))
  else
   WEBOTS_HOME=/usr/local/webots
  endif
 endif
endif

ifeq ($(shell uname -s),Darwin)
 WEBOTS_HOME ?= /Applications/Webots.app
endif

SOURCES = supervisor.c
CC = gcc
CFLAGS = -Wall -I"$(WEBOTS_HOME)/include/controller/c"
LDFLAGS = -L"$(WEBOTS_HOME)/lib/controller" -lController -lm

ifeq ($(shell uname -s),Linux)
 LDFLAGS += -Wl,-rpath,"$(WEBOTS_HOME)/lib/controller"
endif

ifeq ($(shell uname -s),Darwin)
 LDFLAGS += -Wl,-rpath,@executable_path/../lib/controller
endif

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CC) $(CFLAGS) -o $(TARGET) $(SOURCES) $(LDFLAGS)

clean:
	rm -f $(TARGET) *.o *~


