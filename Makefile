OBJ_DIR = obj
SRC = pathFollower.cpp pathfollower.c
OBJ = $(OBJ_DIR)/pathFollower.o $(OBJ_DIR)/pathfollower.o

PREFIX = /usr/local
TARGET = libpathfollower.so
RELEASE_DIR = bin
RELEASE_DEBUG_DIR = binDebug
EXEC =$(RELEASE_DIR)/$(TARGET)
EXEC_DEBUG =$(RELEASE_DEBUG_DIR)/$(TARGET)

CC = g++
CFLAGS = -Wall -Werror -Wextra -O2 -std=c++11
LDFLAGS = -lrobotdriver -fPIC -shared
LDFLAGS_DEBUG = $(LDFLAGS) -g

.PHONY: all test debug clean createDir install

all: createDir $(EXEC)
test: test.c createDir $(EXEC)
	$(CC) -o $(RELEASE_DIR)/$@ $< $(CFLAGS) -lpathfollower -lrobotdriver -lrobot
test-cpp: test.cpp createDir $(EXEC)
	$(CC) -o $(RELEASE_DIR)/$@ $< $(CFLAGS) -lpathfollower -lrobotdriver
debug: createDir $(EXEC_DEBUG)

$(EXEC): $(OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)
$(EXEC_DEBUG): $(OBJ)
	$(CC) -o $@ $^ $(LDFLAGS_DEBUG)

$(OBJ_DIR)/%.o: %.cpp %.hpp
	$(CC) -o $@ -c $< $(CFLAGS)
$(OBJ_DIR)/%.o: %.c %.h
	$(CC) -o $@ -c $< $(CFLAGS)

createDir:
	@mkdir -p $(OBJ_DIR)
	@mkdir -p $(RELEASE_DIR)
	@mkdir -p $(RELEASE_DEBUG_DIR)

clean :
	@rm -rf $(OBJ_DIR)
	@rm -rf $(RELEASE_DIR)
	@rm -rf $(RELEASE_DEBUG_DIR)

install: $(EXEC)
	mkdir -p $(DESTDIR)$(PREFIX)/lib
	mkdir -p $(DESTDIR)$(PREFIX)/include/pathfollower/
	cp $(EXEC) $(DESTDIR)$(PREFIX)/lib/
	cp pathfollower.h $(DESTDIR)$(PREFIX)/include/pathfollower/
	cp pathFollower.hpp $(DESTDIR)$(PREFIX)/include/pathfollower/
	chmod 0755 $(DESTDIR)$(PREFIX)/lib/$(TARGET)
	ldconfig
	ldconfig -p | grep pathfollower
